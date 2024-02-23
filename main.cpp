// Copyright 2021 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <new>
#include <string>
#include <thread>

#include "array_safety.h"
#include "glfw_adapter.h"
#include "simulate.h"
#include <mujoco/mujoco.h>

#include "dynamics.hpp"
#include "estimate.h"
#include "iLQR.h"
#include "pd_controller.h"

#define MUJOCO_PLUGIN_DIR "mujoco_plugin"
#define pi 3.1416

extern "C" {
#if defined(_WIN32) || defined(__CYGWIN__)
#include <windows.h>
#else
#if defined(__APPLE__)
#include <mach-o/dyld.h>
#endif
#include <sys/errno.h>
#include <unistd.h>
#endif
}

namespace {
namespace mj = ::mujoco;
namespace mju = ::mujoco::sample_util;

// constants
const double syncMisalign =
    0.1; // maximum mis-alignment before re-sync (simulation seconds)
const double simRefreshFraction =
    0.7;                       // fraction of refresh available for simulation
const int kErrorLength = 1024; // load error string length

// model and data
mjModel *m = nullptr;
mjData *d = nullptr;

// control noise variables
mjtNum *ctrlnoise = nullptr;

using Seconds = std::chrono::duration<double>;

std::string yaml_name = "../controllers/iLQR/param.yaml";
Cartpole_iLQR iLQR(yaml_name);
H1Estm estimate;
H1State state;
FBDynModel _model;

//---------------------------------------- plugin handling
//-----------------------------------------

// return the path to the directory containing the current executable
// used to determine the location of auto-loaded plugin libraries
std::string getExecutableDir() {
#if defined(_WIN32) || defined(__CYGWIN__)
  constexpr char kPathSep = '\\';
  std::string realpath = [&]() -> std::string {
    std::unique_ptr<char[]> realpath(nullptr);
    DWORD buf_size = 128;
    bool success = false;
    while (!success) {
      realpath.reset(new (std::nothrow) char[buf_size]);
      if (!realpath) {
        std::cerr << "cannot allocate memory to store executable path\n";
        return "";
      }

      DWORD written = GetModuleFileNameA(nullptr, realpath.get(), buf_size);
      if (written < buf_size) {
        success = true;
      } else if (written == buf_size) {
        // realpath is too small, grow and retry
        buf_size *= 2;
      } else {
        std::cerr << "failed to retrieve executable path: " << GetLastError()
                  << "\n";
        return "";
      }
    }
    return realpath.get();
  }();
#else
  constexpr char kPathSep = '/';
#if defined(__APPLE__)
  std::unique_ptr<char[]> buf(nullptr);
  {
    std::uint32_t buf_size = 0;
    _NSGetExecutablePath(nullptr, &buf_size);
    buf.reset(new char[buf_size]);
    if (!buf) {
      std::cerr << "cannot allocate memory to store executable path\n";
      return "";
    }
    if (_NSGetExecutablePath(buf.get(), &buf_size)) {
      std::cerr << "unexpected error from _NSGetExecutablePath\n";
    }
  }
  const char *path = buf.get();
#else
  const char *path = "/proc/self/exe";
#endif
  std::string realpath = [&]() -> std::string {
    std::unique_ptr<char[]> realpath(nullptr);
    std::uint32_t buf_size = 128;
    bool success = false;
    while (!success) {
      realpath.reset(new (std::nothrow) char[buf_size]);
      if (!realpath) {
        std::cerr << "cannot allocate memory to store executable path\n";
        return "";
      }

      std::size_t written = readlink(path, realpath.get(), buf_size);
      if (written < buf_size) {
        realpath.get()[written] = '\0';
        success = true;
      } else if (written == -1) {
        if (errno == EINVAL) {
          // path is already not a symlink, just use it
          return path;
        }

        std::cerr << "error while resolving executable path: "
                  << strerror(errno) << '\n';
        return "";
      } else {
        // realpath is too small, grow and retry
        buf_size *= 2;
      }
    }
    return realpath.get();
  }();
#endif

  if (realpath.empty()) {
    return "";
  }

  for (std::size_t i = realpath.size() - 1; i > 0; --i) {
    if (realpath.c_str()[i] == kPathSep) {
      return realpath.substr(0, i);
    }
  }

  // don't scan through the entire file system's root
  return "";
}

// scan for libraries in the plugin directory to load additional plugins
void scanPluginLibraries() {
  // check and print plugins that are linked directly into the executable
  int nplugin = mjp_pluginCount();
  if (nplugin) {
    std::printf("Built-in plugins:\n");
    for (int i = 0; i < nplugin; ++i) {
      std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
    }
  }

  // define platform-specific strings
#if defined(_WIN32) || defined(__CYGWIN__)
  const std::string sep = "\\";
#else
  const std::string sep = "/";
#endif

  // try to open the ${EXECDIR}/plugin directory
  // ${EXECDIR} is the directory containing the simulate binary itself
  const std::string executable_dir = getExecutableDir();
  if (executable_dir.empty()) {
    return;
  }

  const std::string plugin_dir = getExecutableDir() + sep + MUJOCO_PLUGIN_DIR;
  mj_loadAllPluginLibraries(
      plugin_dir.c_str(), +[](const char *filename, int first, int count) {
        std::printf("Plugins registered by library '%s':\n", filename);
        for (int i = first; i < first + count; ++i) {
          std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
        }
      });
}

//------------------------------------------- simulation
//-------------------------------------------

mjModel *LoadModel(const char *file, mj::Simulate &sim) {
  // this copy is needed so that the mju::strlen call below compiles
  char filename[mj::Simulate::kMaxFilenameLength];
  mju::strcpy_arr(filename, file);

  // make sure filename is not empty
  if (!filename[0]) {
    return nullptr;
  }

  // load and compile
  char loadError[kErrorLength] = "";
  mjModel *mnew = 0;
  if (mju::strlen_arr(filename) > 4 &&
      !std::strncmp(filename + mju::strlen_arr(filename) - 4, ".mjb",
                    mju::sizeof_arr(filename) - mju::strlen_arr(filename) +
                        4)) {
    mnew = mj_loadModel(filename, nullptr);
    if (!mnew) {
      mju::strcpy_arr(loadError, "could not load binary model");
    }
  } else {
    mnew = mj_loadXML(filename, nullptr, loadError, kErrorLength);
    // remove trailing newline character from loadError
    if (loadError[0]) {
      int error_length = mju::strlen_arr(loadError);
      if (loadError[error_length - 1] == '\n') {
        loadError[error_length - 1] = '\0';
      }
    }
  }

  mju::strcpy_arr(sim.load_error, loadError);

  if (!mnew) {
    std::printf("%s\n", loadError);
    return nullptr;
  }

  // compiler warning: print and pause
  if (loadError[0]) {
    // mj_forward() below will print the warning message
    std::printf("Model compiled, but simulation warning (paused):\n  %s\n",
                loadError);
    sim.run = 0;
  }

  return mnew;
}

// normalize angle
void AngleNormalization(double &angle) {
  while (angle > pi)
    angle -= 2 * pi;
  while (angle < -pi)
    angle += 2 * pi;
}

// simulate in background thread (while rendering in main thread)
void PhysicsLoop(mj::Simulate &sim) {
  // cpu-sim syncronization point
  std::chrono::time_point<mj::Simulate::Clock> syncCPU;
  mjtNum syncSim = 0;

  // run until asked to exit
  while (!sim.exitrequest.load()) {
    if (sim.droploadrequest.load()) {
      sim.LoadMessage(sim.dropfilename);
      mjModel *mnew = LoadModel(sim.dropfilename, sim);
      sim.droploadrequest.store(false);

      mjData *dnew = nullptr;
      if (mnew)
        dnew = mj_makeData(mnew);
      if (dnew) {
        sim.Load(mnew, dnew, sim.dropfilename);

        mj_deleteData(d);
        mj_deleteModel(m);

        m = mnew;
        d = dnew;
        mj_forward(m, d);

        // allocate ctrlnoise
        free(ctrlnoise);
        ctrlnoise = (mjtNum *)malloc(sizeof(mjtNum) * m->nu);
        mju_zero(ctrlnoise, m->nu);
      } else {
        sim.LoadMessageClear();
      }
    }

    if (sim.uiloadrequest.load()) {
      sim.uiloadrequest.fetch_sub(1);
      sim.LoadMessage(sim.filename);
      mjModel *mnew = LoadModel(sim.filename, sim);
      mjData *dnew = nullptr;
      if (mnew)
        dnew = mj_makeData(mnew);
      if (dnew) {
        sim.Load(mnew, dnew, sim.filename);

        mj_deleteData(d);
        mj_deleteModel(m);

        m = mnew;
        d = dnew;
        mj_forward(m, d);

        // allocate ctrlnoise
        free(ctrlnoise);
        ctrlnoise = static_cast<mjtNum *>(malloc(sizeof(mjtNum) * m->nu));
        mju_zero(ctrlnoise, m->nu);
      } else {
        sim.LoadMessageClear();
      }
    }

    // sleep for 1 ms or yield, to let main thread run
    //  yield results in busy wait - which has better timing but kills battery
    //  life
    if (sim.run && sim.busywait) {
      std::this_thread::yield();
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    {
      // lock the sim mutex
      const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

      // run only if model is present
      if (m) {
        // running
        if (sim.run) {
          bool stepped = false;

          // record cpu time at start of iteration
          const auto startCPU = mj::Simulate::Clock::now();

          // elapsed CPU and simulation time since last sync
          const auto elapsedCPU = startCPU - syncCPU;
          double elapsedSim = d->time - syncSim;

          // inject noise
          if (sim.ctrl_noise_std) {
            // convert rate and scale to discrete time (Ornstein–Uhlenbeck)
            mjtNum rate = mju_exp(-m->opt.timestep /
                                  mju_max(sim.ctrl_noise_rate, mjMINVAL));
            mjtNum scale = sim.ctrl_noise_std * mju_sqrt(1 - rate * rate);

            for (int i = 0; i < m->nu; i++) {
              // update noise
              ctrlnoise[i] =
                  rate * ctrlnoise[i] + scale * mju_standardNormal(nullptr);

              // apply noise
              d->ctrl[i] = ctrlnoise[i];
            }
          }

          // requested slow-down factor
          double slowdown = 100 / sim.percentRealTime[sim.real_time_index];

          // misalignment condition: distance from target sim time is bigger
          // than syncmisalign
          bool misaligned = mju_abs(Seconds(elapsedCPU).count() / slowdown -
                                    elapsedSim) > syncMisalign;

          // out-of-sync (for any reason): reset sync times, step
          if (elapsedSim < 0 || elapsedCPU.count() < 0 ||
              syncCPU.time_since_epoch().count() == 0 || misaligned ||
              sim.speed_changed) {
            // re-sync
            syncCPU = startCPU;
            syncSim = d->time;
            sim.speed_changed = false;

            // run single step, let next iteration deal with timing
            mj_step(m, d);
            stepped = true;
          }

          // in-sync: step until ahead of cpu
          else {
            bool measured = false;
            mjtNum prevSim = d->time;

            double refreshTime = simRefreshFraction / sim.refresh_rate;

            // step while sim lags behind cpu and within refreshTime
            while (Seconds((d->time - syncSim) * slowdown) <
                       mj::Simulate::Clock::now() - syncCPU &&
                   mj::Simulate::Clock::now() - startCPU <
                       Seconds(refreshTime)) {
              // measure slowdown before first step
              if (!measured && elapsedSim) {
                sim.measured_slowdown =
                    std::chrono::duration<double>(elapsedCPU).count() /
                    elapsedSim;
                measured = true;
              }

              // call mj_step
              mj_step(m, d);
              stepped = true;

              // break if reset
              if (d->time < prevSim) {
                break;
              }
            }
          }

          // save current state to history buffer
          if (stepped) {
            sim.AddToHistory();
          }
        }

        // paused
        else {
          // run mj_forward, to update rendering and joint sliders
          mj_forward(m, d);
          sim.speed_changed = true;
        }
      }
    } // release std::lock_guard<std::mutex>
  }
}
} // namespace

void ControlLoop(mj::Simulate &sim) {

  while (!sim.exitrequest.load()) {
    if (!sim.uiloadrequest.load()) {
      if (d != nullptr) {

        // std::chrono::time_point<std::chrono::system_clock> t_start =
        //     std::chrono::system_clock::now();

        {
          // pd_controller(d);
          FBModelState state;
          state.bodyOrientation << 1, 0, 0, 0;
          state.bodyPosition << 0, 0, 0.98;
          state.bodyVelocity << 0, 0, 0, 0, 0, 0;
          state.q <<
              // left leg
              0,
              0, -0.4, 0.8, -0.4,
              // right leg
              0, 0, -0.4, 0.8, -0.4,
              // torso joint
              0,
              // left arm
              0, 0, 0, 0,
              // right arm
              0, 0, 0, 0;
          state.qd <<
              // left leg
              0,
              0, 0, 0, 0,
              // right leg
              0, 0, 0, 0, 0,
              // torso joint
              0,
              // left arm
              0, 0, 0, 0,
              // right arm
              0, 0, 0, 0;
          _model.updateModel(state);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(2));

        // std::chrono::time_point<std::chrono::system_clock> t_end =
        //     std::chrono::system_clock::now();
        // double time_record =
        //     std::chrono::duration_cast<std::chrono::milliseconds>(t_end -
        //                                                           t_start)
        //         .count();
        // std::cout << "controller_time: " << time_record / 1000 << "\n";
      }
    }
  }
}

void EstimateLoop(mj::Simulate &sim) {

  while (!sim.exitrequest.load()) {
    if (!sim.uiloadrequest.load()) {
      if (d != nullptr) {

        // AngleNormalization(d->sensordata[1]);

        // std::chrono::time_point<std::chrono::system_clock> t_start =
        //     std::chrono::system_clock::now();

        {

          // estimate
          estimate.cheater_compute_state(state, d);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(2));

        // std::chrono::time_point<std::chrono::system_clock> t_end =
        //     std::chrono::system_clock::now();
        // double time_record =
        //     std::chrono::duration_cast<std::chrono::milliseconds>(t_end -
        //                                                           t_start)
        //         .count();
        // std::cout << "controller_time: " << time_record / 1000 << "\n";
      }
    }
  }
}

//-------------------------------------- physics_thread
//--------------------------------------------

void PhysicsThread(mj::Simulate *sim, const char *filename) {
  // request loadmodel if file given (otherwise drag-and-drop)
  if (filename != nullptr) {
    sim->LoadMessage(filename);
    m = LoadModel(filename, *sim);
    if (m)
      d = mj_makeData(m);
    if (d) {
      sim->Load(m, d, filename);
      mj_forward(m, d);

      YAML::Node config = YAML::LoadFile(yaml_name);
      // Initial state
      Vector4d initial_state;
      initial_state(0) = config["initial_state"]["pos"].as<double>();
      initial_state(1) = config["initial_state"]["theta"].as<double>();
      initial_state(2) = config["initial_state"]["vel"].as<double>();
      initial_state(3) = config["initial_state"]["omega"].as<double>();

      // 设置初始状态（可选）
      // Initialize joint positions
      double qpos_init[] = {// pelvis pos
                            0, 0, 0.98,
                            // pelvis quat
                            1, 0, 0, 0,
                            // left leg
                            0, 0, -0.4, 0.8, -0.4,
                            // right leg
                            0, 0, -0.4, 0.8, -0.4,
                            // torso joint
                            0,
                            // left arm
                            0, 0, 0, 0,
                            // right arm
                            0, 0, 0, 0};
      for (int i = 0; i < 26; ++i) {
        d->qpos[i] = qpos_init[i];
      }

      // allocate ctrlnoise
      free(ctrlnoise);
      ctrlnoise = static_cast<mjtNum *>(malloc(sizeof(mjtNum) * m->nu));
      mju_zero(ctrlnoise, m->nu);

    } else {
      sim->LoadMessageClear();
    }
  }
  // Initialize joint velocities
  mj_forward(m, d);
  std::this_thread::sleep_for(std::chrono::milliseconds(5000000000));

  PhysicsLoop(*sim);

  // delete everything we allocated
  free(ctrlnoise);
  mj_deleteData(d);
  mj_deleteModel(m);
}

//-------------------------------------- control_thread
//--------------------------------------------

void ControlThread(mj::Simulate *sim, const char *filename) {
  ControlLoop(*sim);
}

//-------------------------------------- estimate_thread
//--------------------------------------------

void EstimateThread(mj::Simulate *sim, const char *filename) {
  EstimateLoop(*sim);
}

//------------------------------------------ main
//--------------------------------------------------

// machinery for replacing command line error by a macOS dialog box when running
// under Rosetta
#if defined(__APPLE__) && defined(__AVX__)
extern void DisplayErrorDialogBox(const char *title, const char *msg);
static const char *rosetta_error_msg = nullptr;
__attribute__((used, visibility("default"))) extern "C" void
_mj_rosettaError(const char *msg) {
  rosetta_error_msg = msg;
}
#endif

// run event loop
int main(int argc, char **argv) {

  // display an error if running on macOS under Rosetta 2
#if defined(__APPLE__) && defined(__AVX__)
  if (rosetta_error_msg) {
    DisplayErrorDialogBox("Rosetta 2 is not supported", rosetta_error_msg);
    std::exit(1);
  }
#endif

  // print version, check compatibility
  std::printf("MuJoCo version %s\n", mj_versionString());
  if (mjVERSION_HEADER != mj_version()) {
    mju_error("Headers and library have different versions");
  }

  // scan for libraries in the plugin directory to load additional plugins
  scanPluginLibraries();

  mjvCamera cam;
  mjv_defaultCamera(&cam);

  mjvOption opt;
  mjv_defaultOption(&opt);

  mjvPerturb pert;
  mjv_defaultPerturb(&pert);

  // simulate object encapsulates the UI
  auto sim = std::make_unique<mj::Simulate>(std::make_unique<mj::GlfwAdapter>(),
                                            &cam, &opt, &pert,
                                            /* is_passive = */ false);

  const char *filename = nullptr;
  // if (argc > 1) {
  //   filename = argv[1];
  // }
  filename = "../h1_description/mjcf/scene.xml";

  // start physics thread
  std::thread physicsthreadhandle(&PhysicsThread, sim.get(), filename);
  // start control thread
  std::thread controlthreadhandle(&ControlThread, sim.get(), filename);
  // start etsimate thread
  std::thread estimatethreadhandle(&EstimateThread, sim.get(), filename);

  // start simulation UI loop (blocking call)
  sim->RenderLoop();
  physicsthreadhandle.join();
  controlthreadhandle.join();
  estimatethreadhandle.join();

  return 0;
}
