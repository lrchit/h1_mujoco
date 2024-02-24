
#pragma once

#include <Eigen/Eigen>
#include <queue>
#include <string>

using Eigen::Array2d;
using Eigen::Array2i;

class Gait {
public:
  virtual ~Gait() = default;

  virtual Eigen::Vector<double, 2> getContactState() = 0;
  virtual Eigen::Vector<double, 2> getSwingState() = 0;
  virtual int *getMpcTable() = 0;
  virtual void setIterations(int iterationsBetweenMPC,
                             int currentIteration) = 0;
  virtual double getCurrentStanceTime(double dtMPC, int leg) = 0;
  virtual double getCurrentSwingTime(double dtMPC, int leg) = 0;
  virtual int getCurrentGaitPhase() = 0;
  virtual void debugPrint() {}

protected:
  std::string _name;
};

class OffsetDurationGait : public Gait {
public:
  OffsetDurationGait(int nSegment, Eigen::Vector<int, 2> offset,
                     Eigen::Vector<int, 2> durations, const std::string &name);
  ~OffsetDurationGait();
  Eigen::Vector<double, 2> getContactState();
  Eigen::Vector<double, 2> getSwingState();
  int *getMpcTable();
  void setIterations(int iterationsBetweenMPC, int currentIteration);
  double getCurrentStanceTime(double dtMPC, int leg);
  double getCurrentSwingTime(double dtMPC, int leg);
  int getCurrentGaitPhase();
  void debugPrint();

private:
  int *_mpc_table;
  Array2i _offsets;         // offset in mpc segments
  Array2i _durations;       // duration of step in mpc segments
  Array2d _offsetsDouble;   // offsets in phase (0 to 1)
  Array2d _durationsDouble; // durations in phase (0 to 1)
  int _stance;
  int _swing;
  int _iteration;
  int _nIterations;
  double _phase;
};

class MixedFrequncyGait : public Gait {
public:
  MixedFrequncyGait(int nSegment, Eigen::Vector<int, 2> periods,
                    double duty_cycle, const std::string &name);
  ~MixedFrequncyGait();
  Eigen::Vector<double, 2> getContactState();
  Eigen::Vector<double, 2> getSwingState();
  int *getMpcTable();
  void setIterations(int iterationsBetweenMPC, int currentIteration);
  double getCurrentStanceTime(double dtMPC, int leg);
  double getCurrentSwingTime(double dtMPC, int leg);
  int getCurrentGaitPhase();
  void debugPrint();

private:
  double _duty_cycle;
  int *_mpc_table;
  Array2i _periods;
  Array2d _phase;
  int _iteration;
  int _nIterations;
};
