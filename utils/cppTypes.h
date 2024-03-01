#ifndef PROJECT_CPPTYPES_H
#define PROJECT_CPPTYPES_H

#include <eigen3/Eigen/Dense>
#include <vector>

// Dynamically sized matrix
using DMat = typename Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;
// Dynamically sized vector
using DVec = typename Eigen::Matrix<double, Eigen::Dynamic, 1>;
// Dynamically sized matrix with cartesian vector columns
using D3Mat = typename Eigen::Matrix<double, 3, Eigen::Dynamic>;
// 2x1 Vector
using Vec2 = typename Eigen::Matrix<double, 2, 1>;
// 3x1 Vector
using Vec3 = typename Eigen::Matrix<double, 3, 1>;
// 4x1 Vector
using Vec4 = typename Eigen::Matrix<double, 4, 1>;
// 5x1 Vector
using Vec5 = typename Eigen::Matrix<double, 5, 1>;
// 6x1 Vector
using Vec6 = typename Eigen::Matrix<double, 6, 1>;
// 12x1 Vector
using Vec12 = typename Eigen::Matrix<double, 12, 1>;
// 18x1 Vector
using Vec18 = typename Eigen::Matrix<double, 18, 1>;
// 24x1 Vector
using Vec24 = typename Eigen::Matrix<double, 24, 1>; // full_config
// 25x1 Vector
using Vec25 = typename Eigen::Matrix<double, 25, 1>; // full_config
// 4x1 Vector (quaternion)
using Quat = typename Eigen::Matrix<double, 4, 1>;
// Spatial Vector (6x1, all subspaces)
using SVec = typename Eigen::Matrix<double, 6, 1>;
// 3x3 Matrix
using Mat3 = typename Eigen::Matrix<double, 3, 3>;

#endif
