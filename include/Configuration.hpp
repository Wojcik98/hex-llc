#pragma once

#include "Eigen/Eigen"
#include "Pose.hpp"


// x, y, z, theta (in z axis)
using Configuration = Eigen::Vector4f;


Pose toPose(const Configuration &configuration);
Configuration toConfiguration(const Pose &pose);
