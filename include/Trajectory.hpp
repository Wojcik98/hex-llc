#ifndef _TRAJECTORY_HPP
#define _TRAJECTORY_HPP

#include "Pose.hpp"
#include "Waypoint.hpp"
#include "WaypointBuffer.hpp"
#include "Timestamp.hpp"

#include "Eigen/Eigen"

using Coeffs = Eigen::Matrix4f;

class Trajectory {
public:
    using Point = Eigen::Vector4f;

    explicit Trajectory();

    void add_waypoint(Waypoint waypoint);
    Timestamp finish_time();
    bool finished(Timestamp time);
    Pose get_pose(Timestamp time);
    void clear();

private:
    Coeffs interpolation_coeffs(Point x0, Point x1, Point v0, Point v1, Duration duration);
    Coeffs interpolation_coeffs_zero_end_accel(Point x0, Point x1, Point v0, Duration duration);

    WaypointBuffer _buffer;
};

#endif  // _TRAJECTORY_HPP
