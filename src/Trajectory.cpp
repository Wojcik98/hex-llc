#include "Pose.hpp"
#include "Timestamp.hpp"
#include "Trajectory.hpp"
#include "Waypoint.hpp"

#include "Eigen/Eigen"
#include <cstdint>


Trajectory::Trajectory() {
    // _buffer.add(Waypoint())
}


void Trajectory::add_waypoint(Waypoint waypoint) {
    _buffer.add(waypoint);
}


Timestamp Trajectory::finish_time() {
    return _buffer.end().getTimeFromStart();
}


bool Trajectory::finished(Timestamp time) {
    return time >= finish_time();
}


Pose Trajectory::get_pose(Timestamp time) {
    uint8_t next_index = 1;
    
    for (; next_index < _buffer.size(); ++next_index) {
        if (_buffer.at(next_index).getTimeFromStart() > time) {
            break;
        }
    }

    Waypoint next_waypoint = _buffer.at(next_index);
    Waypoint prev_waypoint = _buffer.at(next_index - 1);

    Duration duration = next_waypoint.getTimeFromStart() - prev_waypoint.getTimeFromStart();

    Point x0 = prev_waypoint.getPose().row(3);
    x0(3) = prev_waypoint.getPose().topLeftCorner<3, 3>().eulerAngles(0, 1, 2)(2);  // TODO correct?

    Point x1 = next_waypoint.getPose().row(3);
    x1(3) = next_waypoint.getPose().topLeftCorner<3, 3>().eulerAngles(0, 1, 2)(2);  // TODO correct?

    Coeffs coeffs;

    // TODO velocities
    if (next_waypoint.getPhase() == Waypoint::Phase::RAISE) {
        Point v0;
        coeffs = interpolation_coeffs_zero_end_accel(x0, x1, v0, duration);
    } else if (next_waypoint.getPhase() == Waypoint::Phase::FALL) {
        Point v0;
        Point v1;
        coeffs = interpolation_coeffs(x0, x1, v0, v1, duration);
    } else {
        Point v0;
        Point v1;
        coeffs = interpolation_coeffs(x0, x1, v0, v1, duration);
    }

    Point powers(1, time, time * time, time * time * time);
    Point raw = coeffs * powers;

    Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
    result.row(3).head<3>() = raw.head<3>();
    result.topLeftCorner<3, 3>() = Eigen::AngleAxisf(raw(3), Eigen::Vector3f::UnitZ()).toRotationMatrix();

    return result;
}


void Trajectory::clear() {
    _buffer.clear();
}


Coeffs Trajectory::interpolation_coeffs(Point x0, Point x1, Point v0, Point v1, Duration duration) {
    Coeffs result;

    result.col(0) = x0;
    result.col(1) = v0;
    result.col(2) = (3.0f * (x1 - x0) - (2.0f * v0 + v1) * duration) / (duration * duration);
    result.col(3) = (2.0f * (x0 - x1) - (v0 + v1) * duration) / (duration * duration * duration);

    return result;
}


Coeffs Trajectory::interpolation_coeffs_zero_end_accel(Point x0, Point x1, Point v0, Duration duration) {
    Coeffs result;

    result.col(0) = x0;
    result.col(1) = v0;
    result.col(2) = 3.0f * (x1 - x0 - v0 * duration) / (2.0f * duration * duration);
    result.col(3) = (x1 - x0 - v0 * duration) / (-2.0f * duration * duration * duration);

    return result;
}
