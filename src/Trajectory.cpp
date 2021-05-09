#include "Timestamp.hpp"
#include "Trajectory.hpp"
#include "Waypoint.hpp"

#include <cstdint>


Trajectory::Trajectory() {
    // _buffer.add(Waypoint())
}


void Trajectory::addWaypoint(Waypoint waypoint) {
    _buffer.add(waypoint);
}


Timestamp Trajectory::finishTime() {
    return _buffer.end().getTimeFromStart();
}


bool Trajectory::finished(Timestamp time) {
    return time >= finishTime();
}


Configuration Trajectory::getConfiguration(Timestamp time) {
    uint8_t next_index = 1;

    for (; next_index < _buffer.size(); ++next_index) {
        if (_buffer.at(next_index).getTimeFromStart() > time) {
            break;
        }
    }

    Waypoint next_waypoint = _buffer.at(next_index);
    Waypoint prev_waypoint = _buffer.at(next_index - 1);

    Duration duration = next_waypoint.getTimeFromStart() - prev_waypoint.getTimeFromStart();

    Configuration x0 = prev_waypoint.getConfiguration();
    Configuration v0 = prev_waypoint.getVelocity();
    Configuration x1 = next_waypoint.getConfiguration();
    Configuration v1 = next_waypoint.getVelocity();

    Coeffs coeffs;

    if (next_waypoint.getPhase() == Waypoint::Phase::RAISE) {
        coeffs = interpolationCoeffsZeroEndAccel(x0, x1, v0, duration);
    } else {    // FALL or SUPPORT
        coeffs = interpolationCoeffs(x0, x1, v0, v1, duration);
    }

    Configuration powers(1, time, time * time, time * time * time);
    Configuration result = coeffs * powers;

    // Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
    // result.row(3).head<3>() = raw.head<3>();
    // result.topLeftCorner<3, 3>() = Eigen::AngleAxisf(raw(3), Eigen::Vector3f::UnitZ()).toRotationMatrix();

    return result;
}


void Trajectory::clear() {
    _buffer.clear();
}


Coeffs Trajectory::interpolationCoeffs(const Configuration &x0, const Configuration &x1,
                                       const Configuration &v0, const Configuration &v1,
                                       Duration duration) {
    Coeffs result;

    result.col(0) = x0;
    result.col(1) = v0;
    result.col(2) = (3.0F * (x1 - x0) - (2.0F * v0 + v1) * duration) / (duration * duration);
    result.col(3) = (2.0F * (x0 - x1) - (v0 + v1) * duration) / (duration * duration * duration);

    return result;
}


Coeffs Trajectory::interpolationCoeffsZeroEndAccel(const Configuration &x0, const Configuration &x1,
                                                   const Configuration &v0, Duration duration) {
    Coeffs result;

    result.col(0) = x0;
    result.col(1) = v0;
    result.col(2) = 3.0F * (x1 - x0 - v0 * duration) / (2.0F * duration * duration);
    result.col(3) = (x1 - x0 - v0 * duration) / (-2.0F * duration * duration * duration);

    return result;
}
