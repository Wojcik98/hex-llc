#include "Timestamp.hpp"
#include "Trajectory.hpp"
#include "Waypoint.hpp"

#include <cstdint>


void Trajectory::addWaypoint(Waypoint waypoint) {
    buffer_.add(waypoint);
}


Timestamp Trajectory::finishTime() {
    return buffer_.end().getTimeFromStart();
}


bool Trajectory::finished(Timestamp time) {
    return time >= finishTime();
}


Configuration Trajectory::getConfiguration(Timestamp time) {
    uint8_t next_index = 1;

    for (; next_index < buffer_.size(); ++next_index) {
        if (buffer_.at(next_index).getTimeFromStart() > time) {
            break;
        }
    }

    // Single waypoint in buffer or no next waypoint found, return latest known configuration
    if (buffer_.size() == 1 || buffer_.at(next_index).getTimeFromStart() <= time) {
        return buffer_.at(next_index).getConfiguration();
    }

    Waypoint next_waypoint = buffer_.at(next_index);
    Waypoint prev_waypoint = buffer_.at(next_index - 1);

    Duration duration = next_waypoint.getTimeFromStart() - prev_waypoint.getTimeFromStart();

    Configuration x0 = prev_waypoint.getConfiguration();
    Configuration v0 = prev_waypoint.getVelocity();
    Configuration x1 = next_waypoint.getConfiguration();
    Configuration v1 = next_waypoint.getVelocity();

    Coeffs coeffs;
    Waypoint::Interpolation interpolation = next_waypoint.getInterpolation();

    if (interpolation == Waypoint::Interpolation::END_ACCEL_0) {
        coeffs = interpolationCoeffsZeroEndAccel(x0, x1, v0, duration);
    } else if (interpolation == Waypoint::Interpolation::ENDPOINTS_POS_AND_VEL) {
        coeffs = interpolationCoeffs(x0, x1, v0, v1, duration);
    } else if (interpolation == Waypoint::Interpolation::LINEAR) {
        coeffs = interpolationLinear(x0, x1, duration);
    }

    Timestamp dt = time - prev_waypoint.getTimeFromStart();
    Configuration powers(1, dt, dt * dt, dt * dt * dt);
    Configuration result = coeffs * powers;

    return result;
}


void Trajectory::clear() {
    buffer_.clear();
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


Coeffs Trajectory::interpolationLinear(const Configuration &x0, const Configuration &x1,
                                       Duration duration) {
    Coeffs result;

    result.col(0) = x0;
    result.col(1) = (x1 - x0) / duration;
    result.col(2).setZero();
    result.col(3).setZero();

    return result;
}
