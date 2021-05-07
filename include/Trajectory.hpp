#ifndef _TRAJECTORY_HPP
#define _TRAJECTORY_HPP

#include "Configuration.hpp"
#include "Timestamp.hpp"
#include "Waypoint.hpp"
#include "WaypointBuffer.hpp"

#include "Eigen/Eigen"

using Coeffs = Eigen::Matrix4f;

class Trajectory {
public:
    explicit Trajectory();

    void addWaypoint(Waypoint waypoint);
    Timestamp finishTime();
    bool finished(Timestamp time);
    Configuration getConfiguration(Timestamp time);
    void clear();

private:
    Coeffs interpolationCoeffs(const Configuration &x0, const Configuration &x1,
                               const Configuration &v0, const Configuration &v1,
                               Duration duration);
    Coeffs interpolationCoeffsZeroEndAccel(const Configuration &x0, const Configuration &x1,
                                           const Configuration &v0, Duration duration);

    WaypointBuffer _buffer;
};

#endif  // _TRAJECTORY_HPP
