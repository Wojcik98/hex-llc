#pragma once

#include "Configuration.hpp"
#include "Timestamp.hpp"
#include "Waypoint.hpp"
#include "WaypointBuffer.hpp"


using Coeffs = Eigen::Matrix4f;

class Trajectory {
public:
    explicit Trajectory() = default;

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
    Coeffs interpolationLinear(const Configuration &x0, const Configuration &x1,
                               Duration duration);

    WaypointBuffer buffer_;
};
