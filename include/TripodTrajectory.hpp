#pragma once

#include "Pose.hpp"
#include "Timestamp.hpp"
#include "Trajectory.hpp"
#include "Waypoint.hpp"


using Coeffs = Eigen::Matrix4f;

class TripodTrajectory {
public:
    explicit TripodTrajectory() = default;

    void addWaypoint(Pose pose, Duration duration);
    Pose getPose(Timestamp time);
    Timestamp finishTime();
    bool finished(Timestamp time);
    void clear();

private:
    Trajectory trajectory_;
};
