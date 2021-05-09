#pragma once

#include "Pose.hpp"
#include "Timestamp.hpp"
#include "Trajectory.hpp"


class Odometry {
public:
    explicit Odometry() = default;

    void addWaypoint(Pose pose, Duration duration);
    Pose getPose(Timestamp time);

private:
    Trajectory trajectory_;
};
