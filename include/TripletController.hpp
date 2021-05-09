#pragma once

#include "Pose.hpp"
#include "Timestamp.hpp"
#include "Trajectory.hpp"
#include "Waypoint.hpp"


using Coeffs = Eigen::Matrix4f;

class TripletController {
public:
    explicit TripletController() = default;

    void addWaypoint(const Pose &pose, Duration duration, Timestamp time);
    void addDelay(Duration duration, Timestamp time);
    Pose getPose(Timestamp time);
    bool finished(Timestamp time);
    void clear();

private:
    Trajectory trajectory_;
};
