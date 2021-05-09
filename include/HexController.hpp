#pragma once

#include "HexConfig.hpp"
#include "InverseKinematics.hpp"
#include "Joints.hpp"
#include "Odometry.hpp"
#include "Pose.hpp"
#include "Timestamp.hpp"
#include "TripletController.hpp"


class HexController {
public:
    explicit HexController() = default;

    void addWaypoint(Pose pose, Duration duration, Timestamp time);
    Joints getJoints(Timestamp time);
    Pose getOdom(Timestamp time);

private:
    enum Side {
        LEFT, RIGHT
    };

    TripletController leftTriplet_;
    TripletController rightTriplet_;
    Odometry odometry_;
    HexConfig hexConfig_;
    InverseKinematics inverseKinematics_;

    Side lastTriplet_{LEFT};
};
