#include "HexController.hpp"

#include "HexConfig.hpp"
#include "InverseKinematics.hpp"
#include "Joints.hpp"
#include "Odometry.hpp"
#include "Pose.hpp"
#include "Timestamp.hpp"
#include "TripletController.hpp"


void HexController::addWaypoint(Pose pose, Duration duration, Timestamp time) {
    // Body moves in `duration`, so legs have to move in `duration / 2`
    if (lastTriplet_ == Side::LEFT) {
        leftTriplet_.addDelay(duration / 2, time);
        rightTriplet_.addWaypoint(pose, duration / 2, time);
        lastTriplet_ = Side::RIGHT;
    } else {
        leftTriplet_.addWaypoint(pose, duration / 2, time);
        rightTriplet_.addDelay(duration / 2, time);
        lastTriplet_ = Side::LEFT;
    }

    odometry_.addWaypoint(pose, duration, time);
}


Joints HexController::getJoints(Timestamp time) {
    if (lastTriplet_ == Side::LEFT && rightTriplet_.finished(time)) {
        // TODO add last waypoint from left and add delay to left
    }

    return inverseKinematics_.getJoints(
        odometry_.getPose(time),
        leftTriplet_.getPose(time),
        rightTriplet_.getPose(time),
        hexConfig_
    );
}


Pose HexController::getOdom(Timestamp time) {
    return odometry_.getPose(time);
}
