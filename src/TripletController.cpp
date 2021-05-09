#include "TripletController.hpp"

#include "Configuration.hpp"
#include "Pose.hpp"
#include "Timestamp.hpp"
#include "Waypoint.hpp"

#include "Eigen/Eigen"


void TripletController::addWaypoint(const Pose &pose, Duration duration, Timestamp time) {
    Timestamp startTime = trajectory_.finished(time) ? time : trajectory_.finishTime();
    Configuration prevConfig = trajectory_.getConfiguration(trajectory_.finishTime());
    Configuration nextConfig = toConfiguration(pose);
    Configuration middleConfig = (prevConfig + nextConfig) / 2.0F;
    middleConfig(2) += 0.05F;   // TODO step height

    Waypoint first {    // TODO: is this waypoint always necessary?
        prevConfig, {0.0F, 0.0F, 0.0F, 0.0F},   // configuration and velocity
        startTime,
        Waypoint::Interpolation::ENDPOINTS_POS_AND_VEL
    };
    Waypoint middle {
        middleConfig, {0.0F, 0.0F, 0.0F, 0.0F}, // configuration and velocity
        startTime + duration / 2,
        Waypoint::Interpolation::END_ACCEL_0
    };
    Waypoint last {
        nextConfig, {0.0F, 0.0F, 0.0F, 0.0F},   // configuration and velocity
        startTime + duration,
        Waypoint::Interpolation::ENDPOINTS_POS_AND_VEL
    };

    trajectory_.addWaypoint(first);
    trajectory_.addWaypoint(middle);
    trajectory_.addWaypoint(last);
}


void TripletController::addDelay(Duration duration, Timestamp time) {
    Timestamp startTime = trajectory_.finished(time) ? time : trajectory_.finishTime();
    Configuration prevConfig = trajectory_.getConfiguration(trajectory_.finishTime());

    Waypoint delay {
        prevConfig, {0.0F, 0.0F, 0.0F, 0.0F}, // configuration and velocity
        startTime + duration,
        Waypoint::Interpolation::ENDPOINTS_POS_AND_VEL
    };

    trajectory_.addWaypoint(delay);
}


Pose TripletController::getPose(Timestamp time) {
    Configuration configuration = trajectory_.getConfiguration(time);
    return toPose(configuration);
}


bool TripletController::finished(Timestamp time) {
    return trajectory_.finished(time);
}


void TripletController::clear() {
    trajectory_.clear();
}
