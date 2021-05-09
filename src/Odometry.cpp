#include "Odometry.hpp"

#include "Configuration.hpp"
#include "Pose.hpp"
#include "Timestamp.hpp"

#include "Eigen/Eigen"


void Odometry::addWaypoint(Pose pose, Duration duration, Timestamp time) {
    Timestamp startTime = trajectory_.finished(time) ? time : trajectory_.finishTime();
    Configuration prevConfig = trajectory_.getConfiguration(trajectory_.finishTime());
    Configuration nextConfig = toConfiguration(pose);

    Waypoint first {    // TODO: is this waypoint always necessary?
        prevConfig, {0.0F, 0.0F, 0.0F, 0.0F},   // configuration and velocity
        startTime,
        Waypoint::Interpolation::LINEAR
    };
    Waypoint last {
        nextConfig, {0.0F, 0.0F, 0.0F, 0.0F},   // configuration and velocity
        startTime + duration,
        Waypoint::Interpolation::LINEAR
    };

    trajectory_.addWaypoint(first);
    trajectory_.addWaypoint(last);
}


Pose Odometry::getPose(Timestamp time) {
    Configuration configuration = trajectory_.getConfiguration(time);
    return toPose(configuration);
}
