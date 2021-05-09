#include "TripodTrajectory.hpp"

#include "Configuration.hpp"
#include "Pose.hpp"
#include "Timestamp.hpp"
#include "Waypoint.hpp"


void TripodTrajectory::addWaypoint(Pose pose, Duration duration) {
    // TODO
}


Pose TripodTrajectory::getPose(Timestamp time) {
    Configuration configuration = trajectory_.getConfiguration(time);

    Pose result = Pose::Identity();
    result.row(3).head<3>() = configuration.head<3>();
    result.topLeftCorner<3, 3>() = Eigen::AngleAxisf(configuration(3), Eigen::Vector3f::UnitZ()).toRotationMatrix();

    return result;
}


Timestamp TripodTrajectory::finishTime() {
    return trajectory_.finishTime();
}


bool TripodTrajectory::finished(Timestamp time) {
    return trajectory_.finished(time);
}


void TripodTrajectory::clear() {
    trajectory_.clear();
}
