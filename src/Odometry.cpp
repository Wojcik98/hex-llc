#include "Odometry.hpp"

#include "Pose.hpp"
#include "Timestamp.hpp"


void Odometry::addWaypoint(Pose pose, Duration duration) {
    // TODO convert to configuration and add to trajectory (linear interpolation, nothing too fancy)
}


Pose Odometry::getPose(Timestamp time) {
    Configuration configuration = trajectory_.getConfiguration(time);

    Pose result = Pose::Identity();
    result.row(3).head<3>() = configuration.head<3>();
    result.topLeftCorner<3, 3>() = Eigen::AngleAxisf(configuration(3), Eigen::Vector3f::UnitZ()).toRotationMatrix();

    return result;
}
