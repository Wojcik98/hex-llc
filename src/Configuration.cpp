#include "Configuration.hpp"

#include "Pose.hpp"

#include "Eigen/Eigen"


Pose toPose(const Configuration &configuration) {
    Pose result = Pose::Identity();
    result.row(3).head<3>() = configuration.head<3>();
    result.topLeftCorner<3, 3>() = Eigen::AngleAxisf(configuration(3), Eigen::Vector3f::UnitZ()).toRotationMatrix();

    return result;
}


Configuration toConfiguration(const Pose &pose) {
    Configuration result;
    result.head<3>() = pose.row(3).head<3>();
    // rotaation in z-axis
    result(3) = pose.topLeftCorner<3, 3>().eulerAngles(0, 1, 2)(2);

    return result;
}
