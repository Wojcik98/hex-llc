#pragma once

#include "HexConfig.hpp"
#include "Joints.hpp"
#include "Pose.hpp"


class InverseKinematics {
public:
    explicit InverseKinematics() = default;

    Joints getJoints(const Pose &bodyPose, const Pose &leftPose, const Pose &rightPose,
                     const HexConfig &config);

private:
    LegJoints getLegJoints(const Pose &endEffector, const HexConfig &config);
};
