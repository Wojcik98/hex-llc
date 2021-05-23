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
    constexpr static const float PI = 3.1415926535F;

    LegJoints getLegJoints(const Pose &endEffector, const HexConfig &config);
};
