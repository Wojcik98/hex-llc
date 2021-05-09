#pragma once

#include "HexConfig.hpp"
#include "Joints.hpp"
#include "Pose.hpp"


class InverseKinematics {
public:
    explicit InverseKinematics() = default;

    Joints getJoints(const Pose &body_pose, const Pose &left_pose, const Pose &right_pose,
                     const HexConfig &config);
};
