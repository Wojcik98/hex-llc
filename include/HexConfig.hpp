#pragma once

#include "Pose.hpp"
#include "Triangle.hpp"


struct HexConfig {
    enum Leg {
        L1, L2, L3,
        R1, R2, R3
    };

    Pose bodyToLeg[6];
    Triangle triangleCenterToEndEffectors;
    // TODO joints calibration and servo channels
};
