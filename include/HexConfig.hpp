#pragma once

#include "LegDescription.hpp"
#include "Pose.hpp"
#include "Triplet.hpp"


struct HexConfig {
    enum Leg {
        L1, L2, L3,
        R1, R2, R3
    };

    Pose bodyToLeg[6];
    Triplet leftTripletCenterToEndEffectors;
    LegDescription legDescription;
};
