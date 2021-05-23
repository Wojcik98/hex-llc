#pragma once

#include "LegDescription.hpp"
#include "Pose.hpp"
#include "ServoConfig.hpp"
#include "Triplet.hpp"


struct HexConfig {
    enum Leg {
        L1, L2, L3,
        R1, R2, R3,
        LEGS_NUM
    };

    enum Joints {
        J1, J2, J3,
        JOINTS_NUM
    };

    Pose bodyToLeg[LEGS_NUM];
    Triplet leftTripletCenterToEndEffectors;
    Triplet rightTripletCenterToEndEffectors;
    LegDescription legDescription;
    ServoConfig servos[LEGS_NUM][JOINTS_NUM] = {
        {   // L1
            ServoConfig(2150, -1,  9),
            ServoConfig(1920,  1, 10),
            ServoConfig(600,  -1, 11)
        },
        {   // L2
            ServoConfig(2100, -1, 12),
            ServoConfig(1840,  1, 13),
            ServoConfig(560,  -1, 14)
        },
        {   // L3
            ServoConfig(2600, -1, 21),
            ServoConfig(1700,  1, 22),
            ServoConfig(640,  -1, 23)
        },
        {   // R1
            ServoConfig(810,  -1,  6),
            ServoConfig(1150, -1,  7),
            ServoConfig(2440,  1,  8)
        },
        {   // R2
            ServoConfig(1150, -1, 15),
            ServoConfig(1160, -1, 16),
            ServoConfig(2250,  1, 17)
        },
        {   // R3
            ServoConfig(450,  -1, 18),
            ServoConfig(110,  -1, 19),
            ServoConfig(2420,  1, 20)
        }
    };
};
