#pragma once

#include "HexConfig.hpp"


struct LegJoints {
    float joints[HexConfig::JOINTS_NUM];
};


struct Joints {
    LegJoints legs[HexConfig::LEGS_NUM];
};
