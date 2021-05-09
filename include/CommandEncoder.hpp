#pragma once

#include "HexConfig.hpp"
#include "Joints.hpp"


class CommandEncoder {
public:
    explicit CommandEncoder() = default;

    void getCommand(const Joints &joints, const HexConfig &config);
};
