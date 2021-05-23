#pragma once

#include "HexConfig.hpp"
#include "Joints.hpp"
#include "ServoConfig.hpp"

#include <cstdint>


class CommandEncoder {
public:
    explicit CommandEncoder() = default;

    uint16_t encodeCommand(const Joints &joints, const HexConfig &config, volatile uint8_t out[]);

private:
    static const uint8_t MULTIPLE_TARGETS_CMD = 0x9F;
    constexpr static const float PI = 3.1415926535F;

    void findPin(uint8_t pin, const HexConfig &config, uint8_t &leg, uint8_t &joint);
    uint16_t angle2duty(float angle, const ServoConfig &config);
};
