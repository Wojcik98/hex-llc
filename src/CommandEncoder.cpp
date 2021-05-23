#include "CommandEncoder.hpp"

#include "HexConfig.hpp"
#include "Joints.hpp"

#include <cstdint>


uint16_t CommandEncoder::getCommand(const Joints &joints, const HexConfig &config, uint8_t out[]) {
    uint8_t leg = 0;
    uint8_t joint = 0;
    uint16_t shift = 0;
    uint8_t mask = (1 << 7) - 1;

    // Setting multitarget, servos MUST be on subsequent pins
    // Protocol: 0x9F, num of target, first channel number, cmds...
    out[shift++] = MULTIPLE_TARGETS_CMD;
    out[shift++] = HexConfig::LEGS_NUM * HexConfig::JOINTS_NUM;
    out[shift++] = 0;

    for (uint8_t pin = 0; pin < HexConfig::LEGS_NUM * HexConfig::JOINTS_NUM; ++pin) {
        findPin(pin, config, leg, joint);
        uint16_t duty = angle2duty(joints.legs[leg].joints[joint], config.servos[leg][joint]);
        duty *= 4;

        out[shift++] = duty & mask;
        out[shift++] = (duty >> 7) & mask;
    }

    return shift;
}


uint16_t CommandEncoder::angle2duty(float angle, const ServoConfig &config) {
    int16_t displacement = int16_t(angle * 1000.0F / (PI / 2.0F)) * config.dir;
    int16_t duty = config.center + displacement;

    if (duty < 0 && duty + 4000 <= 2550) {
        duty += 4000;   // reverse direction
    }

    duty = duty > 460 ? duty : 460; // max
    duty = duty < 2550 ? duty : 2550;   //min

    return duty;
}


void CommandEncoder::findPin(uint8_t pin, const HexConfig &config, uint8_t &leg, uint8_t &joint) {
    for (leg = 0; leg < HexConfig::LEGS_NUM; ++leg) {
        for (joint = 0; joint < HexConfig::JOINTS_NUM; ++joint) {
            if (config.servos[leg][joint].pin == pin) {
                return;
            }
        }
    }
}
