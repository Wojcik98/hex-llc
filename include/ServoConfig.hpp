#pragma once

#include <cstdint>


struct ServoConfig {
    ServoConfig(uint16_t center_, uint8_t dir_, uint8_t pin_) :
        center(center_), dir(dir_), pin(pin_) {}

    uint16_t center;
    uint8_t dir;
    uint8_t pin;
};
