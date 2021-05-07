#ifndef _WAYPOINT_BUFFER_HPP
#define _WAYPOINT_BUFFER_HPP

#include "Waypoint.hpp"

#include <cstdint>


class WaypointBuffer {
public:
    explicit WaypointBuffer();

    void add(Waypoint waypoint);
    void clear();

    Waypoint end();
    Waypoint at(uint8_t index);
    uint8_t size();

private:
    static const uint8_t SIZE = 8;

    Waypoint buffer_[SIZE];
    uint8_t head_;
    uint8_t tail_;
    uint8_t current_size_;
};

#endif  // _WAYPOINT_BUFFER_HPP
