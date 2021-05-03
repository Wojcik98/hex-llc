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

    Waypoint _buffer[SIZE];
    uint8_t _head;
    uint8_t _tail;
    uint8_t _current_size;
};

#endif  // _WAYPOINT_BUFFER_HPP
