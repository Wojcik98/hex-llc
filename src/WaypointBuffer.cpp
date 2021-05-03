#include "WaypointBuffer.hpp"
#include "Waypoint.hpp"

#include <cstdint>


WaypointBuffer::WaypointBuffer() : _head(SIZE - 1), _tail(0), _current_size(0) {
}


void WaypointBuffer::add(Waypoint pose) {
    if (_current_size < SIZE) {
        ++_current_size;
    } else {
        _tail = (_tail + 1) & SIZE;
    }

    _head = (_head + 1) & SIZE;
    _buffer[_head] = pose;
}


void WaypointBuffer::clear() {
    _head = SIZE - 1;
    _tail = 0;
    _current_size = 0;
}


Waypoint WaypointBuffer::end() {
    return _buffer[_head];
}


Waypoint WaypointBuffer::at(uint8_t index) {
    if (index >= _current_size && _current_size != 0) {
        index = index % _current_size;
    }

    return _buffer[(_tail + index) & SIZE];
}


uint8_t WaypointBuffer::size() {
    return _current_size;
}
