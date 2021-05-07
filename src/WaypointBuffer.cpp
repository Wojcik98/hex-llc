#include "WaypointBuffer.hpp"
#include "Waypoint.hpp"

#include <cstdint>


WaypointBuffer::WaypointBuffer() : head_(SIZE - 1), tail_(0), current_size_(0) {
}


void WaypointBuffer::add(Waypoint waypoint) {
    if (current_size_ < SIZE) {
        ++current_size_;
    } else {
        tail_ = (tail_ + 1) & SIZE;
    }

    head_ = (head_ + 1) & SIZE;
    buffer_[head_] = waypoint;
}


void WaypointBuffer::clear() {
    head_ = SIZE - 1;
    tail_ = 0;
    current_size_ = 0;
}


Waypoint WaypointBuffer::end() {
    return buffer_[head_];
}


Waypoint WaypointBuffer::at(uint8_t index) {
    if (index >= current_size_ && current_size_ != 0) {
        index = index % current_size_;
    }

    return buffer_[(tail_ + index) & SIZE];
}


uint8_t WaypointBuffer::size() {
    return current_size_;
}
