#ifndef _WAYPOINT_HPP
#define _WAYPOINT_HPP

#include "Pose.hpp"
#include "Timestamp.hpp"

class Waypoint{
public:
    enum Phase {
        SUPPORT,
        RAISE,
        FALL
    };

    Waypoint(Pose pose, Timestamp timeFromStart, Phase phase) : _pose(pose), _timeFromStart(timeFromStart), _phase(phase) {}
    Waypoint() = default;

    Pose getPose() {
        return _pose;
    }

    Timestamp getTimeFromStart() {
        return _timeFromStart;
    }

    Phase getPhase() {
        return _phase;
    }

private:
    Pose _pose;
    Timestamp _timeFromStart;
    Phase _phase;
};

#endif  // _WAYPOINT_HPP