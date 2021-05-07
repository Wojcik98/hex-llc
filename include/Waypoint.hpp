#ifndef _WAYPOINT_HPP
#define _WAYPOINT_HPP

#include "Configuration.hpp"
#include "Timestamp.hpp"


class Waypoint {
public:
    enum Phase {
        SUPPORT,
        RAISE,
        FALL
    };

    Waypoint(Configuration configuration, Configuration velocity, Timestamp timeFromStart, Phase phase)
        : configuration_(configuration), velocity_(velocity), timeFromStart_(timeFromStart), phase_(phase) {}
    Waypoint() = default;

    Configuration getConfiguration() {
        return configuration_;
    }

    Configuration getVelocity() {
        return velocity_;
    }

    Timestamp getTimeFromStart() {
        return timeFromStart_;
    }

    Phase getPhase() {
        return phase_;
    }

private:
    Configuration configuration_;
    Configuration velocity_;
    Timestamp timeFromStart_{0};
    Phase phase_{Phase::SUPPORT};
};

#endif  // _WAYPOINT_HPP