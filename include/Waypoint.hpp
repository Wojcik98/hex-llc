#pragma once

#include "Configuration.hpp"
#include "Timestamp.hpp"


class Waypoint {
public:
    enum Interpolation {
        ENDPOINTS_POS_AND_VEL,
        END_ACCEL_0,
        LINEAR
    };

    Waypoint(Configuration configuration, Configuration velocity, Timestamp timeFromStart, Interpolation interpolation)
        : configuration_(configuration), velocity_(velocity), timeFromStart_(timeFromStart), interpolation_(interpolation) {}
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

    Interpolation getInterpolation() {
        return interpolation_;
    }

private:
    Configuration configuration_;
    Configuration velocity_;
    Timestamp timeFromStart_{0};
    Interpolation interpolation_{Interpolation::LINEAR};
};
