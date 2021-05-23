#include "InverseKinematics.hpp"

#include "GlobalFunctions.h"
#include "HexConfig.hpp"
#include "Joints.hpp"
#include "Pose.hpp"
#include <math.h>


Joints InverseKinematics::getJoints(const Pose &bodyPose, const Pose &leftPose, const Pose &rightPose,
                                    const HexConfig &config) {
    Joints result;
    Pose bodyInverse = bodyPose.inverse();
    Pose bodyToLeft = bodyInverse * leftPose;
    Pose bodyToRight = bodyInverse * rightPose;
    
    // Left triplet
    Pose ee = config.leftTripletCenterToEndEffectors.upper;
    Pose legBase = config.bodyToLeg[HexConfig::Leg::R1];
    result.legs[HexConfig::Leg::R1] = getLegJoints(legBase.inverse() * bodyToLeft * ee, config);

    ee = config.leftTripletCenterToEndEffectors.middle;
    legBase = config.bodyToLeg[HexConfig::Leg::L2];
    result.legs[HexConfig::Leg::L2] = getLegJoints(legBase.inverse() * bodyToLeft * ee, config);

    ee = config.leftTripletCenterToEndEffectors.lower;
    legBase = config.bodyToLeg[HexConfig::Leg::R3];
    result.legs[HexConfig::Leg::R3] = getLegJoints(legBase.inverse() * bodyToLeft * ee, config);
    
    // Right triplet
    ee = config.rightTripletCenterToEndEffectors.upper;
    legBase = config.bodyToLeg[HexConfig::Leg::L1];
    result.legs[HexConfig::Leg::L1] = getLegJoints(legBase.inverse() * bodyToLeft * ee, config);

    ee = config.rightTripletCenterToEndEffectors.middle;
    legBase = config.bodyToLeg[HexConfig::Leg::R2];
    result.legs[HexConfig::Leg::R2] = getLegJoints(legBase.inverse() * bodyToLeft * ee, config);

    ee = config.rightTripletCenterToEndEffectors.lower;
    legBase = config.bodyToLeg[HexConfig::Leg::L3];
    result.legs[HexConfig::Leg::L3] = getLegJoints(legBase.inverse() * bodyToLeft * ee, config);

    return result;
}


LegJoints InverseKinematics::getLegJoints(const Pose &endEffector, const HexConfig &config) {
    float x = endEffector(0);
    float y = endEffector(1);
    float z = endEffector(2);

    float alpha = atan2f(y, x);

    float w = sqrtf(x*x + y*y) - config.legDescription(0);
    float a = config.legDescription(1);
    float b = config.legDescription(2);

    float r = sqrtf(w*w + z*z);
    float delta = acosf((a*a + b*b - r*r) / (2.0F *  a * b));
    float gamma = PI - delta;

    float b1 = atan2f(w, -z);
    float b2 = acosf((a*a + r*r - b*b) / (2.0F * a * r));
    float beta = b1 + b2;

    return {alpha,  beta - PI / 2.0F, -gamma};
}
