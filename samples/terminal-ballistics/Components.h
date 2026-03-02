/*
 * Components.h
 */

#pragma once

#include "ecs/Ecs.h"
#include "math/Vec3.h"

#include <vector>

namespace BulletEngine {
namespace ecs {

class ImpactStateComponent : public Component {
public:
    bool hasImpacted = false;
};

struct EnergyTrajectoryPoint {
    BulletPhysics::math::Vec3 position;
    double energy;
};

class EnergyTrajectoryComponent : public Component {
public:
    std::vector<EnergyTrajectoryPoint> points;
    double initialEnergy = 0.0;
    double minSegment = 0.005;
};

} // namespace ecs
} // namespace BulletEngine
