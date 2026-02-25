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
    float energy;
};

class EnergyTrajectoryComponent : public Component {
public:
    std::vector<EnergyTrajectoryPoint> points;
    float initialEnergy = 0.0f;
    float minSegment = 0.02f;
};

} // namespace ecs
} // namespace BulletEngine
