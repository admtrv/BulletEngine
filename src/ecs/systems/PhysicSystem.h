/*
 * PhysicSystem.h
 */

#pragma once

#include "ecs/Ecs.h"
#include "ecs/Components.h"

#include "PresetManager.h"
#include "math/Integrator.h"
#include "collision/BoxCollider.h"
#include "dynamics/forces/WindDragForce.h"

#include <cmath>

namespace BulletEngine {
namespace ecs {
namespace systems {

class PhysicSystem {
public:
    explicit PhysicSystem(BulletPhysic::math::IIntegrator& integrator);

    void update(World& world, float dt);

    BulletPhysic::dynamics::forces::ForceRegistry& getForceRegistry() { return m_forceRegistry;}

    void setRealismLevel(BulletPhysic::preset::RealismLevel level, float projectileArea = 0.01f);
    void setWindVelocity(const BulletPhysic::math::Vec3& windVel);

private:
    BulletPhysic::math::IIntegrator& m_integrator;
    BulletPhysic::dynamics::forces::ForceRegistry m_forceRegistry;
};

} // namespace systems
} // namespace ecs
} // namespace BulletEngine
