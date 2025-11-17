/*
 * PhysicsSystem.h
 */

#pragma once

#include "ecs/Ecs.h"
#include "ecs/Components.h"

#include "math/Integrator.h"
#include "collision/BoxCollider.h"
#include "dynamics/PhysicsWorld.h"
#include "dynamics/environment/Wind.h"

#include <cmath>

namespace BulletEngine {
namespace ecs {
namespace systems {

class PhysicsSystem {
public:
    PhysicsSystem(BulletPhysic::dynamics::PhysicsWorld& physicsWorld, BulletPhysic::math::IIntegrator& integrator);

    void update(World& world, float dt);

private:
    BulletPhysic::dynamics::PhysicsWorld& m_physicsWorld;
    BulletPhysic::math::IIntegrator& m_integrator;
};

} // namespace systems
} // namespace ecs
} // namespace BulletEngine
