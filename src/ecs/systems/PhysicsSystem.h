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
#include <vector>

namespace BulletEngine {
namespace ecs {
namespace systems {

class PhysicsSystemBase {
public:
    PhysicsSystemBase(BulletPhysics::dynamics::PhysicsWorld& physicsWorld, BulletPhysics::math::IIntegrator& integrator);
    virtual ~PhysicsSystemBase() = default;

    void update(World& world, float dt);

protected:
    // hooks
    virtual bool beforeIntegrate(World&, Entity, RigidBodyComponent&, float) {return true;}
    virtual void afterIntegrate(World&, Entity, RigidBodyComponent&, float) {}

    BulletPhysics::dynamics::PhysicsWorld& m_physicsWorld;
    BulletPhysics::math::IIntegrator& m_integrator;
};

} // namespace systems
} // namespace ecs
} // namespace BulletEngine
