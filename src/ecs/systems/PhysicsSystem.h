/*
 * PhysicsSystem.h
 */

#pragma once

#include "ecs/Ecs.h"
#include "ecs/Components.h"

#include "math/Integrator.h"
#include "builtin/collision/collider/BoxCollider.h"
#include "ballistics/external/PhysicsWorld.h"
#include "ballistics/external/environments/Wind.h"

#include <cmath>
#include <vector>

namespace BulletEngine {
namespace ecs {
namespace systems {

class PhysicsSystemBase {
public:
    PhysicsSystemBase(BulletPhysics::ballistics::external::PhysicsWorld& physicsWorld, BulletPhysics::math::IIntegrator& integrator);
    virtual ~PhysicsSystemBase() = default;

    void update(World& world, float dt);

protected:
    // hooks
    virtual bool beforeIntegrate(World&, Entity, RigidBodyComponent&, float) {return true;}
    virtual void afterIntegrate(World&, Entity, RigidBodyComponent&, float) {}

    BulletPhysics::ballistics::external::PhysicsWorld& m_physicsWorld;
    BulletPhysics::math::IIntegrator& m_integrator;
};

} // namespace systems
} // namespace ecs
} // namespace BulletEngine
