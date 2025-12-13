/*
 * PhysicsSystem.h
 */

#pragma once

#include "ecs/Ecs.h"
#include "ecs/systems/PhysicsSystem.h"
#include "common/Components.h"
#include "collision/BoxCollider.h"

namespace BulletEngine {

class PhysicsSystem : public ecs::systems::PhysicsSystemBase {
public:
    PhysicsSystem(BulletPhysic::dynamics::PhysicsWorld& physicsWorld, BulletPhysic::math::IIntegrator& integrator, ecs::Entity* targetProjectileId)
        : PhysicsSystemBase(physicsWorld, integrator), m_targetId(targetProjectileId)
    {}

protected:
    bool beforeIntegrate(ecs::World& world, ecs::Entity entity, ecs::RigidBodyComponent& rigidBody, float dt) override;
    void afterIntegrate(ecs::World& world, ecs::Entity entity, ecs::RigidBodyComponent& rigidBody, float dt) override;

private:
    ecs::Entity* m_targetId;
};

} // namespace BulletEngine
