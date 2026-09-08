/*
 * PhysicsSystem.h
 */

#pragma once

#include "ecs/Ecs.h"
#include "ecs/Components.h"

#include "dynamics/PhysicsWorld.h"

#include <vector>

namespace BulletEngine {
namespace ecs {
namespace systems {

class PhysicsSystem {
public:
    PhysicsSystem() = default;

    void update(World& world, float dt);

    // contacts
    void setContactListener(BulletPhysics::dynamics::IContactListener* listener) { m_physicsWorld.setContactListener(listener); }   // not owned
    const std::vector<BulletPhysics::collision::Manifold>& getContacts() const { return m_physicsWorld.getContacts(); }

    // queries
    bool raycast(const BulletPhysics::collision::Ray& ray, BulletPhysics::collision::RayHit& outHit) const { return m_physicsWorld.raycast(ray, outHit); }

private:
    void syncBodies(World& world);
    void publishTransforms(World& world);

    BulletPhysics::dynamics::PhysicsWorld m_physicsWorld;
};

} // namespace systems
} // namespace ecs
} // namespace BulletEngine
