/*
 * PhysicsSystem.h
 */

#pragma once

#include "ecs/Ecs.h"
#include "ecs/Components.h"

#include "dynamics/PhysicsWorld.h"

#include <unordered_set>
#include <vector>

namespace BulletEngine {
namespace ecs {
namespace systems {

class PhysicsSystem {
public:
    PhysicsSystem() = default;

    // simulation
    void update(World& world, float dt);    // own clock, runs as many steps as frame owes
    void step(World& world, float dt);      // one step, paced from outside
    void sync(World& world, bool adoptPoses = true);    // colliders reach physics world, poses follow unless running

    // contents
    void watch(World& world);                   // drops bodies as their entities die
    void detach(World& world, Entity entity);   // takes a body out, sync puts it back next step

    // contacts
    void setContactListener(BulletPhysics::dynamics::IContactListener* listener) { m_physicsWorld.setContactListener(listener); }   // not owned
    const std::vector<BulletPhysics::collision::Manifold>& getContacts() const { return m_physicsWorld.getContacts(); }

    // queries
    bool raycast(const BulletPhysics::collision::Ray& ray, BulletPhysics::collision::RayHit& outHit) const { return m_physicsWorld.raycast(ray, outHit); }

private:
    void publishTransforms(World& world);

    BulletPhysics::dynamics::PhysicsWorld m_physicsWorld;

    // entities the simulation already owns, a new one still takes its pose from the transform
    std::unordered_set<Entity> m_simulated;
};

} // namespace systems
} // namespace ecs
} // namespace BulletEngine
