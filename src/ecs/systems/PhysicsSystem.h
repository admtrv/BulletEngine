/*
 * PhysicsSystem.h
 */

#pragma once

#include "ecs/Ecs.h"
#include "ecs/Components.h"
#include "ecs/systems/PhysicsEvents.h"

#include "dynamics/PhysicsWorld.h"
#include "dynamics/contact/ContactListener.h"

#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace BulletEngine {
namespace ecs {
namespace systems {

// owns simulation and turns its contacts into entity events
class PhysicsSystem : public BulletPhysics::dynamics::IContactListener {
public:
    PhysicsSystem();

    // simulation
    void update(World& world, float dt);    // own clock, runs as many steps as frame owes
    void step(World& world, float dt);      // one step, paced from outside
    void sync(World& world, bool adoptPoses = true);    // colliders reach physics world, poses follow unless running

    // contents
    void watch(World& world);                   // drops bodies as their entities die
    void detach(World& world, Entity entity);   // takes a body out, sync puts it back next step

    // contacts
    const std::vector<BulletPhysics::collision::Manifold>& getContacts() const { return m_physicsWorld.getContacts(); }

    // what last step ran into, taken by whoever acts on it
    const std::vector<ContactEvent>& getEvents() const { return m_events; }
    void clearEvents() { m_events.clear(); }

    // queries
    bool raycast(const BulletPhysics::collision::Ray& ray, BulletPhysics::collision::RayHit& outHit) const { return m_physicsWorld.raycast(ray, outHit); }

    // answers with entity rather than collider, what scripts and gameplay ask for
    RayResult raycast(const glm::vec3& origin, const glm::vec3& direction, float distance, BulletPhysics::collision::collider::LayerMask mask = BulletPhysics::collision::collider::LAYER_ALL) const;

private:
    // simulation reports here, mid step, so events only pile up
    void onContactBegin(const BulletPhysics::collision::Manifold& manifold) override;
    void onContactEnd(BulletPhysics::collision::collider::Collider* a, BulletPhysics::collision::collider::Collider* b) override;

    // both sides hear about pair, each with other named
    void report(ContactPhase phase, BulletPhysics::collision::collider::Collider* a, BulletPhysics::collision::collider::Collider* b, const glm::vec3& point, const glm::vec3& normal, float depth);

    Entity entityOf(const BulletPhysics::collision::collider::Collider* collider) const;

    void publishTransforms(World& world);

    BulletPhysics::dynamics::PhysicsWorld m_physicsWorld;

    // entities the simulation already owns, a new one still takes its pose from the transform
    std::unordered_set<Entity> m_simulated;

    // rebuilt by sync, so contact can name entity behind collider
    std::unordered_map<const BulletPhysics::collision::collider::Collider*, Entity> m_owners;

    // filled mid step, emptied by whoever delivered them
    std::vector<ContactEvent> m_events;
};

} // namespace systems
} // namespace ecs
} // namespace BulletEngine
