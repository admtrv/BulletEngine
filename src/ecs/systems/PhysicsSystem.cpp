/*
 * PhysicsSystem.cpp
 */

#include "PhysicsSystem.h"

#include <unordered_set>

namespace BulletEngine {
namespace ecs {
namespace systems {

PhysicsSystem::PhysicsSystem()
{
    m_physicsWorld.setContactListener(this);
}

Entity PhysicsSystem::entityOf(const BulletPhysics::collision::collider::Collider* collider) const
{
    const auto it = m_owners.find(collider);
    return it != m_owners.end() ? it->second : INVALID_ENTITY;
}

void PhysicsSystem::report(ContactPhase phase, BulletPhysics::collision::collider::Collider* a, BulletPhysics::collision::collider::Collider* b, const glm::vec3& point, const glm::vec3& normal, float depth)
{
    const Entity first = entityOf(a);
    const Entity second = entityOf(b);

    // collider world no longer knows has no entity to tell
    if (first == INVALID_ENTITY || second == INVALID_ENTITY)
    {
        return;
    }

    const bool trigger = (a && a->isTrigger()) || (b && b->isTrigger());

    // normal runs from to b, so it points away from whoever is told
    m_events.push_back({phase, first, second, point, normal, depth, trigger});
    m_events.push_back({phase, second, first, point, -normal, depth, trigger});
}

void PhysicsSystem::onContactBegin(const BulletPhysics::collision::Manifold& manifold)
{
    const auto& info = manifold.info;

    const glm::vec3 normal{static_cast<float>(info.normal.x), static_cast<float>(info.normal.y), static_cast<float>(info.normal.z)};

    glm::vec3 point{};

    if (info.pointCount > 0)
    {
        const auto& position = info.points[0].position;
        point = {static_cast<float>(position.x), static_cast<float>(position.y), static_cast<float>(position.z)};
    }

    report(ContactPhase::Begin, manifold.colliderA, manifold.colliderB, point, normal, static_cast<float>(info.penetration));
}

void PhysicsSystem::onContactEnd(BulletPhysics::collision::collider::Collider* a, BulletPhysics::collision::collider::Collider* b)
{
    report(ContactPhase::End, a, b, {}, {}, 0.0f);
}

RayResult PhysicsSystem::raycast(const glm::vec3& origin, const glm::vec3& direction, float distance, BulletPhysics::collision::collider::LayerMask mask) const
{
    BulletPhysics::collision::Ray ray;
    ray.origin = {origin.x, origin.y, origin.z};
    ray.direction = {direction.x, direction.y, direction.z};
    ray.maxDistance = distance;

    BulletPhysics::collision::RayHit hit;

    if (!m_physicsWorld.raycast(ray, hit, mask))
    {
        return {};
    }

    const glm::vec3 point{static_cast<float>(hit.point.x), static_cast<float>(hit.point.y), static_cast<float>(hit.point.z)};
    const glm::vec3 normal{static_cast<float>(hit.normal.x), static_cast<float>(hit.normal.y), static_cast<float>(hit.normal.z)};

    return {true, entityOf(hit.collider), point, normal, static_cast<float>(hit.distance)};
}

void PhysicsSystem::watch(World& world)
{
    world.addListener([this, &world](Entity entity) { detach(world, entity); });
}

void PhysicsSystem::detach(World& world, Entity entity)
{
    if (auto* component = world.get<RigidBodyComponent>(entity))
    {
        m_physicsWorld.removeBody(&component->body);
    }
}

void PhysicsSystem::update(World& world, float dt)
{
    sync(world, false);

    // physics keeps its own clock, frame time only says how much of it passed
    m_physicsWorld.update(dt);

    publishTransforms(world);
}

void PhysicsSystem::step(World& world, float dt)
{
    sync(world, false);

    m_physicsWorld.step(dt);

    publishTransforms(world);
}

void PhysicsSystem::sync(World& world, bool adoptPoses)
{
    std::unordered_set<const BulletPhysics::dynamics::RigidBody*> alive;
    m_owners.clear();

    for (auto entity : world.getEntities())
    {
        auto* colliderComponent = world.get<ColliderComponent>(entity);
        auto* collider = colliderComponent ? colliderComponent->collider.get() : nullptr;
        auto* rigidBodyComponent = world.get<RigidBodyComponent>(entity);

        if (!collider && !rigidBodyComponent)
        {
            continue;
        }

        if (collider)
        {
            m_owners.emplace(collider, entity);
        }

        const auto* transformComponent = world.get<TransformComponent>(entity);

        // a shape without a body is placed straight from transform, nothing simulates it
        if (!rigidBodyComponent)
        {
            if (transformComponent)
            {
                const glm::vec3 position = transformComponent->transform.getPosition();
                const glm::quat rotation = transformComponent->transform.getRotation();

                collider->place({position.x, position.y, position.z}, {rotation.w, rotation.x, rotation.y, rotation.z});
            }

            m_physicsWorld.addCollider(collider);
            continue;
        }

        alive.insert(&rigidBodyComponent->body);

        // body born mid play has never been stepped, it still owes its pose to transform
        const bool isNew = m_simulated.insert(entity).second;

        // transform owns pose, simulation takes over once it runs
        if (transformComponent && (adoptPoses || isNew))
        {
            const glm::vec3 position = transformComponent->transform.getPosition();
            const glm::quat rotation = transformComponent->transform.getRotation();

            rigidBodyComponent->body.setPosition({position.x, position.y, position.z});
            rigidBodyComponent->body.setOrientation({rotation.w, rotation.x, rotation.y, rotation.z});
        }

        // shape and mass decide how body spins, both may change between steps
        if (collider)
        {
            rigidBodyComponent->body.setInverseInertiaLocal(collider->inverseInertia(rigidBodyComponent->body.getMass()));
        }

        // world takes each body once, second call would list it twice
        m_physicsWorld.addBody(&rigidBodyComponent->body, collider);
    }

    // entity that dropped its body starts over if it gets another
    std::erase_if(m_simulated, [&world](Entity entity) { return !world.has<RigidBodyComponent>(entity); });

    // body whose entity is gone has nothing left to follow
    const auto& bodies = m_physicsWorld.getBodies();

    for (size_t i = bodies.size(); i > 0; i--)
    {
        auto* body = bodies[i - 1];

        if (alive.count(body) == 0)
        {
            m_physicsWorld.removeBody(body);
        }
    }
}

void PhysicsSystem::publishTransforms(World& world)
{
    for (auto entity : world.getEntities())
    {
        auto* rigidBodyComponent = world.get<RigidBodyComponent>(entity);
        auto* transformComponent = world.get<TransformComponent>(entity);

        if (!rigidBodyComponent || !transformComponent)
        {
            continue;
        }

        const auto& position = rigidBodyComponent->body.getPosition();
        const auto& orientation = rigidBodyComponent->body.getOrientation();

        transformComponent->transform.setPosition({
            static_cast<float>(position.x),
            static_cast<float>(position.y),
            static_cast<float>(position.z)
        });

        transformComponent->transform.setRotation({
            static_cast<float>(orientation.w),
            static_cast<float>(orientation.x),
            static_cast<float>(orientation.y),
            static_cast<float>(orientation.z)
        });
    }
}

} // namespace systems
} // namespace ecs
} // namespace BulletEngine
