/*
 * PhysicsSystem.cpp
 */

#include "PhysicsSystem.h"

#include <unordered_set>

namespace BulletEngine {
namespace ecs {
namespace systems {

void PhysicsSystem::update(World& world, float dt)
{
    syncBodies(world);

    // physics keeps its own clock, frame time only says how much of it passed
    m_physicsWorld.update(dt);

    publishTransforms(world);
}

void PhysicsSystem::step(World& world, float dt)
{
    syncBodies(world);

    m_physicsWorld.step(dt);

    publishTransforms(world);
}

void PhysicsSystem::syncBodies(World& world)
{
    std::unordered_set<const BulletPhysics::dynamics::RigidBody*> alive;

    for (auto entity : world.entities())
    {
        auto* rigidBodyComponent = world.get<RigidBodyComponent>(entity);
        if (!rigidBodyComponent)
        {
            continue;
        }

        alive.insert(&rigidBodyComponent->body);

        auto* colliderComponent = world.get<ColliderComponent>(entity);
        auto* collider = colliderComponent ? colliderComponent->collider.get() : nullptr;

        // the world takes each body once, a second call would list it twice
        m_physicsWorld.addBody(&rigidBodyComponent->body, collider);
    }

    // a body whose entity is gone has nothing left to follow
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
    for (auto entity : world.entities())
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
