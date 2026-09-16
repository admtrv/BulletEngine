/*
 * PhysicsSystem.cpp
 */

#include "PhysicsSystem.h"

#include <unordered_set>

namespace BulletEngine {
namespace ecs {
namespace systems {

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

    for (auto entity : world.getEntities())
    {
        auto* rigidBodyComponent = world.get<RigidBodyComponent>(entity);
        if (!rigidBodyComponent)
        {
            continue;
        }

        alive.insert(&rigidBodyComponent->body);

        // a body born mid play has never been stepped, it still owes its pose to the transform
        const bool isNew = m_simulated.insert(entity).second;

        // transform owns the pose, simulation takes over once it runs
        if (const auto* transformComponent = world.get<TransformComponent>(entity); transformComponent && (adoptPoses || isNew))
        {
            const glm::vec3 position = transformComponent->transform.getPosition();
            const glm::quat rotation = transformComponent->transform.getRotation();

            rigidBodyComponent->body.setPosition({position.x, position.y, position.z});
            rigidBodyComponent->body.setOrientation({rotation.w, rotation.x, rotation.y, rotation.z});
        }

        auto* colliderComponent = world.get<ColliderComponent>(entity);
        auto* collider = colliderComponent ? colliderComponent->collider.get() : nullptr;

        // shape and mass decide how the body spins, both may change between steps
        if (collider)
        {
            rigidBodyComponent->body.setInverseInertiaLocal(
                collider->inverseInertia(rigidBodyComponent->body.getMass()));
        }

        // the world takes each body once, a second call would list it twice
        m_physicsWorld.addBody(&rigidBodyComponent->body, collider);
    }

    // an entity that dropped its body starts over if it gets another
    std::erase_if(m_simulated, [&world](Entity entity) { return !world.has<RigidBodyComponent>(entity); });

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
