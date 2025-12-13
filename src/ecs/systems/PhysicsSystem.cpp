/*
 * PhysicsSystem.cpp
 */

#include "PhysicsSystem.h"

namespace BulletEngine {
namespace ecs {
namespace systems {

PhysicsSystemBase::PhysicsSystemBase(BulletPhysic::dynamics::PhysicsWorld& physicsWorld, BulletPhysic::math::IIntegrator& integrator)
    : m_physicsWorld(physicsWorld)
    , m_integrator(integrator)
{}

void PhysicsSystemBase::update(World& world, float dt)
{
    for (auto entity : world.entities())
    {
        auto* transformComponent = world.get<TransformComponent>(entity);
        auto* rigidBodyComponent = world.get<RigidBodyComponent>(entity);

        if (!rigidBodyComponent || !rigidBodyComponent->body)
        {
            continue;
        }

        // apply forces
        if (beforeIntegrate(world, entity, *rigidBodyComponent, dt))
        {
            m_integrator.step(*rigidBodyComponent->body, &m_physicsWorld, dt);
        }

        afterIntegrate(world, entity, *rigidBodyComponent, dt);

        // update transform
        if (transformComponent)
        {
            const auto& p = rigidBodyComponent->body->getPosition();
            transformComponent->transform.setPosition({p.x, p.y, p.z});
        }

        // update collider
        auto* colliderComponent = world.get<ColliderComponent>(entity);
        if (colliderComponent && colliderComponent->collider && transformComponent)
        {
            const auto& p = rigidBodyComponent->body->getPosition();
            colliderComponent->collider->setPosition(p);
        }
    }
}

} // namespace systems
} // namespace ecs
} // namespace BulletEngine
