/*
 * PhysicsSystem.cpp
 */

#include "PhysicsSystem.h"

namespace BulletEngine {

bool PhysicsSystem::beforeIntegrate(ecs::World& world, ecs::Entity entity, ecs::RigidBodyComponent& rigidBody, float dt)
{
    auto* projectile = world.get<ecs::ProjectileRigidBodyComponent>(entity);
    if (!projectile)
    {
        // let base class handle it
        return true;
    }

    // only update target
    if (m_targetId && *m_targetId != 0 && entity != *m_targetId)
    {
        // not our target, skip it
        return false;
    }

    // don't integrate if grounded
    if (projectile->isGrounded)
    {
        return false;
    }

    return true; // let base class integrate
}

void PhysicsSystem::afterIntegrate(ecs::World& world, ecs::Entity entity, ecs::RigidBodyComponent& rigidBody, float dt)
{
    auto* projectile = world.get<ecs::ProjectileRigidBodyComponent>(entity);
    if (!projectile)
    {
        return;
    }

    // skip if not our target
    if (m_targetId && *m_targetId != 0 && entity != *m_targetId)
    {
        return;
    }

    // update orientation based on velocity
    auto* transformComponent = world.get<ecs::TransformComponent>(entity);
    if (transformComponent && !projectile->isGrounded)
    {
        const auto& v = rigidBody.body->getVelocity();
        float velLen2 = v.x*v.x + v.y*v.y + v.z*v.z;

        if (velLen2 > 1e-6f)
        {
            transformComponent->transform.rotateFromDirection({0.0f, 1.0f, 0.0f}, {v.x, v.y, v.z});
        }
    }

    // update box collider orientation
    auto* colliderComponent = world.get<ecs::ColliderComponent>(entity);
    if (colliderComponent && colliderComponent->collider && transformComponent)
    {
        if (colliderComponent->collider->getShape() == BulletPhysic::collision::CollisionShape::Box)
        {
            auto* boxCollider = static_cast<BulletPhysic::collision::BoxCollider*>(colliderComponent->collider.get());

            auto axisX = reinterpret_cast<const BulletPhysic::math::Vec3&>(transformComponent->transform.getMatrix()[0]);
            auto axisY = reinterpret_cast<const BulletPhysic::math::Vec3&>(transformComponent->transform.getMatrix()[1]);
            auto axisZ = reinterpret_cast<const BulletPhysic::math::Vec3&>(transformComponent->transform.getMatrix()[2]);

            boxCollider->setAxes(axisX, axisY, axisZ);
        }
    }
}

} // namespace BulletEngine
