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
        double velLen2 = v.x*v.x + v.y*v.y + v.z*v.z;

        if (velLen2 > 1e-6)
        {
            transformComponent->transform.rotateFromDirection({0.0f, 1.0f, 0.0f}, {static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z)});
        }
    }

    // update box collider orientation
    auto* colliderComponent = world.get<ecs::ColliderComponent>(entity);
    if (colliderComponent && colliderComponent->collider && transformComponent)
    {
        if (colliderComponent->collider->getShape() == BulletPhysics::builtin::collision::collider::CollisionShape::Box)
        {
            auto* boxCollider = static_cast<BulletPhysics::builtin::collision::collider::BoxCollider*>(colliderComponent->collider.get());

            const auto& mat = transformComponent->transform.getMatrix();
            BulletPhysics::math::Vec3 axisX{mat[0].x, mat[0].y, mat[0].z};
            BulletPhysics::math::Vec3 axisY{mat[1].x, mat[1].y, mat[1].z};
            BulletPhysics::math::Vec3 axisZ{mat[2].x, mat[2].y, mat[2].z};

            boxCollider->setAxes(axisX, axisY, axisZ);
        }
    }
}

} // namespace BulletEngine
