/*
 * PhysicsSystem.cpp
 */

#include "PhysicsSystem.h"

namespace BulletEngine {
namespace ecs {
namespace systems {

bool PhysicsSystem::beforeIntegrate(World& world, Entity entity, RigidBodyComponent& rigidBody, float dt)
{
    // check if projectile component
    auto* projectileComponent = world.get<ProjectileRigidBodyComponent>(entity);
    if (projectileComponent)
    {
        // don't integrate if grounded
        if (projectileComponent->isGrounded)
        {
            return false;
        }

        // let base class integrate
        return true;
    }

    return true; // let base class integrate
}

void PhysicsSystem::afterIntegrate(World& world, Entity entity, RigidBodyComponent& rigidBody, float dt)
{
    // check if projectile component
    auto* projectileComponent = world.get<ProjectileRigidBodyComponent>(entity);
    if (!projectileComponent)
    {
        return;
    }

    // update orientation based on velocity
    auto* transformComponent = world.get<TransformComponent>(entity);
    if (transformComponent && !projectileComponent->isGrounded)
    {
        const auto& v = rigidBody.body->getVelocity();
        double velLen2 = v.x*v.x + v.y*v.y + v.z*v.z;

        if (velLen2 > 1e-6)
        {
            transformComponent->transform.rotateFromDirection({0.0f, 1.0f, 0.0f}, {static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z)});
        }
    }

    // update box collider orientation
    auto* colliderComponent = world.get<ColliderComponent>(entity);
    if (colliderComponent && colliderComponent->collider && transformComponent)
    {
        // update orientation for box colliders
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

} // namespace systems
} // namespace ecs
} // namespace BulletEngine
