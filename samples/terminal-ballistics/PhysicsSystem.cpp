/*
 * PhysicsSystem.cpp
 */

#include "PhysicsSystem.h"

namespace BulletEngine {
namespace ecs {
namespace systems {

bool PhysicsSystem::beforeIntegrate(World& world, Entity entity, RigidBodyComponent& rigidBody, float dt)
{
    auto* projectileComponent = world.get<ProjectileRigidBodyComponent>(entity);
    if (projectileComponent)
    {
        // don't integrate if grounded
        if (projectileComponent->isGrounded)
        {
            return false;
        }

        return true;
    }

    return true;
}

void PhysicsSystem::afterIntegrate(World& world, Entity entity, RigidBodyComponent& rigidBody, float dt)
{
    auto* projectileComponent = world.get<ProjectileRigidBodyComponent>(entity);
    if (!projectileComponent)
    {
        return;
    }

    // update orientation based on velocity
    auto* impactState = world.get<ImpactStateComponent>(entity);
    bool impacted = impactState && impactState->hasImpacted;

    auto* transformComponent = world.get<TransformComponent>(entity);
    if (transformComponent && !impacted)
    {
        const auto& v = rigidBody.body->getVelocity();
        float velLen2 = v.x*v.x + v.y*v.y + v.z*v.z;

        if (velLen2 > 1e-6f)
        {
            transformComponent->transform.rotateFromDirection({0.0f, 1.0f, 0.0f}, {v.x, v.y, v.z});
        }
    }

    // update box collider orientation
    auto* colliderComponent = world.get<ColliderComponent>(entity);
    if (colliderComponent && colliderComponent->collider && transformComponent)
    {
        if (colliderComponent->collider->getShape() == BulletPhysics::collision::CollisionShape::Box)
        {
            auto* boxCollider = static_cast<BulletPhysics::collision::BoxCollider*>(colliderComponent->collider.get());

            auto axisX = reinterpret_cast<const BulletPhysics::math::Vec3&>(transformComponent->transform.getMatrix()[0]);
            auto axisY = reinterpret_cast<const BulletPhysics::math::Vec3&>(transformComponent->transform.getMatrix()[1]);
            auto axisZ = reinterpret_cast<const BulletPhysics::math::Vec3&>(transformComponent->transform.getMatrix()[2]);

            boxCollider->setAxes(axisX, axisY, axisZ);
        }
    }
}

} // namespace systems
} // namespace ecs
} // namespace BulletEngine
