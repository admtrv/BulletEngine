/*
 * CollisionSystem.cpp
 */

#include "CollisionSystem.h"

namespace BulletEngine {
namespace ecs {
namespace systems {

void CollisionSystem::onCollision(World& world, Entity entityA, Entity entityB, const BulletPhysic::collision::Manifold& manifold)
{
    // determine which is ground and which is projectile
    Entity groundEntity = 0;
    Entity projectileEntity = 0;

    auto* colliderA = world.get<ColliderComponent>(entityA);
    auto* colliderB = world.get<ColliderComponent>(entityB);

    if (!colliderA || !colliderB)
    {
        return;
    }

    // check which one is ground
    if (colliderA->collider->getShape() == BulletPhysic::collision::CollisionShape::Ground)
    {
        groundEntity = entityA;
        projectileEntity = entityB;
    }
    else if (colliderB->collider->getShape() == BulletPhysic::collision::CollisionShape::Ground)
    {
        groundEntity = entityB;
        projectileEntity = entityA;
    }
    else
    {
        return; // neither is ground, ignore
    }

    // handle ground collision
    auto* rigidBodyComponent = world.get<ProjectileRigidBodyComponent>(projectileEntity);
    if (rigidBodyComponent)
    {
        // stop projectile and mark as grounded
        rigidBodyComponent->getProjectileBody().setVelocity({0.0f, 0.0f, 0.0f});
        rigidBodyComponent->isGrounded = true;
    }
}

} // namespace systems
} // namespace ecs
} // namespace BulletEngine
