/*
 * CollisionSystem.cpp
 */

#include "CollisionSystem.h"

namespace BulletEngine {
namespace ecs {
namespace systems {

CollisionSystem::CollisionSystem()
    : m_collisionDetector(std::make_unique<BulletPhysic::collision::CollisionDetection>()) {}

void CollisionSystem::update(World& world)
{
    // register all colliders
    m_collisionDetector->clear();

    for (auto entity : world.entities())
    {
        auto* colliderComponent = world.get<ColliderComponent>(entity);
        if (colliderComponent && colliderComponent->collider)
        {
            m_collisionDetector->addCollider(colliderComponent->collider.get());
        }
    }

    // detect collisions
    std::vector<BulletPhysic::collision::Manifold> manifolds;
    m_collisionDetector->detect(manifolds);

    // handle collisions
    for (const auto& manifold : manifolds)
    {
        // find the entities that collided
        BulletPhysic::collision::Collider* groundCollider = nullptr;
        BulletPhysic::collision::Collider* otherCollider = nullptr;

        // check which one is ground
        if (manifold.colliderA->getShape() == BulletPhysic::collision::CollisionShape::Ground)
        {
            groundCollider = manifold.colliderA;
            otherCollider = manifold.colliderB;
        }
        else if (manifold.colliderB->getShape() == BulletPhysic::collision::CollisionShape::Ground)
        {
            groundCollider = manifold.colliderB;
            otherCollider = manifold.colliderA;
        }

        if (groundCollider && otherCollider)
        {
            // find the entity with this collider
            for (auto entity : world.entities())
            {
                auto* colliderComponent = world.get<ColliderComponent>(entity);
                if (colliderComponent && colliderComponent->collider.get() == otherCollider)
                {
                    auto* rigidBodyComponent = world.get<ProjectileRigidBodyComponent>(entity);
                    if (rigidBodyComponent)
                    {
                        // ground collision detected
                        rigidBodyComponent->body.setVelocity({0.0f, 0.0f, 0.0f});
                        rigidBodyComponent->body.setGrounded(true);
                    }
                    break;
                }
            }
        }
    }
}

} // namespace systems
} // namespace ecs
} // namespace BulletEngine
