/*
 * CollisionSystem.cpp
 */

#include "CollisionSystem.h"

namespace BulletEngine {
namespace ecs {
namespace systems {

CollisionSystemBase::CollisionSystemBase() : m_collisionDetector(std::make_unique<BulletPhysics::collision::CollisionDetection>()) {}

void CollisionSystemBase::update(World& world)
{
    // register all colliders
    m_collisionDetector->clear();

    std::unordered_map<BulletPhysics::collision::Collider*, Entity> colliderToEntity;    // map collider -> entity

    for (auto entity : world.entities())
    {
        auto* colliderComponent = world.get<ColliderComponent>(entity);
        if (colliderComponent && colliderComponent->collider)
        {
            auto* collider = colliderComponent->collider.get();
            m_collisionDetector->addCollider(collider);
            colliderToEntity[collider] = entity;
        }
    }

    // detect collisions
    std::vector<BulletPhysics::collision::Manifold> manifolds;
    m_collisionDetector->detect(manifolds);

    // handle collisions
    for (const auto& manifold : manifolds)
    {
        auto itA = colliderToEntity.find(manifold.colliderA);
        auto itB = colliderToEntity.find(manifold.colliderB);

        if (itA == colliderToEntity.end() || itB == colliderToEntity.end())
        {
            continue;
        }

        onCollision(world, itA->second, itB->second, manifold);
    }
}

} // namespace systems
} // namespace ecs
} // namespace BulletEngine
