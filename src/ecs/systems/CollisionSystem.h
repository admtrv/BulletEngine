/*
 * CollisionSystem.h
 */

#pragma once

#include "ecs/Ecs.h"
#include "ecs/Components.h"

#include "collision/CollisionDetection.h"

#include <memory>

namespace BulletEngine {
namespace ecs {
namespace systems {

class CollisionSystemBase {
public:
    CollisionSystemBase();
    virtual ~CollisionSystemBase() = default;

    void update(World& world);

protected:
    // hooks
    virtual void onCollision(World&, Entity, Entity, const BulletPhysic::collision::Manifold&) {}

    std::unique_ptr<BulletPhysic::collision::CollisionDetection> m_collisionDetector;
};

} // namespace systems
} // namespace ecs
} // namespace BulletEngine
