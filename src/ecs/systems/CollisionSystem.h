/*
 * CollisionSystem.h
 */

#pragma once

#include "ecs/Ecs.h"
#include "ecs/Components.h"

#include "builtin/collision/Collision.h"

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
    virtual void onCollision(World&, Entity, Entity, const BulletPhysics::builtin::collision::Manifold&) {}

    std::unique_ptr<BulletPhysics::builtin::collision::Collision> m_collisionDetector;
};

} // namespace systems
} // namespace ecs
} // namespace BulletEngine
