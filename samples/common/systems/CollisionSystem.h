/*
 * CollisionSystem.h
 */

#pragma once

#include "ecs/systems/CollisionSystem.h"
#include "common/Components.h"

namespace BulletEngine {
namespace ecs {
namespace systems {

class CollisionSystem : public CollisionSystemBase {
public:
    using CollisionSystemBase::CollisionSystemBase;

protected:
    void onCollision(World& world, Entity entityA, Entity entityB, const BulletPhysics::collision::Manifold& manifold) override;
};

} // namespace systems
} // namespace ecs
} // namespace BulletEngine
