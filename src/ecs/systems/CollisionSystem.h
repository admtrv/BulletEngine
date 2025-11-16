/*
 * CollisionSystem.h
 */

#pragma once

#include "ecs/Ecs.h"
#include "ecs/Components.h"

#include "collision/GroundCollider.h"
#include "collision/BoxCollider.h"
#include "collision/CollisionDetection.h"

#include <memory>

namespace BulletEngine {
namespace ecs {
namespace systems {

class CollisionSystem {
public:
    CollisionSystem();

    void update(World& world);

private:
    std::unique_ptr<BulletPhysic::collision::CollisionDetection> m_collisionDetector;
};

} // namespace systems
} // namespace ecs
} // namespace BulletEngine
