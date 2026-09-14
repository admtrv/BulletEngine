/*
 * PickSystem.h
 */

#pragma once

#include "ecs/Ecs.h"
#include "ecs/systems/PhysicsSystem.h"

#include "scene/Camera.h"

#include "collision/Query.h"

#include <glm/glm.hpp>

namespace BulletEngine {
namespace ecs {
namespace systems {

// entity under the cursor, found by casting a ray into the world
class PickSystem {
public:
    PickSystem(BulletRender::scene::Camera& camera, const PhysicsSystem& physics);

    Entity pick(World& world, const glm::vec2& cursor, const glm::vec2& viewport) const;

private:
    BulletPhysics::collision::Ray rayThroughCursor(const glm::vec2& cursor, const glm::vec2& viewport) const;

    BulletRender::scene::Camera& m_camera;
    const PhysicsSystem& m_physics;
};

} // namespace systems
} // namespace ecs
} // namespace BulletEngine
