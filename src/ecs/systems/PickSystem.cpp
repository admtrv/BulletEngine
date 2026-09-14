/*
 * PickSystem.cpp
 */

#include "PickSystem.h"

#include "ecs/Components.h"

#include <glm/gtc/matrix_transform.hpp>

namespace BulletEngine {
namespace ecs {
namespace systems {

PickSystem::PickSystem(BulletRender::scene::Camera& camera, const PhysicsSystem& physics)
    : m_camera(camera), m_physics(physics) {}

Entity PickSystem::pick(World& world, const glm::vec2& cursor, const glm::vec2& viewport) const
{
    if (viewport.x <= 0.0f || viewport.y <= 0.0f)
    {
        return INVALID_ENTITY;
    }

    BulletPhysics::collision::RayHit hit;

    if (!m_physics.raycast(rayThroughCursor(cursor, viewport), hit) || !hit.collider)
    {
        return INVALID_ENTITY;
    }

    for (Entity entity : world.getEntities())
    {
        const auto* component = world.get<ColliderComponent>(entity);

        if (component && component->collider.get() == hit.collider)
        {
            return entity;
        }
    }

    return INVALID_ENTITY;
}

BulletPhysics::collision::Ray PickSystem::rayThroughCursor(const glm::vec2& cursor, const glm::vec2& viewport) const
{
    // cursor to clip space, y runs down the screen and up in clip space
    const float clipX = 2.0f * cursor.x / viewport.x - 1.0f;
    const float clipY = 1.0f - 2.0f * cursor.y / viewport.y;

    const glm::mat4 inverse = glm::inverse(m_camera.getProj(viewport.x / viewport.y) * m_camera.getView());

    // unproject the near and far ends of the pixel, the line between them is the ray
    glm::vec4 near = inverse * glm::vec4(clipX, clipY, -1.0f, 1.0f);
    glm::vec4 far = inverse * glm::vec4(clipX, clipY, 1.0f, 1.0f);

    near /= near.w;
    far /= far.w;

    const glm::vec3 direction = glm::normalize(glm::vec3(far - near));

    BulletPhysics::collision::Ray ray;
    ray.origin = {near.x, near.y, near.z};
    ray.direction = {direction.x, direction.y, direction.z};
    ray.maxDistance = glm::length(glm::vec3(far - near));

    return ray;
}

} // namespace systems
} // namespace ecs
} // namespace BulletEngine
