/*
 * TrajectorySystem.cpp
 */

#include "TrajectorySystem.h"

#include <glm/glm.hpp>

namespace BulletEngine {
namespace systems {

void TrajectorySystem::update(ecs::World& world)
{
    for (auto entity : world.entities())
    {
        auto* transformComponent = world.get<ecs::TransformComponent>(entity);
        auto* trajectoryComponent = world.get<ecs::TrajectoryComponent>(entity);
        if (!transformComponent || !trajectoryComponent)
        {
            continue;
        }

        glm::vec3 p = glm::vec3(transformComponent->transform.getMatrix()[3]);

        if (trajectoryComponent->points.empty() || glm::distance(trajectoryComponent->points.back(), p) >= trajectoryComponent->minSegment)
        {
            trajectoryComponent->points.push_back(p);
        }

        if (trajectoryComponent->points.size() >= 2 && m_lines)
        {
            m_lines->addPolyline(trajectoryComponent->points, trajectoryComponent->color);
        }
    }
}

} // namespace systems
} // namespace BulletEngine
