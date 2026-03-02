/*
 * TrajectorySystem.cpp
 */

#include "TrajectorySystem.h"

namespace BulletEngine {
namespace ecs {
namespace systems {

void TrajectorySystem::update(World& world)
{
    for (auto entity : world.entities())
    {
        auto* transformComponent = world.get<TransformComponent>(entity);
        auto* trajectoryComponent = world.get<TrajectoryComponent>(entity);
        if (!transformComponent || !trajectoryComponent)
        {
            continue;
        }

        auto pos = transformComponent->transform.getPosition();
        BulletPhysics::math::Vec3 p{pos.x, pos.y, pos.z};

        // calculate distance to last point
        double distToLast = 0.0;
        if (!trajectoryComponent->points.empty())
        {
            const auto& last = trajectoryComponent->points.back();
            double dx = p.x - last.x;
            double dy = p.y - last.y;
            double dz = p.z - last.z;
            distToLast = std::sqrt(dx*dx + dy*dy + dz*dz);
        }

        if (trajectoryComponent->points.empty() || distToLast >= trajectoryComponent->minSegment)
        {
            trajectoryComponent->points.push_back(p);
        }

        if (trajectoryComponent->points.size() >= 2 && m_lines)
        {
            // convert Vec3(double) points to glm::vec3(float) for rendering
            std::vector<glm::vec3> renderPoints;
            renderPoints.reserve(trajectoryComponent->points.size());
            for (const auto& pt : trajectoryComponent->points)
            {
                renderPoints.push_back({static_cast<float>(pt.x), static_cast<float>(pt.y), static_cast<float>(pt.z)});
            }

            glm::vec3 renderColor{static_cast<float>(trajectoryComponent->color.x), static_cast<float>(trajectoryComponent->color.y), static_cast<float>(trajectoryComponent->color.z)};

            m_lines->addPolyline({renderPoints.data(), renderPoints.data() + renderPoints.size()}, renderColor);
        }
    }
}

} // namespace systems
} // namespace ecs
} // namespace BulletEngine
