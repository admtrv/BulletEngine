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
        float distToLast = 0.0f;
        if (!trajectoryComponent->points.empty())
        {
            const auto& last = trajectoryComponent->points.back();
            float dx = p.x - last.x;
            float dy = p.y - last.y;
            float dz = p.z - last.z;
            distToLast = std::sqrt(dx*dx + dy*dy + dz*dz);
        }

        if (trajectoryComponent->points.empty() || distToLast >= trajectoryComponent->minSegment)
        {
            trajectoryComponent->points.push_back(p);
        }

        if (trajectoryComponent->points.size() >= 2 && m_lines)
        {
            const auto* renderPoints = reinterpret_cast<const decltype(pos)*>(trajectoryComponent->points.data());
            const auto& renderColor = reinterpret_cast<const decltype(pos)&>(trajectoryComponent->color);

            m_lines->addPolyline({renderPoints, renderPoints + trajectoryComponent->points.size()}, renderColor);
        }
    }
}

} // namespace systems
} // namespace ecs
} // namespace BulletEngine
