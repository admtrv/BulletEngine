/*
 * TrajectorySystem.cpp
 */

#include "TrajectorySystem.h"

namespace BulletEngine {
namespace ecs {
namespace systems {

glm::vec3 EnergyTrajectorySystem::energyToColor(float ratio)
{
    // ratio: 1.0 = full energy, 0.0 = zero energy
    // 1.0 -> red (1, 0, 0)
    // 0.66 -> orange (1, 0.5, 0)
    // 0.33 -> yellow (1, 1, 0)
    // 0.0 -> green (0, 1, 0)

    ratio = std::clamp(ratio, 0.0f, 1.0f);

    float r, g;

    if (ratio > 0.5f)
    {
        // red to orange to yellow: r=1, g goes 0 -> 1
        float t = (1.0f - ratio) * 2.0f; // 0..1
        r = 1.0f;
        g = t;
    }
    else
    {
        // yellow to green: r goes 1 -> 0, g=1
        float t = ratio * 2.0f; // 1..0
        r = t;
        g = 1.0f;
    }

    return {r, g, 0.0f};
}

void EnergyTrajectorySystem::update(World& world)
{
    for (auto entity : world.entities())
    {
        auto* transformComponent = world.get<TransformComponent>(entity);
        auto* trajectoryComponent = world.get<EnergyTrajectoryComponent>(entity);
        auto* rigidBodyComponent = world.get<ProjectileRigidBodyComponent>(entity);

        if (!transformComponent || !trajectoryComponent || !rigidBodyComponent)
        {
            continue;
        }

        auto pos = transformComponent->transform.getPosition();
        BulletPhysics::math::Vec3 p{pos.x, pos.y, pos.z};

        // calculate current kinetic energy
        auto vel = rigidBodyComponent->body->getVelocity();
        double speed = vel.length();
        double mass = rigidBodyComponent->getProjectileBody().getMass();
        double energy = 0.5 * mass * speed * speed;

        // record initial energy on first point
        if (trajectoryComponent->points.empty())
        {
            trajectoryComponent->initialEnergy = energy;
        }

        // check distance to last point
        double distToLast = 0.0;
        if (!trajectoryComponent->points.empty())
        {
            const auto& last = trajectoryComponent->points.back().position;
            double dx = p.x - last.x;
            double dy = p.y - last.y;
            double dz = p.z - last.z;
            distToLast = std::sqrt(dx * dx + dy * dy + dz * dz);
        }

        if (trajectoryComponent->points.empty() || distToLast >= trajectoryComponent->minSegment)
        {
            trajectoryComponent->points.push_back({p, energy});
        }

        // render segments with energy-based color
        if (trajectoryComponent->points.size() >= 2 && m_lines)
        {
            double initE = trajectoryComponent->initialEnergy;
            if (initE < 1e-9) initE = 1.0;

            for (size_t i = 0; i + 1 < trajectoryComponent->points.size(); ++i)
            {
                const auto& a = trajectoryComponent->points[i];
                const auto& b = trajectoryComponent->points[i + 1];

                // average energy of segment endpoints for smooth color
                double avgEnergy = (a.energy + b.energy) * 0.5;
                float ratio = static_cast<float>(avgEnergy / initE);

                glm::vec3 color = energyToColor(ratio);
                glm::vec3 pa{static_cast<float>(a.position.x), static_cast<float>(a.position.y), static_cast<float>(a.position.z)};
                glm::vec3 pb{static_cast<float>(b.position.x), static_cast<float>(b.position.y), static_cast<float>(b.position.z)};

                m_lines->addLine(pa, pb, color);
            }
        }
    }
}

} // namespace systems
} // namespace ecs
} // namespace BulletEngine
