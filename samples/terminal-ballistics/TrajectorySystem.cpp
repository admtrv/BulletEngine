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
    // ratio = ratio * ratio; // quadratic falloff

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
        float speed = vel.length();
        float mass = rigidBodyComponent->getProjectileBody().getMass();
        float energy = 0.5f * mass * speed * speed;

        // record initial energy on first point
        if (trajectoryComponent->points.empty())
        {
            trajectoryComponent->initialEnergy = energy;
        }

        // check distance to last point
        float distToLast = 0.0f;
        if (!trajectoryComponent->points.empty())
        {
            const auto& last = trajectoryComponent->points.back().position;
            float dx = p.x - last.x;
            float dy = p.y - last.y;
            float dz = p.z - last.z;
            distToLast = std::sqrt(dx * dx + dy * dy + dz * dz);
        }

        if (trajectoryComponent->points.empty() || distToLast >= trajectoryComponent->minSegment)
        {
            trajectoryComponent->points.push_back({p, energy});
        }

        // render segments with energy-based color
        if (trajectoryComponent->points.size() >= 2 && m_lines)
        {
            float initE = trajectoryComponent->initialEnergy;
            if (initE < 1e-9f) initE = 1.0f;

            for (size_t i = 0; i + 1 < trajectoryComponent->points.size(); ++i)
            {
                const auto& a = trajectoryComponent->points[i];
                const auto& b = trajectoryComponent->points[i + 1];

                // average energy of segment endpoints for smooth color
                float avgEnergy = (a.energy + b.energy) * 0.5f;
                float ratio = avgEnergy / initE;

                glm::vec3 color = energyToColor(ratio);
                glm::vec3 pa{a.position.x, a.position.y, a.position.z};
                glm::vec3 pb{b.position.x, b.position.y, b.position.z};

                m_lines->addLine(pa, pb, color);
            }
        }
    }
}

} // namespace systems
} // namespace ecs
} // namespace BulletEngine
