/*
 * BallisticSystem.cpp
 */

#include "BallisticSystem.h"
#include "dynamics/forces/WindDragForce.h"

namespace BulletEngine {
namespace ecs {
namespace systems {

BallisticSystem::BallisticSystem(BulletPhysic::math::IIntegrator& integrator) : m_integrator(integrator)
{
    setRealismLevel(BulletPhysic::preset::RealismLevel::BASIC); // start with basic simulation
}

void BallisticSystem::update(ecs::World& world, float dt)
{
    for (auto entity : world.entities())
    {
        auto* transformComponent = world.get<ecs::TransformComponent>(entity);
        auto* rigidBodyComponent = world.get<ecs::RigidBodyComponent>(entity);

        if (!transformComponent || !rigidBodyComponent)
        {
            continue;
        }

        m_integrator.step(rigidBodyComponent->body, &m_forceRegistry, dt);

        const auto& p = rigidBodyComponent->body.position();
        const auto& v = rigidBodyComponent->body.velocity();

        glm::vec3 pos(p.x, p.y, p.z);
        glm::vec3 vel(v.x, v.y, v.z);

        glm::mat4 M = transformComponent->transform.getMatrix();

        // update position
        M[3] = glm::vec4(pos, 1.0f);

        // update orientation based on velocity
        if (glm::length2(vel) > 1e-6f)
        {
            glm::vec3 dir = glm::normalize(vel);

            // local model forward - axis of the tip
            const glm::vec3 modelForward(0.0f, 1.0f, 0.0f);

            glm::quat q = glm::rotation(modelForward, dir);
            glm::mat4 R = glm::toMat4(q);

            M[0] = glm::vec4(R[0].x, R[0].y, R[0].z, 0.0f);
            M[1] = glm::vec4(R[1].x, R[1].y, R[1].z, 0.0f);
            M[2] = glm::vec4(R[2].x, R[2].y, R[2].z, 0.0f);
        }

        transformComponent->transform.setMatrix(M);
    }
}

void BallisticSystem::setRealismLevel(BulletPhysic::preset::RealismLevel level, float projectileArea)
{
    BulletPhysic::preset::PresetManager::configure(m_forceRegistry, level, projectileArea);
}

void BallisticSystem::setWindVelocity(const BulletPhysic::math::Vec3& windVel)
{
    auto wind = m_forceRegistry.getByName(std::string("Wind Drag"));
    if (wind)
    {
        static_cast<BulletPhysic::dynamics::forces::WindDragForce*>(wind)->setWindVelocity(windVel);
    }
}

} // namespace systems
} // namespace ecs
} // namespace BulletEngine
