/*
 * InputSystem.cpp
 */

#include "InputSystem.h"

namespace BulletEngine {
namespace ecs {
namespace systems {

InputSystem::InputSystem(ecs::World& world) : m_world(world)
{
    // bind keys
    BulletRender::utils::Input::instance().bindKey(
        BulletRender::utils::InputKey::SPACE,[this]() {
            launchProjectile();
        }
    );
}

void InputSystem::update(GLFWwindow* window)
{
    BulletRender::utils::Input::instance().update(window);
}

void InputSystem::setLaunchCallback(const std::function<void(ecs::Entity)>& callback)
{
    m_launchCallback = callback;
}

ecs::Entity InputSystem::createProjectile()
{
    ecs::Entity projectile = m_world.create();

    // transform
    auto& transformComponent = m_world.add<ecs::TransformComponent>(projectile);
    transformComponent.transform.setPosition({0.0f, 1.5f, 0.0f});

    // trajectory
    m_world.add<ecs::TrajectoryComponent>(projectile);

    // rigid body
    auto& rigidBodyComponent = m_world.add<ecs::RigidBodyComponent>(projectile);
    rigidBodyComponent.body.setMass(0.05f);
    rigidBodyComponent.body.setPosition({0.0f, 1.5f, 0.0f});
    rigidBodyComponent.body.setVelocity({0.0f, 0.0f, 0.0f});

    return projectile;
}

void InputSystem::launchProjectile()
{
    ecs::Entity projectile = createProjectile();

    auto* rigidBodyComponent = m_world.get<ecs::RigidBodyComponent>(projectile);
    if (rigidBodyComponent)
    {
        // launch with predefined angle and speed
        rigidBodyComponent->body.setVelocityFromAngles(25.0f, 45.0f, 90.0f);
    }

    if (m_launchCallback)
    {
        m_launchCallback(projectile);
    }
}

} // namespace systems
} // namespace ecs
} // namespace BulletEngine
