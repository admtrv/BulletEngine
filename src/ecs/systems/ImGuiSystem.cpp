/*
 * ImGuiSystem.cpp
 */

#include "ImGuiSystem.h"
#include "ecs/Components.h"

namespace BulletEngine {
namespace ecs {
namespace systems {

ImGuiSystem::ImGuiSystem(BulletPhysic::dynamics::PhysicsWorld& physicsWorld, BulletRender::scene::Camera& camera, World& world)
    : m_physicsWorld(physicsWorld), m_camera(camera), m_world(world)
{}

static void renderDebugWindow(BulletRender::scene::Camera& camera, float dt)
{
    ImGui::Begin("Debug");

    float fps = dt > 0.0f ? 1.0f / dt : 0.0f;
    ImGui::Text("FPS: %.1f", fps);

    ImGui::Separator();

    glm::vec3 camPos = camera.position();
    ImGui::Text("Position:");
    ImGui::Text("   X: %.4f", camPos.x);
    ImGui::Text("   Y: %.4f", camPos.y);
    ImGui::Text("   Z: %.4f", camPos.z);

    ImGui::End();
}

static void renderProjectileWindow(BulletPhysic::dynamics::PhysicsWorld& physicsWorld, World& world)
{
    ImGui::Begin("Projectile");

    // find last active projectile
    bool foundProjectile = false;
    Entity lastProjectile;
    for (const auto& entity : world.entities())
    {
        if (world.has<ProjectileRigidBodyComponent>(entity) && world.has<TransformComponent>(entity))
        {
            lastProjectile = entity;
            foundProjectile = true;
        }
    }

    if (foundProjectile)
    {
        const auto& transform = world.get<TransformComponent>(lastProjectile);
        glm::vec3 pos = transform->transform.getPosition();

        ImGui::Text("Position:");
        ImGui::Text("   X: %.4f", pos.x);
        ImGui::Text("   Y: %.4f", pos.y);
        ImGui::Text("   Z: %.4f", pos.z);

        ImGui::Separator();

        ImGui::Text("Forces:");
        for (const auto& force : physicsWorld.getForces())
        {
            if (force && force->isActive())
            {
                float magnitude = force->getForce().length();
                ImGui::Text("   %s: %.8f N", force->getSymbol().c_str(), magnitude);
            }
        }
    }
    else
    {
        ImGui::Text("No active projectile");
    }

    ImGui::End();
}

void ImGuiSystem::render(float dt)
{
    if (m_showDebug)
    {
        renderDebugWindow(m_camera, dt);
    }

    if (m_showProjectile)
    {
        renderProjectileWindow(m_physicsWorld, m_world);
    }
}

} // namespace systems
} // namespace ecs
} // namespace BulletEngine
