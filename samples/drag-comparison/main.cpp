/*
 * main.cpp
 */

// BulletRender
#include "app/Window.h"
#include "app/Loop.h"
#include "render/passes/Grid.h"
#include "render/passes/WorldAxis.h"
#include "render/passes/Fog.h"
#include "render/Renderer.h"
#include "render/Shader.h"
#include "scene/Scene.h"
#include "scene/Model.h"
#include "scene/Camera.h"
#include "scene/Light.h"
#include "imgui.h"

// BulletPhysic
#include "math/Integrator.h"
#include "math/Angles.h"
#include "collision/BoxCollider.h"
#include "collision/GroundCollider.h"
#include "dynamics/environment/Atmosphere.h"
#include "dynamics/environment/Geographic.h"
#include "dynamics/environment/Humidity.h"
#include "dynamics/forces/drag/Drag.h"
#include "dynamics/forces/Gravity.h"

// BulletEngine
#include "ecs/Ecs.h"
#include "ecs/systems/RenderSystem.h"
#include "ecs/systems/InputSystem.h"
#include "ecs/systems/ImGuiSystem.h"

// common
#include "common/Components.h"
#include "common/systems/CollisionSystem.h"
#include "common/systems/TrajectorySystem.h"
#include "common/objects/Projectile.h"

// local
#include "PhysicsSystem.h"

using namespace BulletEngine;

using RenderSystem = ecs::systems::RenderSystemBase;
using ImGuiSystem = ecs::systems::ImGuiSystemBase;

// colors
const BulletPhysic::math::Vec3 RED_COLOR{1.0f, 0.2f, 0.2f};
const BulletPhysic::math::Vec3 BLUE_COLOR{0.2f, 0.4f, 1.0f};

// entity IDs
ecs::Entity gravityProjectileId = 0;
ecs::Entity dragProjectileId = 0;

void launchBoth(ecs::World& world)
{
    // gravity-only projectile
    gravityProjectileId = objects::Projectile::launch(world);

    auto* groundTrajectory = world.get<ecs::TrajectoryComponent>(gravityProjectileId);
    if (groundTrajectory)
    {
        groundTrajectory->color = RED_COLOR;
    }

    // with drag projectile
    dragProjectileId = objects::Projectile::launch(world);

    auto* dragProjectile = world.get<ecs::TrajectoryComponent>(dragProjectileId);
    if (dragProjectile)
    {
        dragProjectile->color = BLUE_COLOR;
    }
}

// imgui display functions
void setupDebugDisplay(ImGuiSystem& imgui, BulletRender::scene::Camera& camera, float& dt)
{
    imgui.add([&camera, &dt]() {
        ImGui::Begin("Debug");

        float fps = dt > 0.0f ? 1.0f / dt : 0.0f;
        ImGui::Text("FPS: %.1f", fps);

        ImGui::Separator();

        auto p = camera.position();
        ImGui::Text("Camera:");
        ImGui::Text("   X: %.2f", p.x);
        ImGui::Text("   Y: %.2f", p.y);
        ImGui::Text("   Z: %.2f", p.z);

        ImGui::End();
    });
}

void setupProjectilesDisplay(ImGuiSystem& imgui, ecs::World& world)
{
    imgui.add([&world]() {
        ImGui::Begin("Projectiles Comparison");

        // gravity-only projectile
        ImGui::TextColored(ImVec4(RED_COLOR.x, RED_COLOR.y, RED_COLOR.z, 1.0f), "Gravity Only:");
        if (gravityProjectileId != 0 && world.has<ecs::ProjectileRigidBodyComponent>(gravityProjectileId))
        {
            auto* projectile = world.get<ecs::ProjectileRigidBodyComponent>(gravityProjectileId);

            if (projectile)
            {
                auto pos = projectile->getProjectileBody().getPosition();
                ImGui::Text("   Position: (%.2f, %.2f, %.2f)", pos.x, pos.y, pos.z);

                auto vel = projectile->getProjectileBody().getVelocity();
                float speed = std::sqrt(vel.x*vel.x + vel.y*vel.y + vel.z*vel.z);
                ImGui::Text("   Speed: %.2f m/s", speed);
            }
        }
        else
        {
            ImGui::Text("   Not launched");
        }

        ImGui::Separator();

        // with drag projectile
        ImGui::TextColored(ImVec4(BLUE_COLOR.x, BLUE_COLOR.y, BLUE_COLOR.z, 1.0f), "With Drag:");
        if (dragProjectileId != 0 && world.has<ecs::ProjectileRigidBodyComponent>(dragProjectileId))
        {
            auto* projectile = world.get<ecs::ProjectileRigidBodyComponent>(dragProjectileId);

            if (projectile)
            {
                auto pos = projectile->getProjectileBody().getPosition();
                ImGui::Text("   Position: (%.2f, %.2f, %.2f)", pos.x, pos.y, pos.z);

                auto vel = projectile->getProjectileBody().getVelocity();
                float speed = std::sqrt(vel.x*vel.x + vel.y*vel.y + vel.z*vel.z);
                ImGui::Text("   Speed: %.2f m/s", speed);
            }
        }
        else
        {
            ImGui::Text("   Not launched");
        }

        ImGui::End();
    });
}

int main()
{
    // window
    BulletRender::app::WindowConfig windowCfg{800, 600, "Drag Comparison Demo", true, true};
    if (!BulletRender::app::Window::init(windowCfg))
    {
        return -1;
    }

    // renderer
    BulletRender::render::RenderConfig renderCfg{{0.05f, 0.05f, 0.08f, 1.0f}};
    BulletRender::render::Renderer::init(renderCfg);

    // grid
    auto grid = std::make_shared<BulletRender::render::Grid>();
    BulletRender::render::Renderer::registerPrePass(grid);

    // world coordinated
    auto worldAxis = std::make_shared<BulletRender::render::WorldAxis>();
    BulletRender::render::Renderer::registerPrePass(worldAxis);

    // trajectory lines
    auto lines = std::make_shared<BulletRender::render::Lines>(2.0f);
    BulletRender::render::Renderer::registerPrePass(lines);

    // scene
    BulletRender::scene::Scene scene;

    // camera
    BulletRender::scene::FlyCamera camera({0.0f, 1.5f, 5.0f});
    scene.setCamera(&camera);

    // light
    BulletRender::scene::DirectionalLight light;
    scene.setLight(&light);

    // fog
    auto fog = std::make_shared<BulletRender::render::Fog>(true, 10.0f, 90.0f);
    BulletRender::render::Renderer::registerPostPass(fog);

    // ecs
    ecs::World world;

    // systems
    RenderSystem renderSystem(scene);
    ImGuiSystem imguiSystem;
    ecs::systems::CollisionSystem collisionSystem;
    ecs::systems::TrajectorySystem trajectorySystem(lines);

    // integrator
    BulletPhysic::math::MidpointIntegrator integrator;

    // physics worlds
    // gravity only
    BulletPhysic::dynamics::PhysicsWorld gravityWorld;
    gravityWorld.addForce(std::make_unique<BulletPhysic::dynamics::forces::Gravity>());
    PhysicsSystem gravityPhysicsSystem(gravityWorld, integrator, &gravityProjectileId);

    // with drag
    BulletPhysic::dynamics::PhysicsWorld dragWorld;
    dragWorld.addForce(std::make_unique<BulletPhysic::dynamics::forces::Gravity>());
    dragWorld.addEnvironment(std::make_unique<BulletPhysic::dynamics::environment::Atmosphere>(280.0f, 100000.0f));
    dragWorld.addEnvironment(std::make_unique<BulletPhysic::dynamics::environment::Humidity>(60));
    dragWorld.addEnvironment(std::make_unique<BulletPhysic::dynamics::environment::Wind>(BulletPhysic::math::Vec3{0.0f, 0.0f, 2.0f}));
    dragWorld.addForce(std::make_unique<BulletPhysic::dynamics::forces::drag::Drag>());
    PhysicsSystem dragPhysicsSystem(dragWorld, integrator, &dragProjectileId);

    // ground collider
    auto groundObject = world.create();
    auto& groundCollider = world.add<ecs::ColliderComponent>(groundObject);
    groundCollider.collider = std::make_shared<BulletPhysic::collision::GroundCollider>(0.0f);

    // input
    ecs::systems::InputSystem inputSystem;
    inputSystem.bind(BulletRender::utils::InputKey::SPACE, [&world]() {
        launchBoth(world);
    });
    inputSystem.bind(BulletRender::utils::InputKey::ESCAPE, []() {
        BulletRender::app::Window::setShouldClose(true);
    });

    // imgui
    float lastDt = 0.0f;
    setupDebugDisplay(imguiSystem, camera, lastDt);
    setupProjectilesDisplay(imguiSystem, world);

    // loop
    BulletRender::app::Loop loop(scene);
    loop.run(
        [&](float dt) {
            lastDt = dt;

            camera.update(BulletRender::app::Window::get(), dt);

            BulletRender::utils::Input::instance().update(BulletRender::app::Window::get());

            gravityPhysicsSystem.update(world, dt);
            dragPhysicsSystem.update(world, dt);

            collisionSystem.update(world);
            trajectorySystem.update(world);

            renderSystem.render(world);
            imguiSystem.render();
        }
    );

    BulletRender::app::Window::shutdown();
    return 0;
}
