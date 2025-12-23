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

// BulletPhysics
#include "math/Integrator.h"
#include "math/Angles.h"
#include "collision/BoxCollider.h"
#include "collision/GroundCollider.h"
#include "dynamics/environment/Atmosphere.h"
#include "dynamics/environment/Geographic.h"
#include "dynamics/environment/Humidity.h"
#include "dynamics/forces/Coriolis.h"
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

void setupProjectileDisplay(ImGuiSystem& imgui, ecs::World& world, BulletPhysics::dynamics::PhysicsWorld& physicsWorld)
{
    imgui.add([&world, &physicsWorld]() {
        ImGui::Begin("Projectile");

        // find last projectile
        ecs::Entity lastProjectile = 0;
        for (const auto& entity : world.entities())
        {
            if (world.has<ecs::ProjectileRigidBodyComponent>(entity) && world.has<ecs::TransformComponent>(entity))
            {
                lastProjectile = entity;
            }
        }

        if (lastProjectile != 0)
        {
            auto* transform = world.get<ecs::TransformComponent>(lastProjectile);
            auto pos = transform->transform.getPosition();

            ImGui::Text("Position:");
            ImGui::Text("   X: %.3f", pos.x);
            ImGui::Text("   Y: %.3f", pos.y);
            ImGui::Text("   Z: %.3f", pos.z);

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
    });
}

int main()
{
    // window
    BulletRender::app::WindowConfig windowCfg{800, 600, "Projectile Basic Demo", true, true};
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
    ecs::systems::CollisionSystem collisionSystem;
    ecs::systems::TrajectorySystem trajectorySystem(lines);

    // physics
    BulletPhysics::dynamics::PhysicsWorld physicsWorld;
    BulletPhysics::math::MidpointIntegrator integrator;
    ecs::systems::PhysicsSystem physicsSystem(physicsWorld, integrator);

    // configure physics world
    physicsWorld.addForce(std::make_unique<BulletPhysics::dynamics::forces::Gravity>());
    physicsWorld.addEnvironment(std::make_unique<BulletPhysics::dynamics::environment::Atmosphere>(280.0f, 100000.0f));                      // t_0 = 280 K, p_0 = 100.000 Pa
    physicsWorld.addEnvironment(std::make_unique<BulletPhysics::dynamics::environment::Humidity>(60));                                       // relative humidity = 60%
    physicsWorld.addEnvironment(std::make_unique<BulletPhysics::dynamics::environment::Wind>(BulletPhysics::math::Vec3{0.0f, 0.0f, 2.0f}));   // wind velocity = 2 m/s
    physicsWorld.addEnvironment(std::make_unique<BulletPhysics::dynamics::environment::Geographic>(BulletPhysics::math::deg2rad(48.1482), BulletPhysics::math::deg2rad(17.1067))); // Bratislava coordinates
    physicsWorld.addForce(std::make_unique<BulletPhysics::dynamics::forces::drag::Drag>());
    physicsWorld.addForce(std::make_unique<BulletPhysics::dynamics::forces::Coriolis>());

    // ground collider
    auto groundObject = world.create();
    auto& groundCollider = world.add<ecs::ColliderComponent>(groundObject);
    groundCollider.collider = std::make_shared<BulletPhysics::collision::GroundCollider>(0.0f);

    // input
    ecs::systems::InputSystem inputSystem;
    inputSystem.bind(BulletRender::utils::InputKey::SPACE, [&world]() {
        objects::Projectile::launch(world);
    });
    inputSystem.bind(BulletRender::utils::InputKey::ESCAPE, []() {
        BulletRender::app::Window::setShouldClose(true);
    });

    // imgui
    ImGuiSystem imguiSystem;
    float lastDt = 0.0f;
    setupDebugDisplay(imguiSystem, camera, lastDt);
    setupProjectileDisplay(imguiSystem, world, physicsWorld);

    // loop
    BulletRender::app::Loop loop(scene);
    loop.run(
        [&](float dt) {
            lastDt = dt;

            camera.update(BulletRender::app::Window::get(), dt);

            BulletRender::utils::Input::instance().update(BulletRender::app::Window::get());

            physicsSystem.update(world, dt);
            collisionSystem.update(world);
            trajectorySystem.update(world);

            renderSystem.render(world);
            imguiSystem.render();
        }
    );

    BulletRender::app::Window::shutdown();
    return 0;
}
