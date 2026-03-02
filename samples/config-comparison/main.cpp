/*
 * main.cpp
 */

// std
#include <iostream>
#include <iomanip>

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
#include "dynamics/forces/drag/Drag.h"
#include "dynamics/forces/Gravity.h"
#include "dynamics/forces/Coriolis.h"
#include "dynamics/forces/SpinDrift.h"
#include "dynamics/environment/Wind.h"

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
using ImGuiSystem  = ecs::systems::ImGuiSystemBase;

static constexpr float LAUNCH_SPEED = 750.0f;

const BulletPhysics::math::Vec3 COLOR_IDEAL {1.0f, 1.0f, 1.0f};
const BulletPhysics::math::Vec3 COLOR_DRAG {0.2f, 0.6f, 1.0f};
const BulletPhysics::math::Vec3 COLOR_CORIOLIS {0.2f, 1.0f, 0.4f};
const BulletPhysics::math::Vec3 COLOR_SPIN {1.0f, 0.4f, 0.2f};
const BulletPhysics::math::Vec3 COLOR_WIND {0.8f, 0.2f, 1.0f};

ecs::Entity idealId = 0;
ecs::Entity dragId = 0;
ecs::Entity coriolisId = 0;
ecs::Entity spinId = 0;
ecs::Entity windId = 0;

struct FlightStats
{
    float range = 0.0f;
    float drift = 0.0f;
    float flightTime = 0.0f;
    bool  landed = false;
};

FlightStats idealStats;
FlightStats dragStats;
FlightStats coriolisStats;
FlightStats spinStats;
FlightStats windStats;

void collectStats(FlightStats& stats, const char* label, const BulletPhysics::math::Vec3& pos, float elapsed, bool grounded)
{
    if (stats.landed)
        return;

    stats.flightTime = elapsed;

    if (grounded)
    {
        stats.range  = std::sqrt(pos.x * pos.x + pos.z * pos.z);
        stats.drift  = pos.z;
        stats.landed = true;

        std::cout << std::fixed << std::setprecision(4);
        std::cout << "[" << label << "]\n";
        std::cout << "  range      : " << stats.range      << " m\n";
        std::cout << "  drift (z)  : " << stats.drift      << " m\n";
        std::cout << "  flight time: " << stats.flightTime << " s\n";
        std::cout << "\n";
    }
}

void launchAll(ecs::World& world, float& elapsed)
{
    idealStats = {};
    dragStats = {};
    coriolisStats = {};
    spinStats = {};
    windStats = {};
    elapsed = 0.0f;

    idealId = objects::Projectile::launch(world, {0.0f, 1.5f, 0.0f}, LAUNCH_SPEED, 0.0f, 90.0f);
    dragId = objects::Projectile::launch(world, {0.0f, 1.5f, 0.0f}, LAUNCH_SPEED, 0.0f, 90.0f);
    coriolisId = objects::Projectile::launch(world, {0.0f, 1.5f, 0.0f}, LAUNCH_SPEED, 0.0f, 90.0f);
    spinId = objects::Projectile::launch(world, {0.0f, 1.5f, 0.0f}, LAUNCH_SPEED, 0.0f, 90.0f);
    windId = objects::Projectile::launch(world, {0.0f, 1.5f, 0.0f}, LAUNCH_SPEED, 0.0f, 90.0f);

    world.get<ecs::TrajectoryComponent>(idealId)->color = COLOR_IDEAL;
    world.get<ecs::TrajectoryComponent>(dragId)->color = COLOR_DRAG;
    world.get<ecs::TrajectoryComponent>(coriolisId)->color = COLOR_CORIOLIS;
    world.get<ecs::TrajectoryComponent>(spinId)->color = COLOR_SPIN;
    world.get<ecs::TrajectoryComponent>(windId)->color = COLOR_WIND;
}

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


int main()
{
    BulletRender::app::WindowConfig windowCfg{800, 600, "Config Comparison Demo", true, true};
    if (!BulletRender::app::Window::init(windowCfg))
        return -1;

    BulletRender::render::RenderConfig renderCfg{{0.05f, 0.05f, 0.08f, 1.0f}};
    BulletRender::render::Renderer::init(renderCfg);

    auto grid = std::make_shared<BulletRender::render::Grid>();
    auto worldAxis = std::make_shared<BulletRender::render::WorldAxis>();
    auto lines = std::make_shared<BulletRender::render::Lines>(2.0f);
    auto fog = std::make_shared<BulletRender::render::Fog>(true, 10.0f, 900.0f);

    BulletRender::render::Renderer::registerPrePass(grid);
    BulletRender::render::Renderer::registerPrePass(worldAxis);
    BulletRender::render::Renderer::registerPrePass(lines);
    BulletRender::render::Renderer::registerPostPass(fog);

    BulletRender::scene::Scene scene;
    BulletRender::scene::FlyCamera camera({0.0f, 1.5f, 5.0f});
    BulletRender::scene::DirectionalLight light;
    scene.setCamera(&camera);
    scene.setLight(&light);

    ecs::World world;

    RenderSystem renderSystem(scene);
    ImGuiSystem  imguiSystem;
    ecs::systems::CollisionSystem  collisionSystem;
    ecs::systems::TrajectorySystem trajectorySystem(lines);

    BulletPhysics::math::RK4Integrator integrator;

    // 1. g only
    BulletPhysics::dynamics::PhysicsWorld idealWorld;
    idealWorld.addForce(std::make_unique<BulletPhysics::dynamics::forces::Gravity>());
    PhysicsSystem idealPhysics(idealWorld, integrator, &idealId);

    // 2. g + drag
    BulletPhysics::dynamics::PhysicsWorld dragWorld;
    dragWorld.addForce(std::make_unique<BulletPhysics::dynamics::forces::Gravity>());
    dragWorld.addEnvironment(std::make_unique<BulletPhysics::dynamics::environment::Atmosphere>(280.0f, 100000.0f));                      // t_0 = 280 K, p_0 = 100.000 Pa
    dragWorld.addEnvironment(std::make_unique<BulletPhysics::dynamics::environment::Humidity>(60));                                       // relative humidity = 60%
    dragWorld.addForce(std::make_unique<BulletPhysics::dynamics::forces::drag::Drag>());
    PhysicsSystem dragPhysics(dragWorld, integrator, &dragId);

    // 3. g + drag + coriolis
    BulletPhysics::dynamics::PhysicsWorld coriolisWorld;
    coriolisWorld.addForce(std::make_unique<BulletPhysics::dynamics::forces::Gravity>());
    coriolisWorld.addEnvironment(std::make_unique<BulletPhysics::dynamics::environment::Atmosphere>(280.0f, 100000.0f));                      // t_0 = 280 K, p_0 = 100.000 Pa
    coriolisWorld.addEnvironment(std::make_unique<BulletPhysics::dynamics::environment::Humidity>(60));                                       // relative humidity = 60%
    coriolisWorld.addEnvironment(std::make_unique<BulletPhysics::dynamics::environment::Geographic>(BulletPhysics::math::deg2rad(48.1482), BulletPhysics::math::deg2rad(17.1067))); // Bratislava coordinates
    coriolisWorld.addForce(std::make_unique<BulletPhysics::dynamics::forces::drag::Drag>());
    coriolisWorld.addForce(std::make_unique<BulletPhysics::dynamics::forces::Coriolis>());
    PhysicsSystem coriolisPhysics(coriolisWorld, integrator, &coriolisId);

    // 4. g + drag + coriolis + spin
    BulletPhysics::dynamics::PhysicsWorld spinWorld;
    spinWorld.addForce(std::make_unique<BulletPhysics::dynamics::forces::Gravity>());
    spinWorld.addEnvironment(std::make_unique<BulletPhysics::dynamics::environment::Atmosphere>(280.0f, 100000.0f));                      // t_0 = 280 K, p_0 = 100.000 Pa
    spinWorld.addEnvironment(std::make_unique<BulletPhysics::dynamics::environment::Humidity>(60));                                       // relative humidity = 60%
    spinWorld.addEnvironment(std::make_unique<BulletPhysics::dynamics::environment::Geographic>(BulletPhysics::math::deg2rad(48.1482), BulletPhysics::math::deg2rad(17.1067))); // Bratislava coordinates
    spinWorld.addForce(std::make_unique<BulletPhysics::dynamics::forces::drag::Drag>());
    spinWorld.addForce(std::make_unique<BulletPhysics::dynamics::forces::Coriolis>());
    BulletPhysics::dynamics::forces::SpinDrift::addTo(spinWorld);
    PhysicsSystem spinPhysics(spinWorld, integrator, &spinId);

    // 5. g + drag + coriolis + spin + wind
    BulletPhysics::dynamics::PhysicsWorld windWorld;
    windWorld.addForce(std::make_unique<BulletPhysics::dynamics::forces::Gravity>());
    windWorld.addEnvironment(std::make_unique<BulletPhysics::dynamics::environment::Atmosphere>(280.0f, 100000.0f)); // t_0 = 280 K, p_0 = 100.000 Pa
    windWorld.addEnvironment(std::make_unique<BulletPhysics::dynamics::environment::Humidity>(60));                  // relative humidity = 60%
    windWorld.addEnvironment(std::make_unique<BulletPhysics::dynamics::environment::Geographic>(BulletPhysics::math::deg2rad(48.1482), BulletPhysics::math::deg2rad(17.1067))); // Bratislava coordinates
    windWorld.addEnvironment(std::make_unique<BulletPhysics::dynamics::environment::Wind>(BulletPhysics::math::Vec3{0.0f, 0.0f, 10.0f})); // 10 m/s crosswind
    windWorld.addForce(std::make_unique<BulletPhysics::dynamics::forces::drag::Drag>());
    windWorld.addForce(std::make_unique<BulletPhysics::dynamics::forces::Coriolis>());
    BulletPhysics::dynamics::forces::SpinDrift::addTo(windWorld);
    PhysicsSystem windPhysics(windWorld, integrator, &windId);

    // ground
    auto groundObject = world.create();
    world.add<ecs::ColliderComponent>(groundObject).collider =
        std::make_shared<BulletPhysics::collision::GroundCollider>(0.0f);

    // input
    float elapsed = 0.0f;
    ecs::systems::InputSystem inputSystem;
    inputSystem.bind(BulletRender::utils::InputKey::SPACE, [&]() {
        launchAll(world, elapsed);
    });
    inputSystem.bind(BulletRender::utils::InputKey::ESCAPE, []() {
        BulletRender::app::Window::setShouldClose(true);
    });

    // imgui
    float lastDt = 0.0f;
    setupDebugDisplay(imguiSystem, camera, lastDt);

    // loop
    BulletRender::app::Loop loop(scene);
    loop.run([&](float dt) {
        lastDt  = dt;
        elapsed += dt;

        camera.update(BulletRender::app::Window::get(), dt);
        BulletRender::utils::Input::instance().update(BulletRender::app::Window::get());

        idealPhysics.update(world, dt);
        dragPhysics.update(world, dt);
        coriolisPhysics.update(world, dt);
        spinPhysics.update(world, dt);
        windPhysics.update(world, dt);

        auto gather = [&](ecs::Entity id, FlightStats& stats, const char* label) {
            if (id == 0 || !world.has<ecs::ProjectileRigidBodyComponent>(id))
                return;

            auto* rb = world.get<ecs::ProjectileRigidBodyComponent>(id);

            if (!rb)
                return;

            collectStats(stats, label, rb->getProjectileBody().getPosition(), elapsed, rb->isGrounded);
        };

        gather(idealId, idealStats, "G only");
        gather(dragId, dragStats, "G + Drag");
        gather(coriolisId, coriolisStats, "G + Drag + Coriolis");
        gather(spinId, spinStats, "G + Drag + Coriolis + Spin");
        gather(windId, windStats, "G + Drag + Coriolis + Spin + Wind");

        collisionSystem.update(world);
        trajectorySystem.update(world);

        renderSystem.render(world);
        imguiSystem.render();
    });

    BulletRender::app::Window::shutdown();
    return 0;
}
