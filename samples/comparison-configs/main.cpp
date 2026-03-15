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
#include "scene/Scene.h"
#include "scene/Camera.h"
#include "scene/Light.h"
#include "imgui.h"

// BulletPhysics
#include "math/Integrator.h"
#include "math/Angles.h"
#include "builtin/collision/collider/GroundCollider.h"
#include "ballistics/external/environments/Atmosphere.h"
#include "ballistics/external/environments/Geographic.h"
#include "ballistics/external/environments/Humidity.h"
#include "ballistics/external/environments/Wind.h"
#include "ballistics/external/forces/Gravity.h"
#include "ballistics/external/forces/Coriolis.h"
#include "ballistics/external/forces/SpinDrift.h"
#include "ballistics/external/forces/drag/Drag.h"

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

// configuration
static constexpr float LAUNCH_SPEED = 750.0f;
static constexpr double TEMPERATURE = 280.0;                        // K
static constexpr double PRESSURE = 100000.0;                        // Pa
static constexpr double REL_HUMIDITY = 60.0;                        // %
static constexpr double LATITUDE = 48.1482;                         // deg
static constexpr double LONGITUDE = 17.1067;                        // deg
static const BulletPhysics::math::Vec3 WIND = {0.0, 0.0, -10.0};    // m/s

// trajectory colors
static const BulletPhysics::math::Vec3 COLOR_GRAVITY  {1.0f, 1.0f, 1.0f};
static const BulletPhysics::math::Vec3 COLOR_DRAG     {0.2f, 0.6f, 1.0f};
static const BulletPhysics::math::Vec3 COLOR_CORIOLIS {0.2f, 1.0f, 0.4f};
static const BulletPhysics::math::Vec3 COLOR_SPIN     {1.0f, 0.4f, 0.2f};
static const BulletPhysics::math::Vec3 COLOR_WIND     {0.8f, 0.2f, 1.0f};

// flight statistics
struct FlightStats
{
    float range = 0.0f;
    float drift = 0.0f;
    float flightTime = 0.0f;
    bool landed = false;
};

struct Config
{
    const char* name;
    ecs::Entity entityId;
    FlightStats stats;
    BulletPhysics::math::Vec3 color;
};

static Config configs[] = {
    {"gravity", 0, {}, COLOR_GRAVITY},
    {"+ drag", 0, {}, COLOR_DRAG},
    {"+ coriolis", 0, {}, COLOR_CORIOLIS},
    {"+ spin", 0, {}, COLOR_SPIN},
    {"+ wind", 0, {}, COLOR_WIND},
};

static float elapsed = 0.0f;

static void collectStats(Config& config, const BulletPhysics::math::Vec3& pos, bool grounded)
{
    if (config.stats.landed)
        return;

    config.stats.flightTime = elapsed;

    if (grounded)
    {
        config.stats.range = std::sqrt(pos.x * pos.x + pos.z * pos.z);
        config.stats.drift = pos.z;
        config.stats.landed = true;

        std::cout << std::fixed << std::setprecision(4);
        std::cout << config.name << ": " << "range " << config.stats.range << " m, " << "drift " << config.stats.drift << " m, " << "time " << config.stats.flightTime << " s\n";
    }
}

static void launchAll(ecs::World& world)
{
    elapsed = 0.0f;

    for (auto& config : configs)
    {
        config.stats = {};
        auto specs = BulletPhysics::projectile::ProjectileSpecs::create(0.01, 0.00762)
            .withDragModel(BulletPhysics::ballistics::external::forces::drag::DragCurveModel::G7)
            .withMuzzle(LAUNCH_SPEED, BulletPhysics::projectile::Direction::RIGHT, 12.0);
        config.entityId = objects::Projectile::launch(world, specs, {0.0, 1.5, 0.0}, 0.0, 90.0);
        world.get<ecs::TrajectoryComponent>(config.entityId)->color = config.color;
    }
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
    // window
    BulletRender::app::WindowConfig windowCfg{800, 600, "Config Comparison", true, true};
    if (!BulletRender::app::Window::init(windowCfg))
        return -1;

    // renderer
    BulletRender::render::RenderConfig renderCfg{{0.05f, 0.05f, 0.08f, 1.0f}};
    BulletRender::render::Renderer::init(renderCfg);

    // grid
    auto grid = std::make_shared<BulletRender::render::Grid>();
    BulletRender::render::Renderer::registerPrePass(grid);

    // world coordinates
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
    auto fog = std::make_shared<BulletRender::render::Fog>(true, 10.0f, 900.0f);
    BulletRender::render::Renderer::registerPostPass(fog);

    // ecs
    ecs::World world;

    // systems
    RenderSystem renderSystem(scene);
    ImGuiSystem imguiSystem;
    ecs::systems::CollisionSystem collisionSystem;
    ecs::systems::TrajectorySystem trajectorySystem(lines);

    BulletPhysics::math::RK4Integrator integrator;

    // gravity only
    BulletPhysics::ballistics::external::PhysicsWorld gravityWorld;
    gravityWorld.addForce(std::make_unique<BulletPhysics::ballistics::external::forces::Gravity>());
    PhysicsSystem gravityPhysics(gravityWorld, integrator, &configs[0].entityId);

    // + drag
    BulletPhysics::ballistics::external::PhysicsWorld dragWorld;
    dragWorld.addForce(std::make_unique<BulletPhysics::ballistics::external::forces::Gravity>());
    dragWorld.addEnvironment(std::make_unique<BulletPhysics::ballistics::external::environments::Atmosphere>(TEMPERATURE, PRESSURE));
    dragWorld.addEnvironment(std::make_unique<BulletPhysics::ballistics::external::environments::Humidity>(REL_HUMIDITY));
    dragWorld.addForce(std::make_unique<BulletPhysics::ballistics::external::forces::Drag>());
    PhysicsSystem dragPhysics(dragWorld, integrator, &configs[1].entityId);

    // + coriolis
    BulletPhysics::ballistics::external::PhysicsWorld coriolisWorld;
    coriolisWorld.addForce(std::make_unique<BulletPhysics::ballistics::external::forces::Gravity>());
    coriolisWorld.addEnvironment(std::make_unique<BulletPhysics::ballistics::external::environments::Atmosphere>(TEMPERATURE, PRESSURE));
    coriolisWorld.addEnvironment(std::make_unique<BulletPhysics::ballistics::external::environments::Humidity>(REL_HUMIDITY));
    coriolisWorld.addEnvironment(std::make_unique<BulletPhysics::ballistics::external::environments::Geographic>(BulletPhysics::math::deg2rad(LATITUDE), BulletPhysics::math::deg2rad(LONGITUDE)));
    coriolisWorld.addForce(std::make_unique<BulletPhysics::ballistics::external::forces::Drag>());
    coriolisWorld.addForce(std::make_unique<BulletPhysics::ballistics::external::forces::Coriolis>());
    PhysicsSystem coriolisPhysics(coriolisWorld, integrator, &configs[2].entityId);

    // + spin
    BulletPhysics::ballistics::external::PhysicsWorld spinWorld;
    spinWorld.addForce(std::make_unique<BulletPhysics::ballistics::external::forces::Gravity>());
    spinWorld.addEnvironment(std::make_unique<BulletPhysics::ballistics::external::environments::Atmosphere>(TEMPERATURE, PRESSURE));
    spinWorld.addEnvironment(std::make_unique<BulletPhysics::ballistics::external::environments::Humidity>(REL_HUMIDITY));
    spinWorld.addEnvironment(std::make_unique<BulletPhysics::ballistics::external::environments::Geographic>(BulletPhysics::math::deg2rad(LATITUDE), BulletPhysics::math::deg2rad(LONGITUDE)));
    spinWorld.addForce(std::make_unique<BulletPhysics::ballistics::external::forces::Drag>());
    spinWorld.addForce(std::make_unique<BulletPhysics::ballistics::external::forces::Coriolis>());
    BulletPhysics::ballistics::external::forces::SpinDrift::addTo(spinWorld);
    PhysicsSystem spinPhysics(spinWorld, integrator, &configs[3].entityId);

    // + wind
    BulletPhysics::ballistics::external::PhysicsWorld windWorld;
    windWorld.addForce(std::make_unique<BulletPhysics::ballistics::external::forces::Gravity>());
    windWorld.addEnvironment(std::make_unique<BulletPhysics::ballistics::external::environments::Atmosphere>(TEMPERATURE, PRESSURE));
    windWorld.addEnvironment(std::make_unique<BulletPhysics::ballistics::external::environments::Humidity>(REL_HUMIDITY));
    windWorld.addEnvironment(std::make_unique<BulletPhysics::ballistics::external::environments::Geographic>(BulletPhysics::math::deg2rad(LATITUDE), BulletPhysics::math::deg2rad(LONGITUDE)));
    windWorld.addEnvironment(std::make_unique<BulletPhysics::ballistics::external::environments::Wind>(WIND));
    windWorld.addForce(std::make_unique<BulletPhysics::ballistics::external::forces::Drag>());
    windWorld.addForce(std::make_unique<BulletPhysics::ballistics::external::forces::Coriolis>());
    BulletPhysics::ballistics::external::forces::SpinDrift::addTo(windWorld);
    PhysicsSystem windPhysics(windWorld, integrator, &configs[4].entityId);

    PhysicsSystem* physicsSystems[] = {&gravityPhysics, &dragPhysics, &coriolisPhysics, &spinPhysics, &windPhysics};

    // ground
    auto groundObject = world.create();
    world.add<ecs::ColliderComponent>(groundObject).collider =
        std::make_shared<BulletPhysics::builtin::collision::collider::GroundCollider>(0.0f);

    // input
    ecs::systems::InputSystem inputSystem;
    inputSystem.bind(BulletRender::utils::InputKey::SPACE, [&world]() {
        launchAll(world);
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

        for (auto* physics : physicsSystems)
            physics->update(world, dt);

        for (auto& config : configs)
        {
            if (config.entityId == 0 || !world.has<ecs::ProjectileRigidBodyComponent>(config.entityId))
                continue;

            auto* rb = world.get<ecs::ProjectileRigidBodyComponent>(config.entityId);
            if (rb)
                collectStats(config, rb->getProjectileBody().getPosition(), rb->isGrounded);
        }

        collisionSystem.update(world);
        trajectorySystem.update(world);

        renderSystem.render(world);
        imguiSystem.render();
    });

    BulletRender::app::Window::shutdown();
    return 0;
}
