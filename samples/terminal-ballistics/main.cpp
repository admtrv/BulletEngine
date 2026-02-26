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
#include "collision/terminal/Material.h"
#include "collision/terminal/ImpactResult.h"
#include "dynamics/environment/Atmosphere.h"
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
#include "common/objects/Projectile.h"

// local
#include "Components.h"
#include "CollisionSystem.h"
#include "TrajectorySystem.h"
#include "PhysicsSystem.h"

using namespace BulletEngine;

using RenderSystem = ecs::systems::RenderSystemBase;
using ImGuiSystem = ecs::systems::ImGuiSystemBase;

// projectile entity IDs
ecs::Entity penetrationProjectileId = 0;
ecs::Entity embedProjectileId = 0;
ecs::Entity multilayerProjectileId = 0;
ecs::Entity ricochetProjectileId = 0;

// z positions
constexpr float LANE_PENETRATION = 3.0f;
constexpr float LANE_EMBED = 1.0f;
constexpr float LANE_MULTILAYER = -1.0f;
constexpr float LANE_RICOCHET = -3.0f;

ecs::Entity launchProjectile(ecs::World& world, const BulletPhysics::math::Vec3& position, float speed, float elevation, float azimuth)
{
    ecs::Entity entity = objects::Projectile::launch(world, position, speed, elevation, azimuth);
    world.add<ecs::EnergyTrajectoryComponent>(entity);
    world.add<ecs::ImpactStateComponent>(entity);
    return entity;
}

void launchAll(ecs::World& world)
{
    penetrationProjectileId = launchProjectile(world, {0.0f, 1.5f, LANE_PENETRATION}, 50.0f, 0.0f, 90.0f);
    embedProjectileId = launchProjectile(world, {0.0f, 1.5f, LANE_EMBED}, 50.0f, 0.0f, 90.0f);
    multilayerProjectileId = launchProjectile(world, {0.0f, 1.5f, LANE_MULTILAYER}, 50.0f, 0.0f, 90.0f);
    ricochetProjectileId = launchProjectile(world, {0.0f, 1.5f, LANE_RICOCHET - 0.5f}, 15.0f, 0.0f, 85.0f);
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

void showProjectileInfo(const char* label, ecs::World& world, ecs::Entity id)
{
    ImGui::Text("%s", label);

    if (id != 0 && world.has<ecs::ProjectileRigidBodyComponent>(id))
    {
        auto* rb = world.get<ecs::ProjectileRigidBodyComponent>(id);
        if (rb)
        {
            auto pos = rb->getProjectileBody().getPosition();
            ImGui::Text("   Position: (%.3f, %.3f, %.3f)", pos.x, pos.y, pos.z);

            auto vel = rb->getProjectileBody().getVelocity();
            float speed = vel.length();
            ImGui::Text("   Speed: %.2f m/s", speed);

            float ke = 0.5f * rb->getProjectileBody().getMass() * speed * speed;
            ImGui::Text("   Kinetic Energy: %.4f J", ke);

            auto* impactState = world.get<ecs::ImpactStateComponent>(id);
            bool hasImpacted = impactState && impactState->hasImpacted;

            if (hasImpacted)
            {
                if (rb->isGrounded)
                {
                    ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "   Embed");
                }
                else if (speed > 0.1f)
                {
                    ImGui::TextColored(ImVec4(0.2f, 1.0f, 0.2f, 1.0f), "   Penetration");
                    ImGui::Text("   Residual Speed: %.2f m/s", speed);
                }
                else
                {
                    ImGui::TextColored(ImVec4(0.2f, 0.4f, 1.0f, 1.0f), "   Ricochet");
                }
            }
            else
            {
                ImGui::Text("   In flight");
            }
        }
    }
    else
    {
        ImGui::Text("   Not launched");
    }
}

void setupProjectileDisplay(ImGuiSystem& imgui, ecs::World& world)
{
    imgui.add([&world]() {
        ImGui::Begin("Terminal Ballistics");

        showProjectileInfo("Penetration", world, penetrationProjectileId);
        ImGui::Separator();
        showProjectileInfo("Embed", world, embedProjectileId);
        ImGui::Separator();
        showProjectileInfo("Multi-layer Penetration", world, multilayerProjectileId);
        ImGui::Separator();
        showProjectileInfo("Ricochet", world, ricochetProjectileId);

        ImGui::End();
    });
}

// helper: create a wall entity
void createWall(ecs::World& world, const BulletPhysics::math::Vec3& position, const BulletPhysics::math::Vec3& size, const BulletPhysics::collision::terminal::Material& material, const BulletPhysics::math::Vec3& color, std::shared_ptr<BulletRender::render::Shader> shader)
{
    ecs::Entity entity = world.create();

    auto& transform = world.add<ecs::TransformComponent>(entity);
    transform.transform.setPosition({position.x, position.y, position.z});

    auto& renderable = world.add<ecs::RenderableComponent>(entity);
    renderable.model = new BulletRender::scene::Box(size.x, size.y, size.z);
    renderable.material.setShader(shader);
    renderable.material.setColor({color.x, color.y, color.z});

    auto& colliderComp = world.add<ecs::ColliderComponent>(entity);
    auto boxCollider = std::make_shared<BulletPhysics::collision::BoxCollider>(size);
    boxCollider->setPosition(position);
    boxCollider->setMaterial(material);
    colliderComp.collider = boxCollider;
}

int main()
{
    // window
    BulletRender::app::WindowConfig windowCfg{800, 600, "Terminal Ballistics Demo", true, true};
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

    // world coordinates
    auto worldAxis = std::make_shared<BulletRender::render::WorldAxis>();
    BulletRender::render::Renderer::registerPrePass(worldAxis);

    // trajectory lines
    auto lines = std::make_shared<BulletRender::render::Lines>(2.0f);
    BulletRender::render::Renderer::registerPrePass(lines);

    // scene
    BulletRender::scene::Scene scene;

    // camera
    BulletRender::scene::FlyCamera camera({3.0f, 3.0f, 8.0f});
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
    ecs::systems::TerminalCollisionSystem collisionSystem;
    ecs::systems::EnergyTrajectorySystem trajectorySystem(lines);

    // physics
    BulletPhysics::dynamics::PhysicsWorld physicsWorld;
    BulletPhysics::math::RK4Integrator integrator;
    ecs::systems::PhysicsSystem physicsSystem(physicsWorld, integrator);

    // configure physics world
    physicsWorld.addForce(std::make_unique<BulletPhysics::dynamics::forces::Gravity>());
    physicsWorld.addEnvironment(std::make_unique<BulletPhysics::dynamics::environment::Atmosphere>(280.0f, 100000.0f));
    physicsWorld.addEnvironment(std::make_unique<BulletPhysics::dynamics::environment::Humidity>(60));
    physicsWorld.addForce(std::make_unique<BulletPhysics::dynamics::forces::drag::Drag>());

    // ground collider with Soil material
    auto groundObject = world.create();
    auto& groundCollider = world.add<ecs::ColliderComponent>(groundObject);
    auto ground = std::make_shared<BulletPhysics::collision::GroundCollider>(0.0f);
    ground->setMaterial(BulletPhysics::collision::terminal::materials::Soil());
    groundCollider.collider = ground;

    // shared shader for walls
    auto wallShader = std::make_shared<BulletRender::render::Shader>(
        "assets/shaders/normal.vert.glsl",
        "assets/shaders/normal.frag.glsl");

    BulletPhysics::math::Vec3 woodColor{0.6f, 0.4f, 0.2f};
    BulletPhysics::math::Vec3 concreteColor{0.7f, 0.7f, 0.7f};
    BulletPhysics::math::Vec3 steelColor{0.75f, 0.75f, 0.8f};

    // wall dimensions
    float wallHeight = 3.0f;
    float wallWidth = 1.5f;
    float wallThickness = 0.05f;

    // lane 1: single wood wall
    {
        auto mat = BulletPhysics::collision::terminal::materials::Wood();
        createWall(world,{5.0f, 1.5f, LANE_PENETRATION},{wallThickness, wallHeight, wallWidth}, mat, woodColor, wallShader);
    }

    // lane 2: concrete wall
    {
        auto mat = BulletPhysics::collision::terminal::materials::Concrete();
        createWall(world,{5.0f, 1.5f, LANE_EMBED},{wallThickness, wallHeight, wallWidth}, mat, concreteColor, wallShader);
    }

    // lane 3: wood walls in a row
    {
        auto mat = BulletPhysics::collision::terminal::materials::Wood();
        float gap = 1.0f;

        for (int i = 0; i < 4; ++i)
        {
            float x = 5.0f + static_cast<float>(i) * (wallThickness + gap);
            createWall(world,{x, 1.5f, LANE_MULTILAYER},{wallThickness, wallHeight, wallWidth}, mat, woodColor, wallShader);
        }
    }

    // lane 4: steel wall
    {
        auto mat = BulletPhysics::collision::terminal::materials::Steel();
        createWall(world,{5.0f, 1.5f, LANE_RICOCHET},{wallWidth, wallHeight, wallThickness}, mat, steelColor, wallShader);
    }

    // input
    ecs::systems::InputSystem inputSystem;
    inputSystem.bind(BulletRender::utils::InputKey::SPACE, [&world]() {
        launchAll(world);
    });
    inputSystem.bind(BulletRender::utils::InputKey::ESCAPE, []() {
        BulletRender::app::Window::setShouldClose(true);
    });

    // imgui
    ImGuiSystem imguiSystem;
    float lastDt = 0.0f;
    setupDebugDisplay(imguiSystem, camera, lastDt);
    setupProjectileDisplay(imguiSystem, world);

    // loop
    BulletRender::app::Loop loop(scene);

    // fixed physics timestep
    constexpr float PHYSICS_DT = 1.0f / 1000.0f;
    constexpr int MAX_STEPS_PER_FRAME = 50;         // safety cap
    float physicsAccumulator = 0.0f;

    loop.run(
        [&](float dt) {
            lastDt = dt;

            camera.update(BulletRender::app::Window::get(), dt);

            BulletRender::utils::Input::instance().update(BulletRender::app::Window::get());

            // accumulate frame time, step physics at fixed rate
            physicsAccumulator += dt;
            int steps = 0;
            while (physicsAccumulator >= PHYSICS_DT && steps < MAX_STEPS_PER_FRAME)
            {
                physicsSystem.update(world, PHYSICS_DT);
                collisionSystem.update(world);
                trajectorySystem.update(world);
                physicsAccumulator -= PHYSICS_DT;
                ++steps;
            }

            // discard excess to prevent accumulator runaway
            if (physicsAccumulator > PHYSICS_DT)
            {
                physicsAccumulator = 0.0f;
            }

            renderSystem.render(world);
            imguiSystem.render();
        }
    );

    BulletRender::app::Window::shutdown();
    return 0;
}
