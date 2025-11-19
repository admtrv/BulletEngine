/*
 * main.cpp
 */

// BulletRender
#include "app/Window.h"
#include "app/Loop.h"
#include "render/passes/Grid.h"
#include "render/passes/WorldAxis.h"
#include "render/Renderer.h"
#include "render/Shader.h"
#include "scene/Scene.h"
#include "scene/Model.h"
#include "scene/Camera.h"
#include "scene/Light.h"

// BulletPhysic
#include "math/Integrator.h"
#include "math/Angles.h"
#include "collision/BoxCollider.h"
#include "collision/GroundCollider.h"
#include "PresetManager.h"

// BulletEngine
#include "ecs/Ecs.h"
#include "ecs/Components.h"
#include "ecs/systems/PhysicsSystem.h"
#include "ecs/systems/RenderSystem.h"
#include "ecs/systems/TrajectorySystem.h"
#include "ecs/systems/InputSystem.h"
#include "ecs/systems/CollisionSystem.h"
#include "objects/Projectile.h"

using namespace BulletEngine;

int main()
{
    // window
    BulletRender::app::WindowConfig windowCfg{800, 600, "Demo", true};
    if (!BulletRender::app::Window::init(windowCfg))
    {
        return -1;
    }

    // renderer
    BulletRender::render::Renderer::init();

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

    // ecs
    ecs::World world;

    // systems
    ecs::systems::RenderSystem renderSystem(scene);
    ecs::systems::CollisionSystem collisionSystem;
    ecs::systems::TrajectorySystem trajectorySystem(lines);

    // physics
    BulletPhysic::dynamics::PhysicsWorld physicsWorld;
    BulletPhysic::math::EulerIntegrator euler;
    ecs::systems::PhysicsSystem physicsSystem(physicsWorld, euler);

    // configure physics world
    BulletPhysic::preset::PresetManager::configure(physicsWorld, BulletPhysic::preset::Preset::CUSTOM);
    physicsWorld.addForce(std::make_unique<BulletPhysic::dynamics::forces::Gravity>());
    physicsWorld.addEnvironment(std::make_unique<BulletPhysic::dynamics::environment::Atmosphere>(280.0f, 100000.0f));   // t_0 = 280 K, p_0 = 100.000 Pa
    physicsWorld.addEnvironment(std::make_unique<BulletPhysic::dynamics::environment::Humidity>(60));                                             // relative humidity = 60%
    physicsWorld.addEnvironment(std::make_unique<BulletPhysic::dynamics::environment::Wind>(BulletPhysic::math::Vec3{0.0f, 0.0f, 2.0f}));   // wind velocity = 2 m/s
    physicsWorld.addEnvironment(std::make_unique<BulletPhysic::dynamics::environment::Geographic>(BulletPhysic::math::deg2rad(48.1482), BulletPhysic::math::deg2rad(17.1067))); // Bratislava coordinates
    physicsWorld.addForce(std::make_unique<BulletPhysic::dynamics::forces::Drag>());
    physicsWorld.addForce(std::make_unique<BulletPhysic::dynamics::forces::Coriolis>());

    // ground collider
    auto groundObject = world.create();
    auto& groundCollider = world.add<ecs::ColliderComponent>(groundObject);
    groundCollider.collider = std::make_shared<BulletPhysic::collision::GroundCollider>(0.0f);

    // input
    ecs::systems::InputSystem inputSystem;
    inputSystem.bind(BulletRender::utils::InputKey::SPACE, [&world]() {
        objects::Projectile::launch(world);
    });
    inputSystem.bind(BulletRender::utils::InputKey::ESCAPE, []() {
        BulletRender::app::Window::setShouldClose(true);
    });

    // loop
    BulletRender::app::Loop loop(scene);
    loop.run(
        [&](float dt) {
            camera.update(BulletRender::app::Window::get(), dt);

            BulletRender::utils::Input::instance().update(BulletRender::app::Window::get());

            physicsSystem.update(world, dt);
            collisionSystem.update(world);
            trajectorySystem.update(world);
            renderSystem.rebuild(world);
        }
    );

    BulletRender::app::Window::shutdown();
    return 0;
}