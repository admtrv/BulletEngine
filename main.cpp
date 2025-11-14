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

// BulletEngine
#include "ecs/Ecs.h"
#include "ecs/Components.h"
#include "systems/BallisticSystem.h"
#include "systems/RenderSystem.h"
#include "systems/TrajectorySystem.h"

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

    // assets
    BulletRender::scene::Model model("assets/models/bullet.obj");
    std::shared_ptr<BulletRender::render::Shader> shader = std::make_shared<BulletRender::render::Shader>(
        "assets/shaders/normal.vert.glsl",
        "assets/shaders/normal.frag.glsl"
    );

    // scene
    BulletRender::scene::Scene scene;

    BulletRender::scene::FlyCamera camera({0,1.5f,5.0f});
    scene.setCamera(&camera);

    BulletRender::scene::DirectionalLight light;
    scene.setLight(&light);

    // ecs
    ecs::World world;
    ecs::Entity bullet = world.create();

    // components
    auto& transformComponent = world.add<ecs::TransformComponent>(bullet);

    auto& trajectoryComponent = world.add<ecs::TrajectoryComponent>(bullet);

    auto& renderableComponent = world.add<ecs::RenderableComponent>(bullet);
    renderableComponent.model = &model;
    renderableComponent.material.setShader(shader);
    renderableComponent.material.setColor({1.0f, 0.5f, 0.0f});

    // rigid body
    auto& rigidBodyComponent = world.add<ecs::RigidBodyComponent>(bullet);
    rigidBodyComponent.body.setMass(0.05f);
    rigidBodyComponent.body.setPosition({0.0f, 1.5f, 0.0f});
    rigidBodyComponent.body.setVelocityFromAngles(10.0f, 45.0f, 90.0f);

    // connection layers
    systems::RenderSystem renderSystem(scene);

    systems::TrajectorySystem trajectorySystem(lines);

    BulletPhysic::math::EulerIntegrator euler;
    systems::BallisticSystem ballisticSystem(euler);
    ballisticSystem.setRealismLevel(BulletPhysic::preset::RealismLevel::WIND);
    ballisticSystem.setWindVelocity({0.0f, 0.0f, 2.0f});

    // loop
    BulletRender::app::Loop loop(scene);
    loop.run(
        [&](float dt) {
            camera.update(BulletRender::app::Window::get(), dt);

            ballisticSystem.update(world, dt);
            trajectorySystem.update(world);
            renderSystem.rebuild(world);
        }
    );

    BulletRender::app::Window::shutdown();
    return 0;
}