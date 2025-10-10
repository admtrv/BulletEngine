/*
 * main.cpp
 */

// luchrender
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

// luchphysic
#include "math/Integrator.h"

// luchengine
#include "ecs/Ecs.h"
#include "ecs/Components.h"
#include "systems/PhysicsSystem.h"
#include "systems/RenderSystem.h"

using namespace luchengine;

int main()
{
    // window
    luchrender::app::WindowConfig windowCfg{800, 600, "Demo", true};
    if (!luchrender::app::Window::init(windowCfg))
    {
        return -1;
    }

    // renderer
    luchrender::render::Renderer::init();

    // grid
    auto grid = std::make_shared<luchrender::render::Grid>();
    luchrender::render::Renderer::registerPrePass(grid);

    // world coordinated
    auto worldAxis = std::make_shared<luchrender::render::WorldAxis>();
    luchrender::render::Renderer::registerPrePass(worldAxis);

    // assets
    luchrender::scene::Model model("assets/models/bullet.obj");
    std::shared_ptr<luchrender::render::Shader> shader = std::make_shared<luchrender::render::Shader>(
        "assets/shaders/normal.vert.glsl",
        "assets/shaders/normal.frag.glsl"
    );

    // scene
    luchrender::scene::Scene scene;

    luchrender::scene::FlyCamera camera({0,1.5f,5.0f});
    scene.setCamera(&camera);

    luchrender::scene::DirectionalLight light;
    scene.setLight(&light);

    // ecs
    ecs::World world;
    ecs::Entity bullet = world.create();

    // components
    auto& transformComponent = world.add<ecs::TransformComponent>(bullet);

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

    luchphysic::math::EulerIntegrator euler;
    systems::PhysicsSystem physicsSystem(euler);

    // loop
    luchrender::app::Loop loop(scene);
    loop.run(
        [&](float dt) {
            camera.update(luchrender::app::Window::get(), dt);

            physicsSystem.update(world, dt);
            renderSystem.rebuild(world);
        }
    );

    luchrender::app::Window::shutdown();
    return 0;
}