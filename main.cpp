/*
 * main.cpp
 */

// luchrender
#include "app/Window.h"
#include "app/Loop.h"
#include "render/Renderer.h"
#include "render/Shader.h"
#include "scene/Scene.h"
#include "scene/Model.h"
#include "scene/Camera.h"
#include "scene/Light.h"

// luchengine
#include "ecs/Ecs.h"
#include "ecs/Components.h"
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

    // connection layer luchrender <-> luchengine
    systems::RenderSystem renderSystem(scene);

    // loop
    luchrender::app::Loop loop(scene);
    loop.run(
        [&](float dt) {
            camera.update(luchrender::app::Window::get(), dt);

            // if (auto* t = world.get<luchengine::ecs::TransformComponent>(entity))
            // {
                // t->transform.rotateZ(0.5f * dt);
            // }

            renderSystem.rebuild(world);
        }
    );

    luchrender::app::Window::shutdown();
    return 0;
}