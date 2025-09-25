/*
 * main.cpp
 */

// luchrender
#include "app/window.h"
#include "app/loop.h"
#include "render/renderer.h"
#include "render/shader.h"
#include "scene/scene.h"
#include "scene/model.h"
#include "scene/camera.h"
#include "scene/light.h"

// luchengine
#include "ecs/ecs.h"
#include "ecs/components.h"
#include "systems/render_system.h"

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
    luchrender::scene::Model model("assets/models/fox.obj");
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
    ecs::Entity entity = world.create();

    // components
    auto& transformComponent = world.add<ecs::TransformComponent>(entity);
    auto& renderableComponent = world.add<ecs::RenderableComponent>(entity);
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

            if (auto* t = world.get<luchengine::ecs::TransformComponent>(entity))
            {
                t->transform.rotateZ(0.5f * dt);
            }

            renderSystem.rebuild(world);
        }
    );

    luchrender::app::Window::shutdown();
    return 0;
}