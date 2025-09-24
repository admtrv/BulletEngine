/*
 * main.cpp
 */

#include "window/window.h"
#include "render/renderer.h"
#include "scene/scene.h"
#include "scene/model.h"
#include "camera/camera.h"
#include "light/light.h"
#include "render/shader.h"
#include "utils/time.h"

#include "ecs/ecs.h"
#include "ecs/components.h"
#include "systems/render_system.h"

using namespace luchengine;

int main()
{
    // window
    luchrender::window::WindowConfig windowCfg{800, 600, "Demo", true};
    if (!luchrender::window::Window::init(windowCfg))
    {
        return -1;
    }

    // renderer
    luchrender::render::Renderer::init();

    // camera
    luchrender::camera::FlyCamera camera({0,1.5f,5.0f});

    // light
    luchrender::light::DirectionalLight light;

    // model
    luchrender::scene::Model model("assets/models/fox.obj");

    // shader
    auto shader = std::make_shared<luchrender::render::Shader>("assets/shaders/normal.vert.glsl", "assets/shaders/normal.frag.glsl");

    // scene
    luchrender::scene::Scene scene;
    scene.setCamera(&camera);
    scene.setLight(&light);

    // ecs
    ecs::World world;

    auto entity = world.create();

    auto& transformComponent = world.add<ecs::TransformComponent>(entity);
    transformComponent.transform.setIdentity();
    transformComponent.transform.setPosition({0, 0, 0});

    auto& renderableComponent = world.add<ecs::RenderableComponent>(entity);
    renderableComponent.model = &model;
    renderableComponent.material.setShader(shader);
    renderableComponent.material.setColor({1.0f, 0.5f, 0.0f});

    systems::RenderSystem renderSystem(scene);

    // time
    luchrender::utils::FrameTimer timer;

    // loop
    while (!luchrender::window::Window::shouldClose())
    {
        float dt = timer.tick();

        luchrender::window::Window::pollEvents();

        // update camera
        camera.update(luchrender::window::Window::get(), dt);

        // rotate entity
        if (auto* t = world.get<ecs::TransformComponent>(entity))
        {
            t->transform.rotateZ(0.5f * dt);
        }

        // framebuffer size + aspect
        int fbw;
        int fbh;
        luchrender::window::Window::getSize(fbw, fbh);
        luchrender::render::Renderer::resizeViewport(fbw, fbh);
        scene.setAspect(fbh > 0 ? float(fbw) / float(fbh) : 1.0f);

        // sync and render
        renderSystem.rebuild(world);
        luchrender::render::Renderer::clear(0.05f, 0.05f, 0.08f, 1.0f);
        luchrender::render::Renderer::render(scene);
        luchrender::window::Window::swapBuffers();
    }

    luchrender::window::Window::shutdown();
    return 0;
}