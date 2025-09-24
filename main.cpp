/*
 * main.cpp
 */

#include "window/window.h"
#include "render/renderer.h"
#include "scene/scene.h"
#include "scene/model.h"
#include "camera/camera.h"
#include "light/light.h"
#include "utils/time.h"

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
    auto shader = std::make_shared<luchrender::render::Shader>("shaders/normal.vert.glsl", "shaders/normal.frag.glsl");

    // scene
    luchrender::scene::Scene scene;
    scene.setCamera(&camera);
    scene.setLight(&light);

    luchrender::scene::SceneObject* object = scene.addObject(&model);

    // tune object
    object->getMaterial().setShader(shader);
    object->getMaterial().setColor({1.0f, 0.5f, 0.0f});
    object->getTransform().setPosition({0, 0, 0});

    // time
    luchrender::utils::FrameTimer timer;

    // loop
    while (!luchrender::window::Window::shouldClose())
    {
        float dt = timer.tick();

        luchrender::window::Window::pollEvents();

        // update camera
        camera.update(luchrender::window::Window::get(), dt);

        // rotate object continuously
        object->getTransform().rotate({0, 0, 0.5}, dt);

        // framebuffer size + aspect
        int fbw;
        int fbh;
        luchrender::window::Window::getSize(fbw, fbh);
        luchrender::render::Renderer::resizeViewport(fbw, fbh);
        scene.setAspect(fbh > 0 ? float(fbw) / float(fbh) : 1.0f);

        // render
        luchrender::render::Renderer::clear(0.05f, 0.05f, 0.08f, 1.0f);
        luchrender::render::Renderer::render(scene);
        luchrender::window::Window::swapBuffers();
    }

    luchrender::window::Window::shutdown();
    return 0;
}