/*
 * main.cpp
 */

// BulletRender
#include "app/Loop.h"
#include "app/Window.h"
#include "render/passes/Fog.h"
#include "render/passes/Grid.h"
#include "render/passes/Lines.h"
#include "render/passes/WorldAxis.h"
#include "render/Renderer.h"
#include "render/Shader.h"
#include "scene/Camera.h"
#include "scene/Light.h"
#include "scene/Scene.h"

// BulletEngine
#include "app/Application.h"
#include "assets/Loaders.h"
#include "ecs/Ecs.h"
#include "ecs/systems/DebugDrawSystem.h"
#include "ecs/systems/HierarchySystem.h"
#include "ecs/systems/InputSystem.h"
#include "ecs/systems/PhysicsSystem.h"
#include "ecs/systems/PickSystem.h"
#include "ecs/systems/ReloadSystem.h"
#include "ecs/systems/RenderSystem.h"
#include "interface/Editor.h"
#include "io/Log.h"
#include "project/Project.h"

#include <memory>

using namespace BulletEngine;

namespace br = BulletRender;

static const std::string VERTEX_SHADER_PATH = "assets/shaders/normal.vert.glsl";
static const std::string FRAGMENT_SHADER_PATH = "assets/shaders/normal.frag.glsl";

int main(int argc, char** argv)
{
    // streams reach editor from here on, terminal still gets them
    io::Log::instance().capture();

    // folder editor opens, working one when none is named
    if (!project::Project::instance().open(argc > 1 ? argv[1] : "."))
    {
        return -1;
    }

    // window
    br::app::Loop::setDocking(true);
    br::render::Renderer::setOffscreen(true);

    br::app::WindowConfig windowCfg{1600, 900, "BulletEngine", true, true};
    if (!br::app::Window::init(windowCfg))
    {
        return -1;
    }

    // gl objects must die before the context does
    {
        // renderer
        br::render::RenderConfig renderCfg{{0.05f, 0.05f, 0.08f, 1.0f}};
        br::render::Renderer::init(renderCfg);

        auto lines = std::make_shared<br::render::Lines>(1.5f);
        lines->setDepthTest(false);     // gizmos stay visible through geometry

        br::render::Renderer::registerPrePass(std::make_shared<br::render::Grid>());
        br::render::Renderer::registerPrePass(std::make_shared<br::render::WorldAxis>());
        br::render::Renderer::registerOverlayPass(lines);
        br::render::Renderer::registerPostPass(std::make_shared<br::render::Fog>(true, 20.0f, 70.0f));

        // scene
        br::scene::Scene scene;

        br::scene::FlyCamera& camera = *scene.createCamera<br::scene::FlyCamera>(
            glm::vec3{0.0f, 5.0f, 6.0f}, -90.0f, -35.0f
        );

        scene.createLight<br::scene::AmbientLight>()->setIntensity(0.3f);
        scene.createLight<br::scene::DirectionalLight>();

        br::render::Renderer::setDefaultShader(
            std::make_shared<br::render::GraphicsShader>(VERTEX_SHADER_PATH, FRAGMENT_SHADER_PATH));

        // world
        assets::registerLoaders();

        ecs::World world;

        // systems
        ecs::systems::PhysicsSystem physicsSystem;
        physicsSystem.watch(world);

        ecs::systems::HierarchySystem hierarchySystem;
        ecs::systems::RenderSystem renderSystem(scene);
        ecs::systems::DebugDrawSystem debugDrawSystem(lines);
        ecs::systems::PickSystem pickSystem(camera, physicsSystem);
        ecs::systems::ReloadSystem reloadSystem;

        // editor
        interface::Editor editor(world, physicsSystem, debugDrawSystem);
        editor.openFirstScene();

        // phases
        app::Application app;
        app.setWorld(&world);

        app::Scheduler& scheduler = app.getScheduler();

        scheduler.add(app::Phase::PreUpdate, [&camera, &editor](const app::FrameContext& frame) {
            if (editor.isSceneFocused())
            {
                camera.update(frame.deltaTime);
            }

            br::utils::Input::instance().update();
        }, 0, "input");

        scheduler.add(app::Phase::PreUpdate, [&reloadSystem](const app::FrameContext& frame) {
            reloadSystem.update(*frame.world, frame.deltaTime);
        }, 10, "reload");

        scheduler.add(app::Phase::FixedUpdate, [&physicsSystem](const app::FrameContext& frame) {
            physicsSystem.step(*frame.world, frame.fixedDeltaTime);
        }, 0, "physics");

        scheduler.add(app::Phase::PostUpdate, [&pickSystem, &editor](const app::FrameContext& frame) {
            if (!editor.hasScenePick())
            {
                return;
            }

            editor.setSelection(pickSystem.pick(*frame.world, editor.getScenePick(), editor.getSceneSize()));
            editor.clearScenePick();
        }, 0, "pick");

        scheduler.add(app::Phase::PostUpdate, [&hierarchySystem](const app::FrameContext& frame) {
            hierarchySystem.update(*frame.world);
        }, 10, "hierarchy");

        scheduler.add(app::Phase::Render, [&renderSystem](const app::FrameContext& frame) {
            renderSystem.render(*frame.world);
        }, 0, "scene");

        scheduler.add(app::Phase::Render, [&debugDrawSystem, &editor, &physicsSystem](const app::FrameContext& frame) {
            debugDrawSystem.draw(*frame.world, editor.getSelection(), physicsSystem.getContacts());
        }, 10, "debug draw");

        // input
        ecs::systems::InputSystem inputSystem;

        inputSystem.bind(br::utils::InputKey::ESCAPE, []() {
            br::app::Window::setShouldClose(true);
        });

        // loop
        br::app::Loop loop(scene);
        loop.setBeforeFrame([&editor]() { editor.beforeFrame(); });

        loop.run([&](float dt) {
            editor.applySceneSize();

            app.tick(dt);
            editor.draw();

            world.flush();
        });
    }

    br::render::Renderer::shutdown();
    br::app::Window::shutdown();

    io::Log::instance().release();
    return 0;
}
