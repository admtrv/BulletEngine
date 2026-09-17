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
#include "ecs/systems/ScriptSystem.h"
#include "interface/Editor.h"
#include "io/Log.h"
#include "project/Project.h"
#include "script/Api.h"

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
    br::app::Loop::setDrawScene(false);      // editor draws the scene into its panels

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

        auto grid = std::make_shared<br::render::Grid>();
        auto worldAxis = std::make_shared<br::render::WorldAxis>();

        br::render::Renderer::registerPrePass(grid);
        br::render::Renderer::registerPrePass(worldAxis);
        br::render::Renderer::registerOverlayPass(lines);
        br::render::Renderer::registerPostPass(std::make_shared<br::render::Fog>(true, 20.0f, 70.0f));

        // scene
        br::scene::Scene scene;

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
        ecs::systems::ReloadSystem reloadSystem;

        // editor
        interface::Editor editor(world, physicsSystem, debugDrawSystem);

        ecs::systems::ScriptSystem scriptSystem(physicsSystem, editor);
        scriptSystem.observe(world);

        // its own tools, the game view goes without them
        editor.addEditorPass(grid);
        editor.addEditorPass(worldAxis);
        editor.addEditorPass(lines);

        editor.openFirstScene();

        // picking looks through the camera editor owns
        ecs::systems::PickSystem pickSystem(editor.getCamera(), physicsSystem);

        // phases
        app::Application app;
        app.setWorld(&world);

        app::Scheduler& scheduler = app.getScheduler();

        scheduler.add(app::Phase::PreUpdate, [&editor](const app::FrameContext& frame) {
            if (editor.isSceneFocused())
            {
                editor.getCamera().update(frame.deltaTime);
            }

            br::utils::Input::instance().update();
        }, 0, "input");

        scheduler.add(app::Phase::PreUpdate, [&reloadSystem](const app::FrameContext& frame) {
            reloadSystem.update(*frame.world, frame.deltaTime);
        }, 10, "reload");

        editor.addModeListener([&scriptSystem, &world](interface::Mode mode) {
            if (mode == interface::Mode::Play)
            {
                scriptSystem.start(world);
            }
            else
            {
                scriptSystem.stop();
            }
        });

        scheduler.add(app::Phase::Update, [&scriptSystem](const app::FrameContext& frame) {
            scriptSystem.update(*frame.world, frame.deltaTime);
        }, 0, "scripts");

        // forces land before the step that reads them
        scheduler.add(app::Phase::FixedUpdate, [&scriptSystem](const app::FrameContext& frame) {
            scriptSystem.fixedUpdate(*frame.world, frame.fixedDeltaTime);
        }, -10, "scripts fixed");

        scheduler.add(app::Phase::PostUpdate, [&scriptSystem](const app::FrameContext& frame) {
            scriptSystem.lateUpdate(*frame.world, frame.deltaTime);
        }, 0, "scripts late");

        scheduler.add(app::Phase::FixedUpdate, [&physicsSystem, &editor](const app::FrameContext& frame) {
            // idle world still keeps physics in step, picking casts rays into it
            if (editor.isPlaying())
            {
                physicsSystem.step(*frame.world, frame.fixedDeltaTime);
            }
            else
            {
                physicsSystem.sync(*frame.world);
            }
        }, 0, "physics");

        // contacts reach scripts once the step is over, so they may spawn and destroy freely
        scheduler.add(app::Phase::FixedUpdate, [&scriptSystem, &physicsSystem](const app::FrameContext& frame) {
            scriptSystem.deliver(*frame.world, physicsSystem.getEvents());
            physicsSystem.clearEvents();
        }, 10, "contacts");

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
            app.tick(dt);
            editor.renderViews(scene);
            editor.draw();

            world.flush();
            br::utils::Input::instance().endFrame();
        });
    }

    br::render::Renderer::shutdown();
    br::app::Window::shutdown();

    io::Log::instance().release();
    return 0;
}
