/*
 * main.cpp
 */

// BulletRender
#include "app/Window.h"
#include "app/Loop.h"
#include "render/passes/Fog.h"
#include "render/passes/Grid.h"
#include "render/passes/Lines.h"
#include "render/passes/WorldAxis.h"
#include "render/Renderer.h"
#include "render/Shader.h"
#include "scene/Scene.h"
#include "scene/Model.h"
#include "scene/Camera.h"
#include "scene/Light.h"
#include "imgui.h"

// BulletPhysics
#include "collision/collider/BoxCollider.h"
#include "collision/collider/SphereCollider.h"
#include "collision/collider/GroundCollider.h"
#include "dynamics/body/Inertia.h"

// BulletEngine
#include "app/Application.h"
#include "ecs/Ecs.h"
#include "ecs/Components.h"
#include "ecs/systems/PhysicsSystem.h"
#include "ecs/systems/DebugDrawSystem.h"
#include "ecs/systems/CursorDragSystem.h"
#include "ecs/systems/RenderSystem.h"
#include "ecs/systems/ImGuiSystem.h"
#include "ecs/systems/InputSystem.h"

#include <memory>

using namespace BulletEngine;

// scene layout
static constexpr float GROUND_LEVEL = 0.0f;
static constexpr float CUBE_SIZE = 1.0f;
static constexpr float BALL_RADIUS = 0.5f;
static constexpr float DROP_HEIGHT = 6.0f;

static const std::string VERTEX_SHADER_PATH = "assets/shaders/normal.vert.glsl";
static const std::string FRAGMENT_SHADER_PATH = "assets/shaders/normal.frag.glsl";

namespace bp = BulletPhysics;
namespace bpc = BulletPhysics::collision;
namespace bpd = BulletPhysics::dynamics;

// everything spawned body needs beyond its shape
struct BodyDesc {
    bp::math::Vec3 position;
    glm::vec3 color;
    bpc::PhysicsMaterial material{};
    bp::math::Vec3 velocity{};
    bp::math::Quat orientation{};
};

static ecs::Entity spawnBody(ecs::World& world, const BodyDesc& desc, BulletRender::scene::Model* model, const std::shared_ptr<BulletRender::render::GraphicsShader>& shader)
{
    auto entity = world.create();

    auto& transform = world.add<ecs::TransformComponent>(entity);
    transform.transform.setPosition({
        static_cast<float>(desc.position.x), static_cast<float>(desc.position.y), static_cast<float>(desc.position.z)
    });

    auto& renderable = world.add<ecs::RenderableComponent>(entity);
    renderable.model = model;
    renderable.material.setShader(shader);
    renderable.material.setColor(desc.color);

    auto& rigidBody = world.add<ecs::RigidBodyComponent>(entity);
    rigidBody.body.setMass(1.0);
    rigidBody.body.setPosition(desc.position);
    rigidBody.body.setVelocity(desc.velocity);
    rigidBody.body.setOrientation(desc.orientation);

    return entity;
}

static ecs::Entity spawnCube(ecs::World& world, const BodyDesc& desc, BulletRender::scene::Model* model, const std::shared_ptr<BulletRender::render::GraphicsShader>& shader)
{
    const bp::math::Vec3 size{CUBE_SIZE, CUBE_SIZE, CUBE_SIZE};

    auto entity = spawnBody(world, desc, model, shader);
    world.get<ecs::RigidBodyComponent>(entity)->body.setInverseInertiaLocal(bpd::inertia::box(1.0, size));

    auto& collider = world.add<ecs::ColliderComponent>(entity);
    collider.collider = std::make_unique<bpc::collider::BoxCollider>(size);
    collider.collider->setMaterial(desc.material);

    return entity;
}

static ecs::Entity spawnBall(ecs::World& world, const BodyDesc& desc, BulletRender::scene::Model* model, const std::shared_ptr<BulletRender::render::GraphicsShader>& shader)
{
    auto entity = spawnBody(world, desc, model, shader);
    world.get<ecs::RigidBodyComponent>(entity)->body.setInverseInertiaLocal(bpd::inertia::sphere(1.0, BALL_RADIUS));

    auto& collider = world.add<ecs::ColliderComponent>(entity);
    collider.collider = std::make_unique<bpc::collider::SphereCollider>(BALL_RADIUS);
    collider.collider->setMaterial(desc.material);

    return entity;
}

int main()
{
    // window
    BulletRender::app::WindowConfig windowCfg{1280, 720, "BulletEngine", true, true};
    if (!BulletRender::app::Window::init(windowCfg))
    {
        return -1;
    }

    // renderer
    BulletRender::render::RenderConfig renderCfg{{0.05f, 0.05f, 0.08f, 1.0f}};
    BulletRender::render::Renderer::init(renderCfg);

    BulletRender::render::Renderer::registerPrePass(std::make_shared<BulletRender::render::Grid>());
    BulletRender::render::Renderer::registerPrePass(std::make_shared<BulletRender::render::WorldAxis>());

    auto lines = std::make_shared<BulletRender::render::Lines>(1.5f);
    BulletRender::render::Renderer::registerPrePass(lines);

    BulletRender::render::Renderer::registerPostPass(
        std::make_shared<BulletRender::render::Fog>(true, 20.0f, 70.0f));

    // scene
    BulletRender::scene::Scene scene;

    BulletRender::scene::FlyCamera& camera = *scene.createCamera<BulletRender::scene::FlyCamera>(
        glm::vec3{0.0f, 4.0f, 9.0f}, -90.0f, -14.0f
    );

    scene.createLight<BulletRender::scene::AmbientLight>()->setIntensity(0.3f);
    scene.createLight<BulletRender::scene::DirectionalLight>();

    // shared shader and models
    auto shader = std::make_shared<BulletRender::render::GraphicsShader>(VERTEX_SHADER_PATH, FRAGMENT_SHADER_PATH);

    BulletRender::scene::Box cubeModel(CUBE_SIZE, CUBE_SIZE, CUBE_SIZE);
    BulletRender::scene::Sphere ballModel(BALL_RADIUS, 32, 16);

    // ecs
    ecs::World world;

    // ground
    {
        auto entity = world.create();

        auto& rigidBody = world.add<ecs::RigidBodyComponent>(entity);
        rigidBody.body.setMotionType(bpd::MotionType::Static);

        auto& collider = world.add<ecs::ColliderComponent>(entity);
        collider.collider = std::make_unique<bpc::collider::GroundCollider>(GROUND_LEVEL);
    }

    // scenario 1: dropped cube
    spawnCube(world, {.position = {-7.5, DROP_HEIGHT, 0.0}, .color = {0.9f, 0.4f, 0.4f}}, &cubeModel, shader);

    // scenario 2: dropped ball
    spawnBall(world, {.position = {-4.5, DROP_HEIGHT, 0.0}, .color = {0.4f, 0.9f, 0.5f}, .material = bpc::materials::Rubber()}, &ballModel, shader);

    // scenario 3: pushed cube
    spawnCube(world, {.position = {-1.5, 0.5, 0.0}, .color = {0.4f, 0.5f, 0.9f}, .material = bpc::materials::Ice(), .velocity = {0.0, 0.0, 5.0}}, &cubeModel, shader);

    // scenario 4: pushed cube
    spawnBall(world, {.position = {1.5, BALL_RADIUS, 0.0}, .color = {0.9f, 0.9f, 0.4f}, .material = bpc::materials::Wood(), .velocity = {0.0, 0.0, 1.5}}, &ballModel, shader);

    // scenario 5: stack
    for (int i = 0; i < 4; i++)
    {
        spawnCube(world, {.position = {4.5, 0.5 + i * 1.05, 0.0}, .color = {0.9f, 0.3f + i * 0.1f, 0.5f}}, &cubeModel, shader);
    }

    // scenario 6: asimmetric fall
    spawnCube(world, {.position = {7.5, 0.5, 0.0}, .color = {0.9f, 0.6f, 0.3f}, .orientation = bp::math::Quat::fromAxisAngle({0.0, 1.0, 0.0}, 0.785)}, &cubeModel, shader);
    spawnCube(world, {.position = {7.5, DROP_HEIGHT, 0.0}, .color = {0.7f, 0.4f, 0.9f}, .orientation = bp::math::Quat::fromAxisAngle({1.0, 1.0, 0.0}, 0.9)}, &cubeModel, shader);

    // systems
    ecs::systems::PhysicsSystem physicsSystem;
    ecs::systems::RenderSystem renderSystem(scene);
    ecs::systems::DebugDrawSystem debugDrawSystem(lines);
    ecs::systems::CursorDragSystem cursorDragSystem(camera, physicsSystem);

    // phases
    app::Application app;
    app.setWorld(&world);

    app::Scheduler& scheduler = app.getScheduler();

    scheduler.add(app::Phase::PreUpdate, [&camera](const app::FrameContext& frame) {
        camera.update(frame.deltaTime);
        BulletRender::utils::Input::instance().update();
    }, 0, "input");

    scheduler.add(app::Phase::FixedUpdate, [&physicsSystem](const app::FrameContext& frame) {
        physicsSystem.step(*frame.world, frame.fixedDeltaTime);
    }, 0, "physics");

    scheduler.add(app::Phase::PostUpdate, [&cursorDragSystem](const app::FrameContext&) {
        cursorDragSystem.update();
    }, 0, "cursor drag");

    scheduler.add(app::Phase::Render, [&renderSystem](const app::FrameContext& frame) {
        renderSystem.render(*frame.world);
    }, 0, "scene");

    scheduler.add(app::Phase::Render, [&debugDrawSystem, &physicsSystem](const app::FrameContext& frame) {
        debugDrawSystem.draw(*frame.world, physicsSystem.getContacts());
    }, 10, "debug draw");

    // input
    ecs::systems::InputSystem inputSystem;
    inputSystem.bind(BulletRender::utils::InputKey::ESCAPE, []() {
        BulletRender::app::Window::setShouldClose(true);
    });
    inputSystem.bind(BulletRender::utils::InputKey::F2, [&debugDrawSystem]() {
        debugDrawSystem.setEnabled(!debugDrawSystem.isEnabled());
    });

    // imgui
    ecs::systems::ImGuiSystem imguiSystem;
    float lastDt = 0.0f;

    imguiSystem.add([&camera, &lastDt]() {
        ImGui::Begin("Debug");

        float fps = lastDt > 0.0f ? 1.0f / lastDt : 0.0f;
        ImGui::Text("FPS: %.1f", fps);

        ImGui::Separator();

        auto p = camera.getPosition();
        ImGui::Text("Camera:");
        ImGui::Text("   X: %.2f", p.x);
        ImGui::Text("   Y: %.2f", p.y);
        ImGui::Text("   Z: %.2f", p.z);

        ImGui::End();
    });

    // loop
    BulletRender::app::Loop loop(scene);
    loop.run(
        [&](float dt) {
            lastDt = dt;

            app.tick(dt);
            imguiSystem.render();

            world.flush();
        }
    );

    BulletRender::app::Window::shutdown();
    return 0;
}
