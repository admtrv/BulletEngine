/*
 * ImGuiSystem.h
 */

#pragma once

#include "dynamics/PhysicsWorld.h"
#include "scene/Camera.h"
#include "ecs/Ecs.h"

#include "imgui.h"

namespace BulletEngine {
namespace ecs {
namespace systems {

class ImGuiSystem {
public:
    explicit ImGuiSystem(BulletPhysic::dynamics::PhysicsWorld& physicsWorld, BulletRender::scene::Camera& camera, World& world);

    void render(float dt);

private:
    BulletPhysic::dynamics::PhysicsWorld& m_physicsWorld;
    BulletRender::scene::Camera& m_camera;
    World& m_world;

    bool m_showDebug = true;
    bool m_showProjectile = true;
};

} // namespace systems
} // namespace ecs
} // namespace BulletEngine
