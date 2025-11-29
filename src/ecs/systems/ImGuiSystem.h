/*
 * ImGuiSystem.h
 */

#pragma once

#include "dynamics/PhysicsWorld.h"

#include "imgui.h"

namespace BulletEngine {
namespace ecs {
namespace systems {

class ImGuiSystem {
public:
    explicit ImGuiSystem(BulletPhysic::dynamics::PhysicsWorld& physicsWorld);

    void render();

private:
    BulletPhysic::dynamics::PhysicsWorld& m_physicsWorld;
};

} // namespace systems
} // namespace ecs
} // namespace BulletEngine
