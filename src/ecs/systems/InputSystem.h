/*
 * InputSystem.h
 */

#pragma once

#include "ecs/Ecs.h"
#include "ecs/Components.h"

#include "utils/Input.h"

#include <functional>

namespace BulletEngine {
namespace ecs {
namespace systems {

class InputSystem {
public:
    explicit InputSystem(ecs::World& world);

    void update(GLFWwindow* window);

    void setLaunchCallback(const std::function<void(ecs::Entity)>& callback);

private:
    ecs::World& m_world;
    std::function<void(ecs::Entity)> m_launchCallback;

    ecs::Entity createProjectile();
    void launchProjectile();
};

} // namespace systems
} // namespace ecs
} // namespace BulletEngine
