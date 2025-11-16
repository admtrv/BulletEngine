/*
 * InputSystem.h
 */

#pragma once

#include "utils/Input.h"

#include <functional>

namespace BulletEngine {
namespace ecs {
namespace systems {

class InputSystem {
public:
    void bind(BulletRender::utils::InputKey key, const std::function<void()>& callback);
    void unbind(BulletRender::utils::InputKey key);
};

} // namespace systems
} // namespace ecs
} // namespace BulletEngine
