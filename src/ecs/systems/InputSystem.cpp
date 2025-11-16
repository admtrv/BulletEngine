/*
 * InputSystem.cpp
 */

#include "InputSystem.h"

namespace BulletEngine {
namespace ecs {
namespace systems {

void InputSystem::bind(BulletRender::utils::InputKey key, const std::function<void()>& callback)
{
    BulletRender::utils::Input::instance().bindKey(key, callback);
}

void InputSystem::unbind(BulletRender::utils::InputKey key)
{
    BulletRender::utils::Input::instance().unbindKey(key);
}

} // namespace systems
} // namespace ecs
} // namespace BulletEngine
