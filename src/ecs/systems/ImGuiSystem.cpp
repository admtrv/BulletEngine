/*
 * ImGuiSystem.cpp
 */

#include "ImGuiSystem.h"

namespace BulletEngine {
namespace ecs {
namespace systems {

void ImGuiSystem::add(std::function<void()> display)
{
    m_displays.push_back(std::move(display));
}

void ImGuiSystem::render()
{
    for (const auto& display : m_displays)
    {
        display();
    }
}

} // namespace systems
} // namespace ecs
} // namespace BulletEngine
