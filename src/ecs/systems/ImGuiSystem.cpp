/*
 * ImGuiSystem.cpp
 */

#include "ImGuiSystem.h"

namespace BulletEngine {
namespace ecs {
namespace systems {

void ImGuiSystemBase::add(const std::function<void()>& display)
{
    m_displays.push_back(std::move(display));
}

void ImGuiSystemBase::render()
{
    for (auto& display : m_displays)
    {
        display();
    }
}

} // namespace systems
} // namespace ecs
} // namespace BulletEngine
