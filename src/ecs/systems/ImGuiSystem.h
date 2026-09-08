/*
 * ImGuiSystem.h
 */

#pragma once

#include <functional>
#include <vector>

namespace BulletEngine {
namespace ecs {
namespace systems {

// panels drawn every frame
class ImGuiSystem {
public:
    void add(std::function<void()> display);
    void render();

private:
    std::vector<std::function<void()>> m_displays;
};

} // namespace systems
} // namespace ecs
} // namespace BulletEngine
