/*
 * ImGuiSystem.h
 */

#pragma once

#include <functional>

namespace BulletEngine {
namespace ecs {
namespace systems {

class ImGuiSystemBase {
public:
    void add(const std::function<void()>& display);
    void render();

private:
    std::vector<std::function<void()>> m_displays;
};

} // namespace systems
} // namespace ecs
} // namespace BulletEngine
