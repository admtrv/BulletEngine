/*
 * DebugDrawSystem.h
 */

#pragma once

#include "ecs/Ecs.h"
#include "ecs/Components.h"

#include "render/DebugDraw.h"

#include <memory>

namespace BulletEngine {
namespace ecs {
namespace systems {

// draws what physics sees
class DebugDrawSystem {
public:
    explicit DebugDrawSystem(std::shared_ptr<BulletRender::render::Lines> lines);

    void draw(World& world, Entity selected);

    bool isEnabled() const { return m_enabled; }
    void setEnabled(bool enabled) { m_enabled = enabled; }

private:
    void drawColliders(World& world);
    void drawAxes(World& world, Entity selected);

    BulletRender::render::DebugDraw m_debugDraw;

    bool m_enabled = true;
};

} // namespace systems
} // namespace ecs
} // namespace BulletEngine
