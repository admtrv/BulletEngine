/*
 * DebugDrawSystem.h
 */

#pragma once

#include "ecs/Ecs.h"
#include "ecs/Components.h"

#include "collision/Collision.h"
#include "render/DebugDraw.h"

#include <memory>
#include <vector>

namespace BulletEngine {
namespace ecs {
namespace systems {

// draws what physics sees
class DebugDrawSystem {
public:
    explicit DebugDrawSystem(std::shared_ptr<BulletRender::render::Lines> lines);

    void draw(World& world, Entity selected, const std::vector<BulletPhysics::collision::Manifold>& contacts);

    // layers, selected entity keeps its axes either way
    bool isShowColliders() const { return m_showColliders; }
    void setShowColliders(bool show) { m_showColliders = show; }

    bool isShowPhysics() const { return m_showPhysics; }
    void setShowPhysics(bool show) { m_showPhysics = show; }

private:
    void drawColliders(World& world);
    void drawPhysics(World& world, const std::vector<BulletPhysics::collision::Manifold>& contacts);
    void drawAxes(World& world, Entity selected);

    BulletRender::render::DebugDraw m_debugDraw;

    bool m_showColliders = false;
    bool m_showPhysics = false;
};

} // namespace systems
} // namespace ecs
} // namespace BulletEngine
