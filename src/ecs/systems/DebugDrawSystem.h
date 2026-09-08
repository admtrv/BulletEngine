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

    void draw(World& world, const std::vector<BulletPhysics::collision::Manifold>& contacts);

    // what gets drawn
    bool isEnabled() const { return m_enabled; }
    void setEnabled(bool enabled) { m_enabled = enabled; }

    void setShowColliders(bool show) { m_showColliders = show; }
    void setShowVelocities(bool show) { m_showVelocities = show; }
    void setShowContacts(bool show) { m_showContacts = show; }

private:
    void drawColliders(World& world);
    void drawVelocities(World& world);
    void drawContacts(const std::vector<BulletPhysics::collision::Manifold>& contacts);

    BulletRender::render::DebugDraw m_debugDraw;

    bool m_enabled = true;
    bool m_showColliders = true;
    bool m_showVelocities = true;
    bool m_showContacts = true;
};

} // namespace systems
} // namespace ecs
} // namespace BulletEngine
