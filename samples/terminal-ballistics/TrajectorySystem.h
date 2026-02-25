/*
 * TrajectorySystem.h
 */

#pragma once

#include "ecs/Ecs.h"
#include "ecs/Components.h"
#include "render/passes/Lines.h"
#include "common/Components.h"
#include "Components.h"

#include <cmath>
#include <memory>

namespace BulletEngine {
namespace ecs {
namespace systems {

class EnergyTrajectorySystem {
public:
    explicit EnergyTrajectorySystem(std::shared_ptr<BulletRender::render::Lines> lines) : m_lines(std::move(lines)) {}

    void update(World& world);

private:
    // red (full energy) -> orange -> yellow -> green (zero energy)
    static glm::vec3 energyToColor(float ratio);

    std::shared_ptr<BulletRender::render::Lines> m_lines;
};

} // namespace systems
} // namespace ecs
} // namespace BulletEngine
