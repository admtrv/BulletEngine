/*
 * TrajectorySystem.h
 */

#pragma once

#include "ecs/Ecs.h"
#include "ecs/Components.h"
#include "render/passes/Lines.h"

namespace BulletEngine {
namespace systems {

class TrajectorySystem {
public:
    explicit TrajectorySystem(std::shared_ptr<BulletRender::render::Lines> lines) : m_lines(std::move(lines)) {}

    void update(ecs::World& world);

private:
    std::shared_ptr<BulletRender::render::Lines> m_lines;
};

} // namespace systems
} // namespace BulletEngine
