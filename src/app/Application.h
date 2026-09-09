/*
 * Application.h
 */

#pragma once

#include "app/Phases.h"
#include "app/Scheduler.h"

#include <cstdint>

namespace BulletEngine {

// fwd
namespace ecs { class World; }

namespace app {

// drives one frame through the phases
class Application {
public:
    Application() = default;

    void tick(float deltaTime);

    // fixed step
    float getFixedTimeStep() const { return m_fixedTimeStep; }
    void setFixedTimeStep(float timeStep);

    int getMaxFixedSteps() const { return m_maxFixedSteps; }
    void setMaxFixedSteps(int steps);

    float getAlpha() const;     // 0..1 towards next fixed step

    // contents
    Scheduler& getScheduler() { return m_scheduler; }
    const Scheduler& getScheduler() const { return m_scheduler; }

    ecs::World* getWorld() const { return m_world; }
    void setWorld(ecs::World* world) { m_world = world; }   // not owned

    uint64_t getFrame() const { return m_frame; }

private:
    int consumeFixedSteps(float deltaTime);
    FrameContext makeContext(float deltaTime) const;

    Scheduler m_scheduler;
    ecs::World* m_world = nullptr;

    float m_fixedTimeStep = 1.0f / 60.0f;
    int m_maxFixedSteps = 8;
    float m_accumulator = 0.0f;

    uint64_t m_frame = 0;
};

} // namespace app
} // namespace BulletEngine
