/*
 * Application.cpp
 */

#include "Application.h"

namespace BulletEngine {
namespace app {

void Application::setFixedTimeStep(float timeStep)
{
    m_fixedTimeStep = (timeStep > 0.0f ? timeStep : 1.0f / 60.0f);
}

void Application::setMaxFixedSteps(int steps)
{
    m_maxFixedSteps = (steps > 0 ? steps : 1);
}

float Application::getAlpha() const
{
    return m_accumulator / m_fixedTimeStep;
}

void Application::tick(float deltaTime)
{
    m_frame++;

    const int steps = consumeFixedSteps(deltaTime);
    const FrameContext context = makeContext(deltaTime);

    m_scheduler.run(Phase::PreUpdate, context);

    for (int i = 0; i < steps; i++)
    {
        m_scheduler.run(Phase::FixedUpdate, context);
    }

    m_scheduler.run(Phase::Update, context);
    m_scheduler.run(Phase::PostUpdate, context);
    m_scheduler.run(Phase::Render, context);
    m_scheduler.run(Phase::RenderUi, context);
}

int Application::consumeFixedSteps(float deltaTime)
{
    if (deltaTime > 0.0f)
    {
        m_accumulator += deltaTime;
    }

    int steps = static_cast<int>(m_accumulator / m_fixedTimeStep);

    if (steps > m_maxFixedSteps)
    {
        // drop backlog, better slow motion for a moment than a stall
        steps = m_maxFixedSteps;
        m_accumulator = 0.0f;

        return steps;
    }

    m_accumulator -= steps * m_fixedTimeStep;

    return steps;
}

FrameContext Application::makeContext(float deltaTime) const
{
    FrameContext context;
    context.world = m_world;
    context.deltaTime = deltaTime;
    context.fixedDeltaTime = m_fixedTimeStep;
    context.alpha = getAlpha();
    context.frame = m_frame;

    return context;
}

} // namespace app
} // namespace BulletEngine
