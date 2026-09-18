/*
 * CanvasSystem.cpp
 */

#include "CanvasSystem.h"

#include "ecs/Components.h"
#include "ecs/systems/ScriptSystem.h"

#include <algorithm>
#include <utility>

namespace BulletEngine {
namespace ecs {
namespace systems {

CanvasSystem::CanvasSystem(ScriptSystem& scripts) : m_scripts(scripts) {}

void CanvasSystem::draw(World& world, BulletRender::render::Canvas& canvas)
{
    std::vector<std::pair<int, Entity>> drawn;

    for (Entity entity : world.getEntities())
    {
        const auto* component = world.get<CanvasComponent>(entity);

        if (component && component->visible)
        {
            drawn.emplace_back(component->order, entity);
        }
    }

    // stable, so entities sharing order keep sequence world gave them
    std::stable_sort(drawn.begin(), drawn.end(), [](const auto& left, const auto& right) {
        return left.first < right.first;
    });

    for (const auto& [order, entity] : drawn)
    {
        m_scripts.drawCanvas(entity, canvas);
    }
}

} // namespace systems
} // namespace ecs
} // namespace BulletEngine
