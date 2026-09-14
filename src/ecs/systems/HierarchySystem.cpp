/*
 * HierarchySystem.cpp
 */

#include "HierarchySystem.h"

#include "ecs/Components.h"

namespace BulletEngine {
namespace ecs {
namespace systems {

void HierarchySystem::update(World& world)
{
    for (Entity entity : world.getEntities())
    {
        auto* component = world.get<TransformComponent>(entity);

        if (!component)
        {
            continue;
        }

        auto* parent = find(world, component->parent);

        // parent gone or never set leaves the node at the root
        if (!parent)
        {
            component->parent = INVALID_ENTITY;

            if (component->transform.getParent())
            {
                component->transform.setParent(nullptr);
            }

            continue;
        }

        if (component->transform.getParent() != &parent->transform)
        {
            component->transform.setParent(&parent->transform);
        }
    }
}

bool HierarchySystem::canAttach(World& world, Entity child, Entity parent)
{
    if (child == INVALID_ENTITY || child == parent)
    {
        return false;
    }

    if (parent == INVALID_ENTITY)
    {
        return true;
    }

    for (Entity current = parent; current != INVALID_ENTITY; )
    {
        if (current == child)
        {
            return false;
        }

        auto* component = find(world, current);
        current = component ? component->parent : INVALID_ENTITY;
    }

    return true;
}

void HierarchySystem::attach(World& world, Entity child, Entity parent)
{
    if (!canAttach(world, child, parent))
    {
        return;
    }

    if (auto* component = find(world, child))
    {
        component->parent = parent;
    }
}

TransformComponent* HierarchySystem::find(World& world, Entity entity)
{
    return entity != INVALID_ENTITY && world.isAlive(entity) ? world.get<TransformComponent>(entity) : nullptr;
}

} // namespace systems
} // namespace ecs
} // namespace BulletEngine
