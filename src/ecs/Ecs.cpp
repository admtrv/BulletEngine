/*
 * Ecs.cpp
 */

#include "Ecs.h"

#include <algorithm>

namespace BulletEngine {
namespace ecs {

Entity World::create()
{
    Entity entity = m_nextId++;
    m_entities.push_back(entity);
    return entity;
}

void World::destroy(Entity entity)
{
    const auto it = std::find(m_entities.begin(), m_entities.end(), entity);

    if (it == m_entities.end())
    {
        return;
    }

    *it = m_entities.back();
    m_entities.pop_back();

    // out of the world at once, its components live until the frame ends
    m_destroyed.push_back(entity);
}

void World::flush()
{
    for (Entity entity : m_destroyed)
    {
        m_components.erase(entity);
    }

    m_destroyed.clear();
}

bool World::isAlive(Entity entity) const
{
    return std::find(m_entities.begin(), m_entities.end(), entity) != m_entities.end();
}

} // namespace ecs
} // namespace BulletEngine
