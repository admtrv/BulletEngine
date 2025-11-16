/*
 * Ecs.cpp
 */

#include "Ecs.h"

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
    for (size_t i = 0; i < m_entities.size(); i++)
    {
        if (m_entities[i] == entity)
        {
            m_entities[i] = m_entities.back();
            m_entities.pop_back();
            break;
        }
    }
    m_components.erase(entity);
}

} // namespace ecs
} // namespace BulletEngine
