/*
 * Ecs.cpp
 */

#include "Ecs.h"

#include <algorithm>

namespace BulletEngine {
namespace ecs {

static auto findComponent(const std::vector<std::unique_ptr<Component>>& components, std::type_index type)
{
    return std::find_if(components.begin(), components.end(),
        [type](const std::unique_ptr<Component>& component) { return std::type_index(typeid(*component)) == type; });
}

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

    m_entities.erase(it);

    // out of the world at once, its components live until the frame ends
    m_destroyed.push_back(entity);
}

void World::flush()
{
    for (Entity entity : m_destroyed)
    {
        for (const Listener& listener : m_listeners)
        {
            listener(entity);
        }

        m_components.erase(entity);
    }

    m_destroyed.clear();
}

const std::vector<std::unique_ptr<Component>>& World::getComponents(Entity entity) const
{
    static const std::vector<std::unique_ptr<Component>> empty;

    const auto it = m_components.find(entity);
    return it != m_components.end() ? it->second : empty;
}

Component& World::attach(Entity entity, std::unique_ptr<Component> component)
{
    auto& vec = m_components[entity];
    vec.push_back(std::move(component));
    return *vec.back();
}

void World::detach(Entity entity, std::type_index type)
{
    const auto it = m_components.find(entity);

    if (it == m_components.end())
    {
        return;
    }

    auto& components = it->second;
    const auto found = findComponent(components, type);

    if (found != components.end())
    {
        components.erase(found);
    }
}

bool World::has(Entity entity, std::type_index type) const
{
    const auto it = m_components.find(entity);

    if (it == m_components.end())
    {
        return false;
    }

    return findComponent(it->second, type) != it->second.end();
}

bool World::isAlive(Entity entity) const
{
    return std::find(m_entities.begin(), m_entities.end(), entity) != m_entities.end();
}

} // namespace ecs
} // namespace BulletEngine
