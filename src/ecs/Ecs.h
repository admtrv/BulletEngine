/*
 * Ecs.h
 */

#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <typeindex>
#include <unordered_map>
#include <vector>

namespace BulletEngine {
namespace ecs {

using Entity = uint32_t;
inline constexpr Entity INVALID_ENTITY = 0;

class Component {
public:
    virtual ~Component() = default;
};

using Listener = std::function<void(Entity)>;

class World {
public:
    World() = default;
    ~World() = default;

    Entity create();

    // marked now and dropped by flush
    void destroy(Entity entity);
    void clear();               // all at once, walking getEntities while destroying would skip half
    void flush();

    // called with each entity right before its components go
    void addListener(Listener listener) { m_listeners.push_back(std::move(listener)); }

    bool isAlive(Entity entity) const;

    template<class C, class... Args>
    C& add(Entity entity, Args&&... args)
    {
        auto& vec = m_components[entity];
        vec.emplace_back(std::make_unique<C>(std::forward<Args>(args)...));
        return *static_cast<C*>(vec.back().get());
    }

    template<class C>
    C* get(Entity entity)
    {
        auto it = m_components.find(entity);
        if (it == m_components.end()) return nullptr;
        for (auto& up : it->second)
        {
            if (auto* p = dynamic_cast<C*>(up.get()))
            {
                return p;
            }
        }
        return nullptr;
    }

    template<class C>
    bool has(Entity entity) { return get<C>(entity) != nullptr; }

    const std::vector<Entity>& getEntities() const { return m_entities; }

    const std::vector<std::unique_ptr<Component>>& getComponents(Entity entity) const;

    Component& attach(Entity entity, std::unique_ptr<Component> component);
    void detach(Entity entity, std::type_index type);

    bool has(Entity entity, std::type_index type) const;

private:
    Entity m_nextId = 1;
    std::vector<Entity> m_entities;
    std::vector<Entity> m_destroyed;
    std::unordered_map<Entity, std::vector<std::unique_ptr<Component>>> m_components;
    std::vector<Listener> m_listeners;
};

} // namespace ecs
} // namespace BulletEngine
