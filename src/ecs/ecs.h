/*
 * ecs.h
 */

#pragma once

#include <cstdint>
#include <vector>
#include <memory>
#include <unordered_map>

namespace luchengine {
namespace ecs {

using Entity = uint32_t;

class Component {
public:
    virtual ~Component() = default;
};

class World {
public:
    World() = default;
    ~World() = default;

    Entity create();
    void destroy(Entity entity);

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
        for (auto& up : it->second) {
            if (auto* p = dynamic_cast<C*>(up.get())) return p;
        }
        return nullptr;
    }

    template<class C>
    bool has(Entity entity) { return get<C>(entity) != nullptr; }

    const std::vector<Entity>& entities() const { return m_entities; }

private:
    Entity m_nextId = 1;
    std::vector<Entity> m_entities;
    std::unordered_map<Entity, std::vector<std::unique_ptr<Component>>> m_components;
};

} // namespace ecs
} // namespace luchengine
