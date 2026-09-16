/*
 * ScriptSystem.h
 */

#pragma once

#include "ecs/Ecs.h"

#include <sol/sol.hpp>

#include <unordered_map>

namespace BulletEngine {
namespace ecs {
namespace systems {

// runs lua of scripted entities
class ScriptSystem {
public:
    ScriptSystem();

    // play began, compiles what world holds
    void start(World& world);

    // play ended, instances go
    void stop();

    // compiles entities spawned since last frame, then updates all
    void update(World& world, float dt);

private:
    // one entity, own copy of script globals
    struct Instance {
        sol::environment environment;
        sol::protected_function update;
    };

    void bind();

    // compiles script of one entity, skips one already compiled
    void attach(World& world, Entity entity);

    sol::state m_lua;
    std::unordered_map<Entity, Instance> m_instances;
    bool m_running = false;
};

} // namespace systems
} // namespace ecs
} // namespace BulletEngine
