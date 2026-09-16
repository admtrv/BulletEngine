/*
 * ScriptSystem.h
 */

#pragma once

#include "ecs/Ecs.h"

#include <sol/sol.hpp>

#include <unordered_map>
#include <unordered_set>

namespace BulletEngine {
namespace ecs {
namespace systems {

// runs lua of scripted entities
class ScriptSystem {
public:
    ScriptSystem();

    // world
    void observe(World& world);             // onDestroy needs components still standing

    // play
    void start(World& world);               // compiles what world holds
    void stop();                            // instances go

    // frame, one per phase
    void update(World& world, float dt);        // compiles entities spawned since last frame, then onUpdate
    void fixedUpdate(World& world, float dt);   // in step with physics, where forces belong
    void lateUpdate(World& world, float dt);    // after everything moved, where followers belong

private:
    // types

    // what script may define, names live in CALLBACK_NAMES
    enum class Callback : uint8_t {
        Start,
        Update,
        FixedUpdate,
        LateUpdate,
        Destroy,

        Count
    };

    // one entity, own copy of script globals
    struct Instance {
        sol::environment environment;
        sol::protected_function callbacks[static_cast<size_t>(Callback::Count)];
    };

    // machine
    void bind();                                        // what every script sees
    void attach(World& world, Entity entity);           // compiles one, skips one already compiled

    // calls
    void dispatch(World& world, Callback callback, float dt);       // one callback on every live instance
    void call(Instance& instance, Callback callback, float dt = 0.0f);

    sol::state m_lua;
    std::unordered_map<Entity, Instance> m_instances;
    std::unordered_set<Entity> m_broken;    // did not compile, left alone until play restarts
    bool m_running = false;
};

} // namespace systems
} // namespace ecs
} // namespace BulletEngine
