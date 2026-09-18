/*
 * Events.h
 */

#pragma once

#include "ecs/Ecs.h"

#include <sol/sol.hpp>

#include <string>
#include <unordered_map>
#include <vector>

namespace BulletEngine {
namespace script {

// what scripts say to each other, by name rather than by reference
//
// sender does not know who listens, listener does not know who spoke, so
// gameplay systems stay apart
class EventBus {
public:
    // handler stands until entity that made it goes
    void listen(const std::string& name, ecs::Entity owner, sol::protected_function handler);

    // handlers run before this returns, in order they subscribed
    void emit(const std::string& name, sol::variadic_args args);

    void forget(ecs::Entity owner);     // entity died, its handlers go with it
    void clear();                       // play stopped, nothing survives it

private:
    struct Listener {
        ecs::Entity owner;
        sol::protected_function handler;
    };

    void drop(const std::string& name, size_t index);

    // events already running, so handler emitting into itself is caught
    int m_depth = 0;

    std::unordered_map<std::string, std::vector<Listener>> m_listeners;
};

} // namespace script
} // namespace BulletEngine
