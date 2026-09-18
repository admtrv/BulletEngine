/*
 * Events.cpp
 */

#include "Events.h"

#include <algorithm>
#include <iostream>

namespace BulletEngine {
namespace script {

constexpr int MAX_DEPTH = 16;       // events answering each other, cut before stack is

void EventBus::listen(const std::string& name, ecs::Entity owner, sol::protected_function handler)
{
    if (!handler.valid())
    {
        return;
    }

    m_listeners[name].push_back({owner, std::move(handler)});
}

// silences one handler, entry itself waits for quiet moment
void EventBus::drop(const std::string& name, size_t index)
{
    const auto listeners = m_listeners.find(name);

    if (listeners != m_listeners.end() && index < listeners->second.size())
    {
        listeners->second[index].handler = sol::protected_function();
    }
}

void EventBus::emit(const std::string& name, sol::variadic_args args)
{
    // event nobody listens to is ordinary, systems come and go
    if (!m_listeners.count(name))
    {
        return;
    }

    if (m_depth >= MAX_DEPTH)
    {
        std::cerr << "event recursion limit reached on " << name << ", dropped\n";
        return;
    }

    m_depth++;

    // handlers subscribing mid call would move map, so it is found again each round
    for (size_t i = 0; ; i++)
    {
        const auto listeners = m_listeners.find(name);

        if (listeners == m_listeners.end() || i >= listeners->second.size())
        {
            break;
        }

        // taken by value, calling it may reach list it lives in
        const sol::protected_function handler = listeners->second[i].handler;

        if (!handler.valid())
        {
            continue;
        }

        const sol::protected_function_result result = handler(args);

        if (!result.valid())
        {
            std::cerr << "event handler failed on " << name
                      << ": " << result.get<sol::error>().what() << '\n';

            // dropped, otherwise it fails again on every emit
            drop(name, i);
        }
    }

    m_depth--;

    // silenced entries pile up otherwise, outermost emit is where removing them is safe
    if (m_depth == 0)
    {
        if (const auto listeners = m_listeners.find(name); listeners != m_listeners.end())
        {
            std::erase_if(listeners->second, [](const Listener& listener) { return !listener.handler.valid(); });
        }
    }
}

void EventBus::forget(ecs::Entity owner)
{
    for (auto& [name, listeners] : m_listeners)
    {
        // emit may be walking this list, so handlers go quiet rather than away
        if (m_depth > 0)
        {
            for (Listener& listener : listeners)
            {
                if (listener.owner == owner)
                {
                    listener.handler = sol::protected_function();
                }
            }

            continue;
        }

        std::erase_if(listeners, [owner](const Listener& listener) { return listener.owner == owner; });
    }
}

void EventBus::clear()
{
    m_listeners.clear();
    m_depth = 0;
}

} // namespace script
} // namespace BulletEngine
