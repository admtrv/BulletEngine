/*
 * ScriptSystem.cpp
 */

#include "ScriptSystem.h"

#include "ecs/Components.h"
#include "reflect/Registry.h"
#include "script/Binding.h"

#include <iostream>

namespace BulletEngine {
namespace ecs {
namespace systems {

ScriptSystem::ScriptSystem()
{
    m_lua.open_libraries(sol::lib::base, sol::lib::math, sol::lib::string, sol::lib::table);
    bind();
}

// components of entity, by name reflection registered
static void bindComponents(sol::environment& environment, World& world, Entity entity)
{
    environment["get"] = [&world, entity](const std::string& name) -> script::Handle {
        const reflect::Type* type = reflect::Registry::instance().find(name);

        if (!type)
        {
            std::cerr << "script asked for unknown component: " << name << '\n';
            return {};
        }

        return {&world, entity, type};
    };
}

void ScriptSystem::bind()
{
    script::bindTypes(m_lua);

    // print reaches editor console, streams mirrored there
    m_lua.set_function("print", [](sol::variadic_args args) {
        std::string line;

        for (auto arg : args)
        {
            line += line.empty() ? "" : "\t";
            line += arg.as<std::string>();
        }

        std::cout << line << '\n';
    });
}

void ScriptSystem::attach(World& world, Entity entity)
{
    if (m_instances.count(entity))
    {
        return;
    }

    const auto* component = world.get<ScriptComponent>(entity);

    if (!component || !component->script)
    {
        return;
    }

    // own environment, so entities sharing file keep own state
    sol::environment environment(m_lua, sol::create, m_lua.globals());

    const sol::protected_function_result result =
        m_lua.safe_script(component->script->source, environment, sol::script_pass_on_error);

    if (!result.valid())
    {
        std::cerr << "script failed: " << component->getScriptKey()
                  << " (" << result.get<sol::error>().what() << ")\n";
        return;
    }

    environment["entity"] = entity;
    bindComponents(environment, world, entity);

    if (sol::protected_function start = environment["start"]; start.valid())
    {
        start();
    }

    m_instances.emplace(entity, Instance{environment, environment["update"]});
}

void ScriptSystem::start(World& world)
{
    if (m_running)
    {
        return;
    }

    m_running = true;

    for (Entity entity : world.getEntities())
    {
        attach(world, entity);
    }
}

void ScriptSystem::stop()
{
    m_running = false;
    m_instances.clear();
}

void ScriptSystem::update(World& world, float dt)
{
    if (!m_running)
    {
        return;
    }

    // entities spawned since last frame get script here
    for (Entity entity : world.getEntities())
    {
        attach(world, entity);
    }

    for (auto it = m_instances.begin(); it != m_instances.end(); )
    {
        // destroyed entity takes instance, id may come back on new one
        if (!world.isAlive(it->first))
        {
            it = m_instances.erase(it);
            continue;
        }

        Instance& instance = it->second;
        ++it;

        if (!instance.update.valid())
        {
            continue;
        }

        const sol::protected_function_result result = instance.update(dt);

        if (!result.valid())
        {
            std::cerr << "script error: " << result.get<sol::error>().what() << '\n';

            // broken update would repeat every frame, runs once and stops
            instance.update = sol::protected_function();
        }
    }
}

} // namespace systems
} // namespace ecs
} // namespace BulletEngine
