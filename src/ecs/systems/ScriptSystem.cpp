/*
 * ScriptSystem.cpp
 */

#include "ScriptSystem.h"

#include "ecs/Components.h"
#include "script/Api.h"
#include "script/Binding.h"

#include <iostream>

namespace BulletEngine {
namespace ecs {
namespace systems {

// what script may define, order matches Callback
constexpr const char* CALLBACK_NAMES[] = {
    "onStart",
    "onUpdate",
    "onFixedUpdate",
    "onLateUpdate",
    "onDestroy",
    "onCollisionEnter",
    "onCollisionExit",
    "onTriggerEnter",
    "onTriggerExit"
};

// machine

ScriptSystem::ScriptSystem()
{
    m_lua.open_libraries(sol::lib::base, sol::lib::math, sol::lib::string, sol::lib::table);
    bind();
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
    if (m_instances.count(entity) || m_broken.count(entity))
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

        // compiling it again every frame would flood the console
        m_broken.insert(entity);
        return;
    }

    environment["entity"] = entity;

    script::installComponents(environment, world, entity);
    script::installInput(environment);
    script::installPhysics(environment, world, entity);
    script::installWorld(environment, world);

    Instance instance{environment, {}};

    for (size_t i = 0; i < static_cast<size_t>(Callback::Count); i++)
    {
        instance.callbacks[i] = environment[CALLBACK_NAMES[i]];
    }

    call(instance, Callback::Start);
    m_instances.emplace(entity, std::move(instance));
}

// world

void ScriptSystem::observe(World& world)
{
    world.addListener([this](Entity entity) {
        const auto it = m_instances.find(entity);

        if (it == m_instances.end())
        {
            return;
        }

        // components are still there, script may read them one last time
        call(it->second, Callback::Destroy);
        m_instances.erase(it);
    });
}

// play

void ScriptSystem::start(World& world)
{
    if (m_running)
    {
        return;
    }

    m_running = true;

    // onStart may spawn, which would grow the list being walked
    const std::vector<Entity> entities = world.getEntities();

    for (Entity entity : entities)
    {
        attach(world, entity);
    }
}

void ScriptSystem::stop()
{
    // whatever the world listener did not take, ends here
    for (auto& [entity, instance] : m_instances)
    {
        call(instance, Callback::Destroy);
    }

    m_running = false;
    m_instances.clear();
    m_broken.clear();
}

// frame

void ScriptSystem::update(World& world, float dt)
{
    // entities spawned since last frame get script here, dispatch checks the rest
    if (m_running)
    {
        // a script may spawn while this walks, so the list is taken by value
        const std::vector<Entity> entities = world.getEntities();

        for (Entity entity : entities)
        {
            attach(world, entity);
        }
    }

    dispatch(world, Callback::Update, dt);
}

void ScriptSystem::fixedUpdate(World& world, float dt)
{
    dispatch(world, Callback::FixedUpdate, dt);
}

void ScriptSystem::lateUpdate(World& world, float dt)
{
    dispatch(world, Callback::LateUpdate, dt);
}

void ScriptSystem::deliver(World& world, const std::vector<ContactEvent>& events)
{
    if (!m_running)
    {
        return;
    }

    for (const ContactEvent& event : events)
    {
        const auto it = m_instances.find(event.self);

        // most contacts happen between entities that carry no script
        if (it == m_instances.end() || !world.isAlive(event.self))
        {
            continue;
        }

        const bool entering = event.phase == ContactPhase::Begin;

        const Callback callback = event.trigger
            ? (entering ? Callback::TriggerEnter : Callback::TriggerExit)
            : (entering ? Callback::CollisionEnter : Callback::CollisionExit);

        callContact(it->second, callback, event);
    }
}

// calls

void ScriptSystem::dispatch(World& world, Callback callback, float dt)
{
    if (!m_running)
    {
        return;
    }

    for (auto& [entity, instance] : m_instances)
    {
        // script may destroy entity mid frame, instance waits for flush
        if (world.isAlive(entity))
        {
            call(instance, callback, dt);
        }
    }
}

// callback that throws is dropped, otherwise it fails again every frame
void ScriptSystem::report(Instance& instance, Callback callback, const sol::protected_function_result& result)
{
    if (result.valid())
    {
        return;
    }

    std::cerr << "script error in " << CALLBACK_NAMES[static_cast<size_t>(callback)]
              << ": " << result.get<sol::error>().what() << '\n';

    instance.callbacks[static_cast<size_t>(callback)] = sol::protected_function();
}

void ScriptSystem::callContact(Instance& instance, Callback callback, const ContactEvent& event)
{
    sol::protected_function& function = instance.callbacks[static_cast<size_t>(callback)];

    if (!function.valid())
    {
        return;
    }

    report(instance, callback, function(event.other, event.point, event.normal, event.depth));
}

void ScriptSystem::call(Instance& instance, Callback callback, float dt)
{
    sol::protected_function& function = instance.callbacks[static_cast<size_t>(callback)];

    if (!function.valid())
    {
        return;
    }

    // per frame callbacks take time, start and destroy take nothing
    const bool timed = callback != Callback::Start && callback != Callback::Destroy;

    report(instance, callback, timed ? function(dt) : function());
}

} // namespace systems
} // namespace ecs
} // namespace BulletEngine
