/*
 * WorldApi.cpp
 */

#include "Api.h"
#include "Binding.h"

#include "ecs/Components.h"
#include "reflect/Registry.h"

#include <iostream>
#include <string>

namespace BulletEngine {
namespace script {

// component of registered type, attached to entity, handle empty when it cannot be built
static Handle addComponent(ecs::World& world, ecs::Entity entity, const std::string& name)
{
    const reflect::Type* type = reflect::Registry::instance().find(name);

    if (!type)
    {
        std::cerr << "script asked for unknown component: " << name << '\n';
        return {};
    }

    // entity already carrying it keeps the one it has
    if (world.has(entity, type->getIndex()))
    {
        return {&world, entity, type};
    }

    auto* component = static_cast<ecs::Component*>(type->create());

    if (!component)
    {
        std::cerr << "component cannot be built from script: " << name << '\n';
        return {};
    }

    world.attach(entity, std::unique_ptr<ecs::Component>(component));
    return {&world, entity, type};
}

void installWorld(sol::environment& environment, ecs::World& world)
{
    sol::table table = environment.create_named("world");

    // named entity at the origin, caller adds the rest
    table["spawn"] = [&world](sol::optional<std::string> name) {
        const ecs::Entity entity = world.create();

        auto& component = world.add<ecs::NameComponent>(entity);
        component.name = name.value_or("Entity");

        world.add<ecs::TransformComponent>(entity);
        return entity;
    };

    // marked now, components live until the frame ends
    table["destroy"] = [&world](ecs::Entity entity) {
        world.destroy(entity);
    };

    table["isAlive"] = [&world](ecs::Entity entity) {
        return world.isAlive(entity);
    };

    // component by reflected name, handle lets the script fill it in
    table["add"] = [&world](ecs::Entity entity, const std::string& name) {
        return addComponent(world, entity, name);
    };
}

} // namespace script
} // namespace BulletEngine
