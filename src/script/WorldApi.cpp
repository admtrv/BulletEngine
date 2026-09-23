/*
 * WorldApi.cpp
 */

#include "Api.h"
#include "Binding.h"

#include "ecs/Components.h"
#include "project/Project.h"
#include "reflect/Registry.h"
#include "scene/Serializer.h"

#include <string>

namespace BulletEngine {
namespace script {

// component of registered type, attached to entity, handle empty when it cannot be built
static Handle addComponent(ecs::World& world, ecs::Entity entity, const std::string& name)
{
    const reflect::Type* type = findType(name);

    if (!type)
    {
        return {};
    }

    // entity already carrying it keeps one it has
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

    // named entity at origin, caller adds rest
    table["spawn"] = [&world](sol::optional<std::string> name) {
        const ecs::Entity entity = world.create();

        auto& component = world.add<ecs::IdentityComponent>(entity);
        component.name = name.value_or("Entity");

        world.add<ecs::TransformComponent>(entity);
        return entity;
    };

    // copy of entity that already stands, cheap way to repeat something
    table["clone"] = [&world](ecs::Entity entity) {
        return scene::clone(world, entity);
    };

    // entity built from prefab file, nothing when key leads nowhere
    table["instantiate"] = [&world](const std::string& key) -> sol::optional<ecs::Entity> {
        const ecs::Entity entity = scene::loadPrefab(world, project::Project::instance().getPath(key));
        return entity != ecs::INVALID_ENTITY ? sol::optional<ecs::Entity>(entity) : sol::nullopt;
    };

    // marked now, components live until frame ends
    table["destroy"] = [&world](ecs::Entity entity) {
        world.destroy(entity);
    };

    table["isAlive"] = [&world](ecs::Entity entity) {
        return world.isAlive(entity);
    };

    // component by reflected name, handle lets script fill it in
    table["add"] = [&world](ecs::Entity entity, const std::string& name) {
        return addComponent(world, entity, name);
    };

    // same reach as get, for entity script found rather than owns
    table["get"] = [&world](ecs::Entity entity, const std::string& name) -> Handle {
        const reflect::Type* type = findType(name);
        return type ? Handle{&world, entity, type} : Handle{};
    };

    // first entity carrying that name, nothing when none does
    table["find"] = [&world](const std::string& name) -> sol::optional<ecs::Entity> {
        for (ecs::Entity entity : world.getEntities())
        {
            const auto* identity = world.get<ecs::IdentityComponent>(entity);

            if (identity && identity->name == name)
            {
                return entity;
            }
        }

        return sol::nullopt;
    };

    // every entity sharing tag, nothing matches empty one
    table["findByTag"] = [&world](const std::string& tag, sol::this_state state) {
        sol::table found = sol::state_view(state).create_table();

        if (tag.empty())
        {
            return found;
        }

        for (ecs::Entity entity : world.getEntities())
        {
            const auto* identity = world.get<ecs::IdentityComponent>(entity);

            if (identity && identity->tag == tag)
            {
                found.add(entity);
            }
        }

        return found;
    };

    // every entity carrying component, by its reflected name
    table["findWith"] = [&world](const std::string& name, sol::this_state state) {
        sol::table found = sol::state_view(state).create_table();
        const reflect::Type* type = findType(name);

        if (!type)
        {
            return found;
        }

        for (ecs::Entity entity : world.getEntities())
        {
            if (world.has(entity, type->getIndex()))
            {
                found.add(entity);
            }
        }

        return found;
    };
}

} // namespace script
} // namespace BulletEngine
