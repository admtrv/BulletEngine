/*
 * ComponentApi.cpp
 */

#include "Api.h"
#include "Binding.h"

#include "reflect/Registry.h"

#include <iostream>

namespace BulletEngine {
namespace script {

void installComponents(sol::environment& environment, ecs::World& world, ecs::Entity entity)
{
    // components of entity, by name reflection registered
    environment["get"] = [&world, entity](const std::string& name) -> Handle {
        const reflect::Type* type = reflect::Registry::instance().find(name);

        if (!type)
        {
            std::cerr << "script asked for unknown component: " << name << '\n';
            return {};
        }

        return {&world, entity, type};
    };
}

} // namespace script
} // namespace BulletEngine
