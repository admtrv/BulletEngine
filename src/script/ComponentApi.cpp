/*
 * ComponentApi.cpp
 */

#include "Api.h"
#include "Binding.h"

#include <string>

namespace BulletEngine {
namespace script {

void installComponents(sol::environment& environment, ecs::World& world, ecs::Entity entity)
{
    // components of entity, by name reflection registered
    environment["get"] = [&world, entity](const std::string& name) -> Handle {
        const reflect::Type* type = findType(name);
        return type ? Handle{&world, entity, type} : Handle{};
    };
}

} // namespace script
} // namespace BulletEngine
