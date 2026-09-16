/*
 * Api.h
 */

#pragma once

#include "ecs/Ecs.h"

#include <sol/sol.hpp>

#include <string>

namespace BulletEngine {

// fwd
namespace reflect { class Type; }

namespace script {

// registered type by name, complains once per name so script cannot flood console
const reflect::Type* findType(const std::string& name);

// what script reaches beyond its own components, one file per subject

void installComponents(sol::environment& environment, ecs::World& world, ecs::Entity entity);
void installInput(sol::environment& environment);
void installPhysics(sol::environment& environment, ecs::World& world, ecs::Entity entity);
void installWorld(sol::environment& environment, ecs::World& world);

} // namespace script
} // namespace BulletEngine
