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
namespace ecs { namespace systems { class PhysicsSystem; } }
namespace interface { class Editor; }

namespace script {

// registered type by name, complains once per name so script cannot flood console
const reflect::Type* findType(const std::string& name);

// what script reaches beyond its own components, one file per subject

void installComponents(sol::environment& environment, ecs::World& world, ecs::Entity entity);
void installInput(sol::environment& environment);
void installPhysics(sol::environment& environment, ecs::World& world, ecs::Entity entity, ecs::systems::PhysicsSystem& simulation);
void installWorld(sol::environment& environment, ecs::World& world);
void installScene(sol::environment& environment, interface::Editor& editor);

} // namespace script
} // namespace BulletEngine
