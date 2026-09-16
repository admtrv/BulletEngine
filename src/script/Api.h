/*
 * Api.h
 */

#pragma once

#include "ecs/Ecs.h"

#include <sol/sol.hpp>

namespace BulletEngine {
namespace script {

// what a script reaches beyond its own components, one file per subject

void installComponents(sol::environment& environment, ecs::World& world, ecs::Entity entity);
void installInput(sol::environment& environment);
void installPhysics(sol::environment& environment, ecs::World& world, ecs::Entity entity);
void installWorld(sol::environment& environment, ecs::World& world);

} // namespace script
} // namespace BulletEngine
