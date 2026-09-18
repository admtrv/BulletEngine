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
namespace script { class EventBus; }
namespace ecs { namespace systems { class PhysicsSystem; } }
namespace interface { class Editor; }

namespace script {

// registered type by name, complains once per name so script cannot flood console
const reflect::Type* findType(const std::string& name);

// canvas comes as an argument, so its type is bound once for whole state
void bindCanvas(sol::state& lua);

// fonts stay loaded while scripts run, this lets go of them
void releaseFonts();

// what script reaches beyond its own components, one file per subject

void installComponents(sol::environment& environment, ecs::World& world, ecs::Entity entity);
void installInput(sol::environment& environment);
void installEvents(sol::environment& environment, EventBus& events, ecs::Entity entity);

void installPhysics(sol::environment& environment, ecs::World& world, ecs::Entity entity, ecs::systems::PhysicsSystem& simulation);
void installWorld(sol::environment& environment, ecs::World& world);
void installScene(sol::environment& environment, interface::Editor& editor);

} // namespace script
} // namespace BulletEngine
