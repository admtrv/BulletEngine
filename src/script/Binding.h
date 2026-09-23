/*
 * Binding.h
 */

#pragma once

#include "ecs/Ecs.h"
#include "reflect/Type.h"
#include "reflect/Value.h"

#include <sol/sol.hpp>

#include <string>
#include <vector>

namespace BulletEngine {
namespace script {

// one component, held by entity, fields reached through reflection, object ones answer with handle of their own
struct Handle {
    ecs::World* world = nullptr;
    ecs::Entity entity = ecs::INVALID_ENTITY;
    const reflect::Type* type = nullptr;
    std::vector<std::string> path;      // field names leading from component down to what this points at

    sol::object get(const std::string& name, sol::this_state state) const;
    void set(const std::string& name, const sol::object& value);

private:
    void* resolve(const reflect::Type*& outType) const;
};

// what script sees of engine
void bindTypes(sol::state& lua);

} // namespace script
} // namespace BulletEngine
