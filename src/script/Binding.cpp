/*
 * Binding.cpp
 */

#include "Binding.h"

#include "reflect/Registry.h"

#include <glm/gtc/quaternion.hpp>

#include <iostream>

namespace BulletEngine {
namespace script {

static sol::object toLua(const reflect::Value& value, sol::state_view lua)
{
    switch (value.getType())
    {
        case reflect::ValueType::Bool:   return sol::make_object(lua, value.get<bool>());
        case reflect::ValueType::Int:    return sol::make_object(lua, value.get<int>());
        case reflect::ValueType::Float:  return sol::make_object(lua, value.get<float>());
        case reflect::ValueType::String: return sol::make_object(lua, value.get<std::string>());
        case reflect::ValueType::Vec2:   return sol::make_object(lua, value.get<glm::vec2>());
        case reflect::ValueType::Vec3:   return sol::make_object(lua, value.get<glm::vec3>());
        case reflect::ValueType::Vec4:   return sol::make_object(lua, value.get<glm::vec4>());
        case reflect::ValueType::Quat:   return sol::make_object(lua, value.get<glm::quat>());
        default:                         return sol::lua_nil;
    }
}

// field says what it holds, script has to bring that
static reflect::Value fromLua(const sol::object& object, reflect::ValueType type)
{
    switch (type)
    {
        case reflect::ValueType::Bool:   return object.as<bool>();
        case reflect::ValueType::Int:    return object.as<int>();
        case reflect::ValueType::Float:  return object.as<float>();
        case reflect::ValueType::String: return object.as<std::string>();
        case reflect::ValueType::Vec2:   return object.as<glm::vec2>();
        case reflect::ValueType::Vec3:   return object.as<glm::vec3>();
        case reflect::ValueType::Vec4:   return object.as<glm::vec4>();
        case reflect::ValueType::Quat:   return object.as<glm::quat>();
        default:                         return {};
    }
}

// looked up every access, entity may die while script holds handle
void* Handle::resolve() const
{
    if (!world || !type || !world->isAlive(entity))
    {
        return nullptr;
    }

    for (const std::unique_ptr<ecs::Component>& component : world->getComponents(entity))
    {
        if (std::type_index(typeid(*component)) == type->getIndex())
        {
            return component.get();
        }
    }

    return nullptr;
}

sol::object Handle::get(const std::string& name, sol::this_state state) const
{
    const reflect::Field* field = type ? type->findField(name) : nullptr;
    void* instance = resolve();

    if (!field || !instance)
    {
        return sol::lua_nil;
    }

    return toLua(field->get(instance), state);
}

void Handle::set(const std::string& name, const sol::object& value)
{
    const reflect::Field* field = type ? type->findField(name) : nullptr;
    void* instance = resolve();

    if (!field || !instance)
    {
        std::cerr << "script wrote to unknown field: " << name << '\n';
        return;
    }

    // object field holds a type, not a value, script names the one it wants
    if (field->getKind() == reflect::FieldKind::Object)
    {
        const std::string wanted = value.as<std::string>();
        const reflect::Type* built = reflect::Registry::instance().find(wanted);

        if (!built || !field->isBuildable())
        {
            std::cerr << "script cannot build " << wanted << " for field: " << name << '\n';
            return;
        }

        field->build(instance, *built);
        return;
    }

    field->set(instance, fromLua(value, field->getType()));
}

void bindTypes(sol::state& lua)
{
    lua.new_usertype<glm::vec2>("Vector2",
        sol::constructors<glm::vec2(), glm::vec2(float, float)>(),
        "x", &glm::vec2::x,
        "y", &glm::vec2::y);

    lua.new_usertype<glm::vec3>("Vector3",
        sol::constructors<glm::vec3(), glm::vec3(float, float, float)>(),
        "x", &glm::vec3::x,
        "y", &glm::vec3::y,
        "z", &glm::vec3::z);

    lua.new_usertype<glm::vec4>("Vector4",
        sol::constructors<glm::vec4(), glm::vec4(float, float, float, float)>(),
        "x", &glm::vec4::x,
        "y", &glm::vec4::y,
        "z", &glm::vec4::z,
        "w", &glm::vec4::w);

    lua.new_usertype<glm::quat>("Quaternion",
        sol::constructors<glm::quat(), glm::quat(float, float, float, float)>(),
        "w", &glm::quat::w,
        "x", &glm::quat::x,
        "y", &glm::quat::y,
        "z", &glm::quat::z);

    // every component reads and writes same way, reflection knows fields
    lua.new_usertype<Handle>("Component",
        sol::meta_function::index, &Handle::get,
        sol::meta_function::new_index, &Handle::set);
}

} // namespace script
} // namespace BulletEngine
