/*
 * Value.cpp
 */

#include "Value.h"

namespace BulletEngine {
namespace reflect {

const char* toString(ValueType type)
{
    switch (type)
    {
        case ValueType::Bool:   return "bool";
        case ValueType::Int:    return "int";
        case ValueType::Float:  return "float";
        case ValueType::String: return "string";
        case ValueType::Vec2:   return "vec2";
        case ValueType::Vec3:   return "vec3";
        case ValueType::Vec4:   return "vec4";
        case ValueType::Quat:   return "quat";
        default:                return "unknown";
    }
}

} // namespace reflect
} // namespace BulletEngine
