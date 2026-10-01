/*
 * Traits.h
 */

#pragma once

#include "reflect/Value.h"

#include "math/Quat.h"
#include "math/Vec3.h"

#include <string>
#include <type_traits>

namespace BulletEngine {
namespace reflect {

// tags

// maps c++ type to its tag and storage
template<class T>
struct Tag;

template<> struct Tag<bool>       { static constexpr ValueType TYPE = ValueType::Bool;   using Stored = bool; };
template<> struct Tag<int>        { static constexpr ValueType TYPE = ValueType::Int;    using Stored = int; };
template<> struct Tag<unsigned>   { static constexpr ValueType TYPE = ValueType::Int;    using Stored = int; };
template<> struct Tag<float>      { static constexpr ValueType TYPE = ValueType::Float;  using Stored = float; };
template<> struct Tag<double>     { static constexpr ValueType TYPE = ValueType::Float;  using Stored = float; };
template<> struct Tag<std::string>{ static constexpr ValueType TYPE = ValueType::String; using Stored = std::string; };
template<> struct Tag<glm::vec2>  { static constexpr ValueType TYPE = ValueType::Vec2;   using Stored = glm::vec2; };
template<> struct Tag<glm::ivec2> { static constexpr ValueType TYPE = ValueType::Vec2;   using Stored = glm::vec2; };
template<> struct Tag<glm::vec3>  { static constexpr ValueType TYPE = ValueType::Vec3;   using Stored = glm::vec3; };
template<> struct Tag<glm::vec4>  { static constexpr ValueType TYPE = ValueType::Vec4;   using Stored = glm::vec4; };
template<> struct Tag<glm::quat>  { static constexpr ValueType TYPE = ValueType::Quat;   using Stored = glm::quat; };

// physics carries doubles, editor and files keep floats
template<> struct Tag<BulletPhysics::math::Vec3> { static constexpr ValueType TYPE = ValueType::Vec3; using Stored = glm::vec3; };
template<> struct Tag<BulletPhysics::math::Quat> { static constexpr ValueType TYPE = ValueType::Quat; using Stored = glm::quat; };

// enums travel as int
template<class T>
struct EnumTag { static constexpr ValueType TYPE = ValueType::Int; using Stored = int; };

template<class T>
using TagOf = std::conditional_t<std::is_enum_v<T>, EnumTag<T>, Tag<T>>;

// strips reference and const
template<class T>
using Bare = std::remove_cv_t<std::remove_reference_t<T>>;

// conversion

// bridges storage and accessor types
template<class To, class From>
To convert(const From& value)
{
    if constexpr (std::is_same_v<To, From>)                                 return value;
    else if constexpr (std::is_same_v<To, glm::vec3> && std::is_same_v<From, BulletPhysics::math::Vec3>)
        return glm::vec3(static_cast<float>(value.x), static_cast<float>(value.y), static_cast<float>(value.z));
    else if constexpr (std::is_same_v<To, BulletPhysics::math::Vec3> && std::is_same_v<From, glm::vec3>)
        return BulletPhysics::math::Vec3(value.x, value.y, value.z);
    else if constexpr (std::is_same_v<To, glm::quat> && std::is_same_v<From, BulletPhysics::math::Quat>)
        return glm::quat(static_cast<float>(value.w), static_cast<float>(value.x), static_cast<float>(value.y), static_cast<float>(value.z));
    else if constexpr (std::is_same_v<To, BulletPhysics::math::Quat> && std::is_same_v<From, glm::quat>)
        return BulletPhysics::math::Quat(value.w, value.x, value.y, value.z);
    else                                                                    return static_cast<To>(value);
}

} // namespace reflect
} // namespace BulletEngine
