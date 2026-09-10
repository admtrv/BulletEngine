/*
 * Reflect.h
 */

#pragma once

#include "reflect/Registry.h"
#include "reflect/Type.h"
#include "reflect/Value.h"

#include "math/Vec3.h"
#include "math/Quat.h"

#include <type_traits>
#include <typeindex>

namespace BulletEngine {
namespace reflect {

// tags

// maps c++ type to its tag and storage
template<class T>
struct Tag;

template<> struct Tag<bool>       { static constexpr ValueType TYPE = ValueType::Bool;   using Stored = bool; };
template<> struct Tag<int>        { static constexpr ValueType TYPE = ValueType::Int;    using Stored = int; };
template<> struct Tag<float>      { static constexpr ValueType TYPE = ValueType::Float;  using Stored = float; };
template<> struct Tag<double>     { static constexpr ValueType TYPE = ValueType::Float;  using Stored = float; };
template<> struct Tag<std::string>{ static constexpr ValueType TYPE = ValueType::String; using Stored = std::string; };
template<> struct Tag<glm::vec2>  { static constexpr ValueType TYPE = ValueType::Vec2;   using Stored = glm::vec2; };
template<> struct Tag<glm::vec3>  { static constexpr ValueType TYPE = ValueType::Vec3;   using Stored = glm::vec3; };
template<> struct Tag<glm::vec4>  { static constexpr ValueType TYPE = ValueType::Vec4;   using Stored = glm::vec4; };
template<> struct Tag<glm::quat>  { static constexpr ValueType TYPE = ValueType::Quat;   using Stored = glm::quat; };

// physics carries doubles, editor and files keep floats
template<> struct Tag<BulletPhysics::math::Vec3> { static constexpr ValueType TYPE = ValueType::Vec3; using Stored = glm::vec3; };
template<> struct Tag<BulletPhysics::math::Quat> { static constexpr ValueType TYPE = ValueType::Quat; using Stored = glm::quat; };

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

// strips reference and const
template<class T>
using Bare = std::remove_cv_t<std::remove_reference_t<T>>;

// enums travel as int
template<class T>
struct EnumTag { static constexpr ValueType TYPE = ValueType::Int; using Stored = int; };

template<class T>
using TagOf = std::conditional_t<std::is_enum_v<T>, EnumTag<T>, Tag<T>>;

// field builders

// field from getter and setter
template<class C, class Getter, class Setter>
Field makeField(std::string name, Getter getter, Setter setter)
{
    using Raw = Bare<std::invoke_result_t<Getter, const C&>>;
    using Stored = typename TagOf<Raw>::Stored;

    return Field(
        std::move(name),
        TagOf<Raw>::TYPE,
        [getter](const void* instance) -> Value {
            return Value(convert<Stored>((static_cast<const C*>(instance)->*getter)()));
        },
        [setter](void* instance, const Value& value) {
            (static_cast<C*>(instance)->*setter)(convert<Raw>(value.get<Stored>()));
        }
    );
}

// read only field
template<class C, class Getter>
Field makeField(std::string name, Getter getter)
{
    using Raw = Bare<std::invoke_result_t<Getter, const C&>>;
    using Stored = typename TagOf<Raw>::Stored;

    return Field(
        std::move(name),
        TagOf<Raw>::TYPE,
        [getter](const void* instance) -> Value {
            return Value(convert<Stored>((static_cast<const C*>(instance)->*getter)()));
        },
        nullptr
    );
}

// object behind unique_ptr, concrete type resolved at runtime
template<class C, class P>
Field makeObjectField(std::string name, P C::* member)
{
    return Field(
        std::move(name),
        [member](const void* instance, const Type** outType) -> void* {
            const auto& pointer = static_cast<const C*>(instance)->*member;
            auto* object = pointer.get();

            if (outType)
            {
                *outType = object ? Registry::instance().find(std::type_index(typeid(*object))) : nullptr;
            }

            return const_cast<void*>(static_cast<const void*>(object));
        }
    );
}

// factory, skipped for abstract types
template<class T>
void attachFactory(Type& type)
{
    if constexpr (std::is_default_constructible_v<T> && !std::is_abstract_v<T>)
    {
        type.setFactory([]() -> void* { return new T(); });
    }
}

// object held by value
template<class C, class N>
Field makeValueObjectField(std::string name, N C::* member)
{
    return Field(
        std::move(name),
        [member](const void* instance, const Type** outType) -> void* {
            const N& nested = static_cast<const C*>(instance)->*member;

            if (outType)
            {
                *outType = Registry::instance().find(std::type_index(typeid(N)));
            }

            return const_cast<void*>(static_cast<const void*>(&nested));
        }
    );
}

// field reached through nested member
template<class C, class N, class Getter, class Setter>
Field makeNestedField(std::string name, N C::* member, Getter getter, Setter setter)
{
    using Raw = Bare<std::invoke_result_t<Getter, const N&>>;
    using Stored = typename TagOf<Raw>::Stored;

    return Field(
        std::move(name),
        TagOf<Raw>::TYPE,
        [member, getter](const void* instance) -> Value {
            const N& nested = static_cast<const C*>(instance)->*member;
            return Value(convert<Stored>((nested.*getter)()));
        },
        [member, setter](void* instance, const Value& value) {
            N& nested = static_cast<C*>(instance)->*member;
            (nested.*setter)(convert<Raw>(value.get<Stored>()));
        }
    );
}

// field from public data member
template<class C, class M>
Field makeMemberField(std::string name, M C::* member)
{
    using Raw = Bare<M>;
    using Stored = typename TagOf<Raw>::Stored;

    return Field(
        std::move(name),
        TagOf<Raw>::TYPE,
        [member](const void* instance) -> Value {
            return Value(convert<Stored>(static_cast<const C*>(instance)->*member));
        },
        [member](void* instance, const Value& value) {
            static_cast<C*>(instance)->*member = convert<Raw>(value.get<Stored>());
        }
    );
}

} // namespace reflect
} // namespace BulletEngine

// registration, placed in a .cpp next to the type
#define REFLECT(TYPE)                                                                   \
    namespace reflect_of_##TYPE {                                                       \
    struct Registrar {                                                                  \
        using Self = TYPE;                                                              \
        Registrar() {                                                                   \
            using namespace BulletEngine::reflect;                                      \
            Type& type = Registry::instance().add(std::type_index(typeid(Self)), #TYPE);\
            attachFactory<Self>(type);

#define FIELD(NAME, MEMBER)                                                             \
            type.addField(makeMemberField<Self>(NAME, &Self::MEMBER));

#define OBJECT(NAME, MEMBER)                                                            \
            type.addField(makeObjectField<Self>(NAME, &Self::MEMBER));

#define OBJECT_VALUE(NAME, MEMBER)                                                      \
            type.addField(makeValueObjectField<Self>(NAME, &Self::MEMBER));

// object behind a mutable getter
#define OBJECT_REF(NAME, GETTER)                                                        \
            type.addField(Field(NAME,                                                   \
                [](const void* instance, const Type** outType) -> void* {               \
                    auto* owner = const_cast<Self*>(static_cast<const Self*>(instance)); \
                    auto& nested = owner->GETTER();                                     \
                    using N = std::decay_t<decltype(nested)>;                           \
                    if (outType) *outType = Registry::instance().find(std::type_index(typeid(N))); \
                    return &nested;                                                     \
                }));

#define BASE(TYPE)                                                                      \
            type.setBase(Registry::instance().find(std::type_index(typeid(TYPE))));

#define PROPERTY(NAME, GETTER, SETTER)                                                  \
            type.addField(makeField<Self>(NAME, &Self::GETTER, &Self::SETTER));

#define READONLY(NAME, GETTER)                                                          \
            type.addField(makeField<Self>(NAME, &Self::GETTER));

#define NESTED(NAME, MEMBER, GETTER, SETTER)                                            \
            type.addField(makeNestedField<Self>(NAME, &Self::MEMBER,                    \
                &std::decay_t<decltype(std::declval<Self>().MEMBER)>::GETTER,           \
                &std::decay_t<decltype(std::declval<Self>().MEMBER)>::SETTER));

// same, for an overloaded setter
#define NESTED_AS(NAME, MEMBER, GETTER, SETTER, ARG)                                    \
            type.addField(makeNestedField<Self>(NAME, &Self::MEMBER,                    \
                &std::decay_t<decltype(std::declval<Self>().MEMBER)>::GETTER,           \
                static_cast<void (std::decay_t<decltype(std::declval<Self>().MEMBER)>::*)(ARG)>( \
                    &std::decay_t<decltype(std::declval<Self>().MEMBER)>::SETTER)));

#define END_REFLECT()                                                                   \
        }                                                                               \
    };                                                                                  \
    const Registrar g_registrar;                                                        \
    }
