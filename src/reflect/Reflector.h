/*
 * Reflector.h
 */

#pragma once

#include "reflect/Annotations.h"
#include "reflect/Registry.h"
#include "reflect/Traits.h"
#include "reflect/Type.h"

#include <meta>
#include <span>
#include <string>
#include <type_traits>
#include <typeindex>
#include <vector>

namespace BulletEngine {
namespace reflect {

// field builders

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
    Field field(
        std::move(name),
        [member](const void* instance, const Type** outType) -> void* {
            const auto& pointer = static_cast<const C*>(instance)->*member;
            auto* object = pointer.get();

            if (outType)
            {
                *outType = object ? Registry::instance().find(std::type_index(typeid(*object))) : nullptr;
            }

            return const_cast<void*>(static_cast<const void*>(object));
        },
        [member](void* instance, const Type& type) -> void* {
            auto& pointer = static_cast<C*>(instance)->*member;

            using Held = typename P::element_type;
            auto* made = static_cast<Held*>(type.create());

            pointer.reset(made);
            return made;
        }
    );

    field.setBaseType(Registry::instance().find(std::type_index(typeid(typename P::element_type))));
    return field;
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

// field reached through nested member, owner may inherit that member
template<class Owner, class C, class N, class Getter, class Setter>
Field makeNestedField(std::string name, N C::* member, Getter getter, Setter setter)
{
    using Raw = Bare<std::invoke_result_t<Getter, const N&>>;
    using Stored = typename TagOf<Raw>::Stored;

    return Field(
        std::move(name),
        TagOf<Raw>::TYPE,
        [member, getter](const void* instance) -> Value {
            const N& nested = static_cast<const Owner*>(instance)->*member;
            return Value(convert<Stored>((nested.*getter)()));
        },
        [member, setter](void* instance, const Value& value) {
            N& nested = static_cast<Owner*>(instance)->*member;
            (nested.*setter)(convert<Raw>(value.get<Stored>()));
        }
    );
}

// single bit of nested mask, reads and writes as bool
template<class C, class N, class Getter, class Setter, class Bits>
Field makeFlagField(std::string name, N C::* member, Getter getter, Setter setter, Bits bits)
{
    using Raw = Bare<std::invoke_result_t<Getter, const N&>>;

    return Field(
        std::move(name),
        ValueType::Bool,
        [member, getter, bits](const void* instance) -> Value {
            const N& nested = static_cast<const C*>(instance)->*member;
            return Value(((nested.*getter)() & bits) != 0);
        },
        [member, getter, setter, bits](void* instance, const Value& value) {
            N& nested = static_cast<C*>(instance)->*member;
            const Raw current = (nested.*getter)();

            (nested.*setter)(value.get<bool>() ? static_cast<Raw>(current | bits)
                                              : static_cast<Raw>(current & ~bits));
        }
    );
}

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

// annotations

template<class A, std::meta::info MEMBER>
consteval bool hasNote()
{
    return !std::meta::annotations_of_with_type(MEMBER, ^^A).empty();
}

// every annotation is carried over as it stands, nothing here knows what they mean
template<std::meta::info MEMBER>
void applyNotes(Field& field)
{
    template for (constexpr std::meta::info entry : std::define_static_array(std::meta::annotations_of(MEMBER)))
    {
        using A = typename[:std::meta::type_of(entry):];

        // caption is part of what field is called, rest only describes it
        if constexpr (std::is_same_v<A, Label>)
        {
            field.setLabel(std::meta::extract<A>(entry).text);
        }
        // Name and Skip shaped field before it existed
        else if constexpr (!std::is_same_v<A, Name> && !std::is_same_v<A, Skip>)
        {
            field.metadata(std::meta::extract<A>(entry));
        }
    }
}

// what type declares

// what Tag knows, so it reads as plain value
template<class M>
concept Scalar = requires { Tag<Bare<M>>::TYPE; };

// unique_ptr and such, what stands there is chosen at runtime
template<class M>
concept Pointer = requires { typename M::element_type; };

template<std::meta::info MEMBER>
consteval const char* nameOf()
{
    if constexpr (hasNote<Name, MEMBER>())
    {
        return std::meta::extract<Name>(std::meta::annotations_of_with_type(MEMBER, ^^Name)[0]).text;
    }
    else
    {
        return std::define_static_string(std::meta::identifier_of(MEMBER));
    }
}

// static, since neither vector nor string_view outlives consteval
template<class E>
consteval std::span<const char* const> optionsOf()
{
    std::vector<const char*> names;

    for (std::meta::info enumerator : std::meta::enumerators_of(^^E))
    {
        names.push_back(std::define_static_string(std::meta::identifier_of(enumerator)));
    }

    return std::define_static_array(names);
}

// what enum may hold, for member and for property that hands it out as int
template<class E>
Field& options(Field& field)
{
    constexpr std::span names = optionsOf<E>();
    return field.setOptions(std::vector<std::string>(names.begin(), names.end()));
}

template<class T, std::meta::info MEMBER>
Field makeAnyField()
{
    using M = typename[:std::meta::type_of(MEMBER):];

    if constexpr (Pointer<M>)
    {
        return makeObjectField<T>(nameOf<MEMBER>(), &[:MEMBER:]);
    }
    else if constexpr (Scalar<M> || std::is_enum_v<M>)
    {
        return makeMemberField<T>(nameOf<MEMBER>(), &[:MEMBER:]);
    }
    else
    {
        return makeValueObjectField<T>(nameOf<MEMBER>(), &[:MEMBER:]);
    }
}

template<class T, std::meta::info MEMBER>
void addMember(Type& type)
{
    using M = typename[:std::meta::type_of(MEMBER):];

    Field field = makeAnyField<T, MEMBER>();

    // names enum may take come from enum itself
    if constexpr (std::is_enum_v<M>)
    {
        options<M>(field);
    }

    applyNotes<MEMBER>(field);
    type.addField(std::move(field));
}

template<class T>
void describe(Type& type)
{
    constexpr auto context = std::meta::access_context::current();

    template for (constexpr std::meta::info member : std::define_static_array(std::meta::nonstatic_data_members_of(^^T, context)))
    {
        if constexpr (!hasNote<Skip, member>())
        {
            addMember<T, member>(type);
        }
    }
}

// base has to stand in registry already, fields are reached through it, not copied
template<class T>
void attachBase(Type& type)
{
    constexpr auto bases = std::define_static_array(std::meta::bases_of(^^T, std::meta::access_context::current()));

    if constexpr (!bases.empty())
    {
        using B = typename[:std::meta::type_of(bases[0]):];
        type.setBase(Registry::instance().find<B>());
    }
}

template<class T>
Type& reflect(const char* name = nullptr)
{
    Type& type = Registry::instance().add(std::type_index(typeid(T)), name ? name : std::define_static_string(std::meta::identifier_of(^^T)));

    attachFactory<T>(type);
    attachBase<T>(type);
    describe<T>(type);

    return type;
}

// what type cannot declare, spelled out for it

template<class T, class Getter, class Setter>
Field& property(Type& type, std::string name, Getter getter, Setter setter)
{
    type.addField(makeField<T>(std::move(name), getter, setter));

    return type.getLastField();
}

template<class T, class Getter>
Field& readOnly(Type& type, std::string name, Getter getter)
{
    type.addField(makeField<T>(std::move(name), getter));

    return type.getLastField();
}

// field of object component holds, that object shows no row of its own
template<class T, class N, class Getter, class Setter>
Field& nested(Type& type, std::string name, N T::* member, Getter getter, Setter setter)
{
    type.addField(makeNestedField<T>(std::move(name), member, getter, setter));

    return type.getLastField();
}

// one bit of mask that object keeps
template<class T, class N, class Getter, class Setter, class Mask>
Field& flag(Type& type, std::string name, N T::* member, Getter getter, Setter setter, Mask mask)
{
    type.addField(makeFlagField<T>(std::move(name), member, getter, setter, mask));

    return type.getLastField();
}

} // namespace reflect
} // namespace BulletEngine
