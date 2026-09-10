/*
 * Type.h
 */

#pragma once

#include "reflect/Value.h"

#include <functional>
#include <string>
#include <vector>

namespace BulletEngine {
namespace reflect {

class Type;

// what a field holds
enum class FieldKind : uint8_t {
    Value,
    Object
};

// named entry of a type
class Field {
public:
    using Getter = std::function<Value(const void*)>;
    using Setter = std::function<void(void*, const Value&)>;

    // resolves nested object and its concrete type
    using Resolver = std::function<void*(const void*, const Type**)>;

    Field(std::string name, ValueType type, Getter getter, Setter setter)
        : m_name(std::move(name)), m_type(type), m_getter(std::move(getter)), m_setter(std::move(setter)) {}

    Field(std::string name, Resolver resolver)
        : m_name(std::move(name)), m_kind(FieldKind::Object), m_resolver(std::move(resolver)) {}

    const std::string& getName() const { return m_name; }
    FieldKind getKind() const { return m_kind; }
    ValueType getType() const { return m_type; }

    // value access
    Value get(const void* instance) const { return m_getter ? m_getter(instance) : Value{}; }
    void set(void* instance, const Value& value) const { if (m_setter) m_setter(instance, value); }

    bool isReadOnly() const { return m_kind == FieldKind::Value && !m_setter; }

    // object access
    void* resolve(const void* instance, const Type** outType) const { return m_resolver ? m_resolver(instance, outType) : nullptr; }

private:
    std::string m_name;
    FieldKind m_kind = FieldKind::Value;
    ValueType m_type = ValueType::Bool;

    Getter m_getter;
    Setter m_setter;
    Resolver m_resolver;
};

// registered type
class Type {
public:
    using Factory = std::function<void*()>;

    explicit Type(std::string name) : m_name(std::move(name)) {}

    const std::string& getName() const { return m_name; }

    // fields
    void addField(Field field) { m_fields.push_back(std::move(field)); }
    const std::vector<Field>& getFields() const { return m_fields; }
    std::vector<const Field*> getAllFields() const;     // inherited first, then own
    const Field* findField(std::string_view name) const;

    // construction
    void* create() const { return m_factory ? m_factory() : nullptr; }
    void setFactory(Factory factory) { m_factory = std::move(factory); }

    // inheritance
    const Type* getBase() const { return m_base; }
    void setBase(const Type* base) { m_base = base; }

    bool derivesFrom(const Type& type) const;

private:
    std::string m_name;
    std::vector<Field> m_fields;
    Factory m_factory;

    const Type* m_base = nullptr;
};

} // namespace reflect
} // namespace BulletEngine
