/*
 * Type.h
 */

#pragma once

#include "reflect/Value.h"

#include <functional>
#include <memory>
#include <string>
#include <typeindex>
#include <unordered_map>
#include <vector>

namespace BulletEngine {
namespace reflect {

class Type;

// camelCase name to a label the editor shows
std::string toLabel(std::string_view name);

// what a field holds
enum class FieldKind : uint8_t {
    Value,
    Object
};

// named entry of a type
class Field {
public:
    // accessors
    using Getter = std::function<Value(const void*)>;
    using Setter = std::function<void(void*, const Value&)>;
    using Resolver = std::function<void*(const void*, const Type**)>;
    using Builder = std::function<void*(void*, const Type&)>;

    Field(std::string name, ValueType type, Getter getter, Setter setter)
        : m_name(std::move(name)), m_label(toLabel(m_name)), m_type(type), m_getter(std::move(getter)), m_setter(std::move(setter)) {}

    Field(std::string name, Resolver resolver, Builder builder = nullptr)
        : m_name(std::move(name)), m_label(toLabel(m_name)), m_kind(FieldKind::Object), m_resolver(std::move(resolver)), m_builder(std::move(builder)) {}

    // identity
    const std::string& getName() const { return m_name; }
    const std::string& getLabel() const { return m_label; }
    Field& setLabel(std::string label) { m_label = std::move(label); return *this; }
    FieldKind getKind() const { return m_kind; }
    ValueType getType() const { return m_type; }

    const std::vector<std::string>& getOptions() const { return m_options; }
    Field& setOptions(std::vector<std::string> options) { m_options = std::move(options); return *this; }
    bool isEnum() const { return !m_options.empty(); }

    // asked before editor draws field
    using Condition = std::function<bool(const void*)>;

    Field& setCondition(Condition condition) { m_condition = std::move(condition); return *this; }
    bool isShown(const void* instance) const { return !m_condition || m_condition(instance); }

    // metadata, whatever field was told about itself, never read here
    template<class T>
    Field& metadata(T value)
    {
        m_metadata[std::type_index(typeid(T))] = std::make_shared<T>(std::move(value));
        return *this;
    }

    template<class T>
    const T* metadata() const
    {
        const auto it = m_metadata.find(std::type_index(typeid(T)));
        return it != m_metadata.end() ? static_cast<const T*>(it->second.get()) : nullptr;
    }

    template<class T>
    bool has() const { return m_metadata.contains(std::type_index(typeid(T))); }

    // value
    Value get(const void* instance) const { return m_getter ? m_getter(instance) : Value{}; }
    void set(void* instance, const Value& value) const { if (m_setter) m_setter(instance, value); }
    bool isReadOnly() const { return m_kind == FieldKind::Value && !m_setter; }

    // object
    void* resolve(const void* instance, const Type** outType) const { return m_resolver ? m_resolver(instance, outType) : nullptr; }
    void* build(void* instance, const Type& type) const { return m_builder ? m_builder(instance, type) : nullptr; }
    bool isBuildable() const { return m_builder != nullptr; }

    const Type* getBaseType() const { return m_baseType; }      // what the pointer is declared as
    void setBaseType(const Type* type) { m_baseType = type; }

private:
    std::string m_name;
    std::string m_label;
    FieldKind m_kind = FieldKind::Value;
    ValueType m_type = ValueType::Bool;

    std::vector<std::string> m_options;
    Condition m_condition;

    std::unordered_map<std::type_index, std::shared_ptr<const void>> m_metadata;

    Getter m_getter;
    Setter m_setter;
    Resolver m_resolver;
    Builder m_builder;
    const Type* m_baseType = nullptr;
};

// registered type
class Type {
public:
    using Factory = std::function<void*()>;

    Type(std::type_index index, std::string name) : m_index(index), m_name(std::move(name)), m_label(toLabel(m_name)) {}

    std::type_index getIndex() const { return m_index; }
    const std::string& getName() const { return m_name; }

    const std::string& getLabel() const { return m_label; }
    void setLabel(std::string label) { m_label = std::move(label); }

    // fields
    void addField(Field field) { m_fields.push_back(std::move(field)); }
    const std::vector<Field>& getFields() const { return m_fields; }
    Field& getLastField() { return m_fields.back(); }
    std::vector<const Field*> getAllFields() const;     // own first, then inherited
    const Field* findField(std::string_view name) const;
    Field& field(std::string_view name);                // what reflection already made, to say more about it

    // construction
    void* create() const { return m_factory ? m_factory() : nullptr; }
    bool isCreatable() const { return m_factory != nullptr; }
    void setFactory(Factory factory) { m_factory = std::move(factory); }

    // kept out of the editor, still saved to file
    bool isHidden() const { return m_hidden; }
    void setHidden(bool hidden) { m_hidden = hidden; }

    // inheritance
    const Type* getBase() const { return m_base; }
    void setBase(const Type* base) { m_base = base; }

    bool derivesFrom(const Type& type) const;

private:
    std::type_index m_index;
    std::string m_name;
    std::string m_label;
    std::vector<Field> m_fields;
    bool m_hidden = false;
    Factory m_factory;

    const Type* m_base = nullptr;
};

} // namespace reflect
} // namespace BulletEngine
