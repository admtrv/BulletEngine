/*
 * Type.h
 */

#pragma once

#include "reflect/Value.h"

#include <functional>
#include <string>
#include <typeindex>
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
    using Query = std::function<bool(const void*)>;
    using Clear = std::function<void(void*)>;
    using Builder = std::function<void*(void*, const Type&)>;

    Field(std::string name, ValueType type, Getter getter, Setter setter)
        : m_name(std::move(name)), m_label(toLabel(m_name)), m_type(type), m_getter(std::move(getter)), m_setter(std::move(setter)) {}

    Field(std::string name, Resolver resolver, Builder builder = nullptr)
        : m_name(std::move(name)), m_label(toLabel(m_name)), m_kind(FieldKind::Object), m_resolver(std::move(resolver)), m_builder(std::move(builder)) {}

    // identity
    const std::string& getName() const { return m_name; }
    const std::string& getLabel() const { return m_label; }
    void setLabel(std::string label) { m_label = std::move(label); }
    FieldKind getKind() const { return m_kind; }
    ValueType getType() const { return m_type; }

    // editor hints
    const std::vector<std::string>& getOptions() const { return m_options; }
    void setOptions(std::vector<std::string> options) { m_options = std::move(options); }
    bool isEnum() const { return !m_options.empty(); }

    bool isHidden() const { return m_hidden; }
    void setHidden(bool hidden) { m_hidden = hidden; }

    bool isColor() const { return m_color; }        // channels rather than axes, drawn with a swatch
    void setColor(bool color) { m_color = color; }

    bool isAsset() const { return m_asset; }        // an asset key, drawn with a load button
    void setAsset(bool asset) { m_asset = asset; }

    bool isBits() const { return m_bits; }          // bit mask, drawn with a grid of checkboxes
    void setBits(bool bits) { m_bits = bits; }

    bool isAxes() const { return m_axes; }          // first of three, drawn as one row of x y z
    void setAxes(bool axes) { m_axes = axes; }

    // unset until someone asks for it, so whatever the asset brought stands
    bool isOptional() const { return m_has != nullptr; }
    bool has(const void* instance) const { return m_has && m_has(instance); }
    void clear(void* instance) const { if (m_clear) { m_clear(instance); } }
    void setOptional(Query has, Clear clear) { m_has = std::move(has); m_clear = std::move(clear); }

    float getSpeed() const { return m_speed; }      // how fast a drag walks the value, zero leaves it to the editor
    void setSpeed(float speed) { m_speed = speed; }

    // what the value may reach, equal bounds leave it to the editor
    float getMin() const { return m_min; }
    float getMax() const { return m_max; }
    bool hasRange() const { return m_min < m_max; }
    void setRange(float min, float max) { m_min = min; m_max = max; }

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
    bool m_hidden = false;
    bool m_color = false;
    bool m_asset = false;
    bool m_bits = false;
    bool m_axes = false;

    Query m_has;
    Clear m_clear;
    float m_speed = 0.0f;
    float m_min = 0.0f;
    float m_max = 0.0f;

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
