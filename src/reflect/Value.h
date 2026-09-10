/*
 * Value.h
 */

#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <cstdint>
#include <string>
#include <variant>

namespace BulletEngine {
namespace reflect {

// kind of value
enum class ValueType : uint8_t {
    Bool,
    Int,
    Float,
    String,
    Vec2,
    Vec3,
    Vec4,
    Quat,

    Count
};

// tagged value carried between fields and readers
class Value {
public:
    Value() = default;

    // conversions
    Value(bool v) : m_data(v) {}
    Value(int v) : m_data(v) {}
    Value(float v) : m_data(v) {}
    Value(double v) : m_data(static_cast<float>(v)) {}
    Value(std::string v) : m_data(std::move(v)) {}
    Value(const char* v) : m_data(std::string(v)) {}
    Value(const glm::vec2& v) : m_data(v) {}
    Value(const glm::vec3& v) : m_data(v) {}
    Value(const glm::vec4& v) : m_data(v) {}
    Value(const glm::quat& v) : m_data(v) {}

    // access
    ValueType getType() const { return static_cast<ValueType>(m_data.index()); }

    template<class T>
    bool is() const { return std::holds_alternative<T>(m_data); }

    // falls back when tag does not match
    template<class T>
    T get(const T& fallback = T{}) const
    {
        const T* p = std::get_if<T>(&m_data);
        return p ? *p : fallback;
    }

private:
    using Storage = std::variant<bool, int, float, std::string, glm::vec2, glm::vec3, glm::vec4, glm::quat>;

    // alternative order mirrors ValueType
    static_assert(std::variant_size_v<Storage> == static_cast<size_t>(ValueType::Count));
    static_assert(std::is_same_v<std::variant_alternative_t<static_cast<size_t>(ValueType::Bool), Storage>, bool>);
    static_assert(std::is_same_v<std::variant_alternative_t<static_cast<size_t>(ValueType::Quat), Storage>, glm::quat>);

    Storage m_data;
};

const char* toString(ValueType type);

} // namespace reflect
} // namespace BulletEngine
