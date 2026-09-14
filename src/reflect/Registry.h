/*
 * Registry.h
 */

#pragma once

#include "reflect/Type.h"

#include <memory>
#include <string>
#include <string_view>
#include <typeindex>
#include <unordered_map>
#include <vector>

namespace BulletEngine {
namespace reflect {

// every registered type
class Registry {
public:
    static Registry& instance();

    Type& add(std::type_index index, std::string name);

    // lookup
    const Type* find(std::string_view name) const;
    const Type* find(std::type_index index) const;

    template<class T>
    const Type* find() const { return find(std::type_index(typeid(T))); }

    const std::vector<Type*>& getTypes() const { return m_order; }

    // types that derive from the given one and can be built
    std::vector<const Type*> getDerived(const Type& base) const;

private:
    Registry() = default;
    ~Registry() = default;

    Registry(const Registry&) = delete;
    Registry& operator=(const Registry&) = delete;

    std::unordered_map<std::string, std::unique_ptr<Type>> m_byName;
    std::unordered_map<std::type_index, Type*> m_byIndex;
    std::vector<Type*> m_order;     // registration order
};

} // namespace reflect
} // namespace BulletEngine
