/*
 * Registry.cpp
 */

#include "Registry.h"

namespace BulletEngine {
namespace reflect {

Registry& Registry::instance()
{
    static Registry registry;
    return registry;
}

Type& Registry::add(std::type_index index, std::string name)
{
    // registering twice returns the same type
    if (const auto it = m_byIndex.find(index); it != m_byIndex.end())
    {
        return *it->second;
    }

    auto type = std::make_unique<Type>(std::move(name));
    Type* raw = type.get();

    m_byName.emplace(raw->getName(), std::move(type));
    m_byIndex.emplace(index, raw);
    m_order.push_back(raw);

    return *raw;
}

const Type* Registry::find(std::string_view name) const
{
    const auto it = m_byName.find(std::string(name));
    return it != m_byName.end() ? it->second.get() : nullptr;
}

const Type* Registry::find(std::type_index index) const
{
    const auto it = m_byIndex.find(index);
    return it != m_byIndex.end() ? it->second : nullptr;
}

} // namespace reflect
} // namespace BulletEngine
