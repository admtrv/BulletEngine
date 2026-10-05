/*
 * Registry.cpp
 */

#include "Registry.h"

namespace BulletEngine {
namespace assets {

Registry& Registry::instance()
{
    static Registry registry;
    return registry;
}

const Registry::Steps* Registry::loaderOf(std::type_index type) const
{
    const auto it = m_loaders.find(type);
    return it != m_loaders.end() ? &it->second : nullptr;
}

std::shared_ptr<void> Registry::find(std::type_index type, const std::string& key) const
{
    const auto kind = m_slots.find(type);

    if (kind == m_slots.end())
    {
        return nullptr;
    }

    const auto slot = kind->second.find(key);
    return slot != kind->second.end() ? slot->second.lock() : nullptr;
}

void Registry::remember(std::type_index type, const std::string& key, std::shared_ptr<void> slot)
{
    m_slots[type][key] = std::move(slot);
}

void Registry::forget(const std::string& key)
{
    for (auto& [type, slots] : m_slots)
    {
        slots.erase(key);
    }
}

Registry::Retained Registry::retainAll() const
{
    Retained held;

    for (const auto& [type, slots] : m_slots)
    {
        for (const auto& [key, slot] : slots)
        {
            if (std::shared_ptr<void> alive = slot.lock())
            {
                held.push_back(std::move(alive));
            }
        }
    }

    return held;
}

void Registry::clear()
{
    m_loaders.clear();
    m_slots.clear();
}

} // namespace assets
} // namespace BulletEngine
