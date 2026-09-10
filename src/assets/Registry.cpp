/*
 * Registry.cpp
 */

#include "Registry.h"

#include <iostream>

namespace BulletEngine {
namespace assets {

Registry& Registry::instance()
{
    static Registry registry;
    return registry;
}

std::shared_ptr<void> Registry::build(std::type_index type, const std::string& key) const
{
    const auto it = m_loaders.find(type);

    if (it == m_loaders.end())
    {
        std::cerr << "no loader for asset type: " << type.name() << '\n';
        return nullptr;
    }

    return it->second(key);
}

} // namespace assets
} // namespace BulletEngine
