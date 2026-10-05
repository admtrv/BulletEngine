/*
 * ReloadSystem.cpp
 */

#include "ReloadSystem.h"

#include "assets/Registry.h"
#include "project/Project.h"
#include "reflect/Annotations.h"
#include "reflect/Registry.h"

#include <algorithm>
#include <typeindex>

namespace BulletEngine {
namespace ecs {
namespace systems {

void ReloadSystem::update(World& world, float dt)
{
    const std::vector<std::string> keys = project::Project::instance().poll(dt);

    if (!keys.empty())
    {
        reload(world, keys);
    }
}

// asset fields say so themselves, new one needs no word here
void ReloadSystem::reloadObject(const reflect::Type& type, void* instance, const std::vector<std::string>& keys)
{
    for (const reflect::Field* field : type.getAllFields())
    {
        if (field->getKind() == reflect::FieldKind::Object)
        {
            const reflect::Type* nested = nullptr;

            if (void* object = field->resolve(instance, &nested); object && nested)
            {
                reloadObject(*nested, object, keys);
            }

            continue;
        }

        if (!field->has<reflect::Asset>())
        {
            continue;
        }

        // key set back on itself pulls asset in again, past empty cache
        const std::string key = field->get(instance).get<std::string>();

        if (std::find(keys.begin(), keys.end(), key) != keys.end())
        {
            field->set(instance, reflect::Value(key));
        }
    }
}

void ReloadSystem::reload(World& world, const std::vector<std::string>& keys)
{
    const reflect::Registry& registry = reflect::Registry::instance();

    // slot stays until it is forgotten, so a key set back on itself would find the old asset
    for (const std::string& key : keys)
    {
        assets::Registry::instance().forget(key);
    }

    for (Entity entity : world.getEntities())
    {
        for (const std::unique_ptr<Component>& component : world.getComponents(entity))
        {
            if (const reflect::Type* type = registry.find(std::type_index(typeid(*component))))
            {
                reloadObject(*type, component.get(), keys);
            }
        }
    }
}

} // namespace systems
} // namespace ecs
} // namespace BulletEngine
