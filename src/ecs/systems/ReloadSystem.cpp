/*
 * ReloadSystem.cpp
 */

#include "ReloadSystem.h"

#include "ecs/Components.h"
#include "project/Project.h"

#include "render/textures/TextureLoader.h"
#include "scene/models/ModelLoader.h"

#include <algorithm>

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

void ReloadSystem::reload(World& world, const std::vector<std::string>& keys)
{
    project::Project& project = project::Project::instance();

    // caches are keyed by path loader was given, not by key field holds
    for (const std::string& key : keys)
    {
        const std::string path = project.getPath(key);

        BulletRender::scene::ModelLoader::instance().remove(path);
        BulletRender::render::TextureLoader::instance().remove(path);
    }

    // key set back on itself pulls asset in again, past empty cache
    for (Entity entity : world.getEntities())
    {
        auto* renderable = world.get<RenderableComponent>(entity);

        if (!renderable)
        {
            continue;
        }

        // key is copied out, setting it replaces handle getter points into
        if (const std::string key = renderable->getModelKey(); std::find(keys.begin(), keys.end(), key) != keys.end())
        {
            renderable->setModelKey(key);
        }

        if (const std::string key = renderable->getTextureKey(); std::find(keys.begin(), keys.end(), key) != keys.end())
        {
            renderable->setTextureKey(key);
        }
    }
}

} // namespace systems
} // namespace ecs
} // namespace BulletEngine
