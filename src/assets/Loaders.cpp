/*
 * Loaders.cpp
 */

#include "Loaders.h"

#include "assets/Registry.h"
#include "project/Project.h"

#include "render/textures/TextureLoader.h"
#include "scene/models/Model.h"
#include "scene/models/ModelLoader.h"

#include <cstdlib>
#include <string>
#include <vector>

namespace BulletEngine {
namespace assets {

static std::vector<float> parseNumbers(std::string_view text)
{
    std::vector<float> numbers;

    while (!text.empty())
    {
        const size_t comma = text.find(',');
        const std::string_view piece = text.substr(0, comma);

        numbers.push_back(std::strtof(std::string(piece).c_str(), nullptr));

        if (comma == std::string_view::npos)
        {
            break;
        }

        text.remove_prefix(comma + 1);
    }

    return numbers;
}

static bool startsWith(const std::string& key, const char* prefix)
{
    return key.rfind(prefix, 0) == 0;
}

static std::shared_ptr<BulletRender::scene::Model> loadModel(const std::string& key)
{
    if (startsWith(key, BOX_PREFIX))
    {
        const auto size = parseNumbers(std::string_view(key).substr(std::string_view(BOX_PREFIX).size()));

        return size.size() >= 3
            ? std::make_shared<BulletRender::scene::Box>(size[0], size[1], size[2])
            : std::make_shared<BulletRender::scene::Box>();
    }

    if (startsWith(key, SPHERE_PREFIX))
    {
        const auto args = parseNumbers(std::string_view(key).substr(std::string_view(SPHERE_PREFIX).size()));

        return args.size() >= 3
            ? std::make_shared<BulletRender::scene::Sphere>(args[0], static_cast<int>(args[1]), static_cast<int>(args[2]))
            : std::make_shared<BulletRender::scene::Sphere>();
    }

    return BulletRender::scene::ModelLoader::instance().load(project::Project::instance().getPath(key));
}

std::string toLabel(const std::string& key)
{
    if (key.empty())
    {
        return "None";
    }

    if (startsWith(key, BOX_PREFIX))
    {
        return "Box";
    }

    if (startsWith(key, SPHERE_PREFIX))
    {
        return "Sphere";
    }

    const size_t slash = key.find_last_of("/\\");
    return slash == std::string::npos ? key : key.substr(slash + 1);
}

void registerLoaders()
{
    Registry& registry = Registry::instance();

    registry.setLoader<BulletRender::scene::Model>(loadModel);

    registry.setLoader<BulletRender::render::Texture2D>([](const std::string& key) {
        return BulletRender::render::TextureLoader::instance().load(project::Project::instance().getPath(key));
    });
}

} // namespace assets
} // namespace BulletEngine
