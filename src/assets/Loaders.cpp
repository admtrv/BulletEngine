/*
 * Loaders.cpp
 */

#include "Loaders.h"

#include "assets/Registry.h"
#include "project/Project.h"
#include "script/Script.h"

#include "render/text/FontLoader.h"
#include "render/textures/CubeMapLoader.h"
#include "render/textures/TextureLoader.h"
#include "scene/models/Model.h"
#include "scene/models/ModelLoader.h"

#include <any>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <string_view>
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

// shape key spells out, built from numbers following its prefix
struct Primitive {
    const char* prefix;
    const char* label;
    size_t arity;       // how many numbers it takes, shorter key gets default size instead

    using Model = std::shared_ptr<BulletRender::scene::Model>;

    Model (*build)(const std::vector<float>& args);
    Model (*fallback)();
};

template<class Shape>
static Primitive::Model makeDefault()
{
    return std::make_shared<Shape>();
}

static const Primitive PRIMITIVES[] = {
    {BOX_PREFIX, "Box", 3, [](const std::vector<float>& a) -> Primitive::Model {
        return std::make_shared<BulletRender::scene::Box>(a[0], a[1], a[2]);
    }, makeDefault<BulletRender::scene::Box>},

    {SPHERE_PREFIX, "Sphere", 3, [](const std::vector<float>& a) -> Primitive::Model {
        return std::make_shared<BulletRender::scene::Sphere>(a[0], static_cast<int>(a[1]), static_cast<int>(a[2]));
    }, makeDefault<BulletRender::scene::Sphere>},

    {QUAD_PREFIX, "Quad", 2, [](const std::vector<float>& a) -> Primitive::Model {
        return std::make_shared<BulletRender::scene::Quad>(a[0], a[1]);
    }, makeDefault<BulletRender::scene::Quad>},

    {CIRCLE_PREFIX, "Circle", 2, [](const std::vector<float>& a) -> Primitive::Model {
        return std::make_shared<BulletRender::scene::Circle>(a[0], static_cast<int>(a[1]));
    }, makeDefault<BulletRender::scene::Circle>}
};

// primitive key names, nothing when it points at file instead
static const Primitive* findPrimitive(const std::string& key)
{
    for (const Primitive& primitive : PRIMITIVES)
    {
        if (startsWith(key, primitive.prefix))
        {
            return &primitive;
        }
    }

    return nullptr;
}

static std::vector<BulletRender::scene::MeshData> readModel(const std::string& source)
{
    // primitive is spelled out in key, nothing to read for it
    if (findPrimitive(source))
    {
        return {};
    }

    return BulletRender::scene::ModelLoader::read(source);
}

static std::shared_ptr<BulletRender::scene::Model> buildModel(const std::string& path, const std::vector<BulletRender::scene::MeshData>& meshes)
{
    const Primitive* primitive = findPrimitive(path);

    // primitive is spelled out in key itself, file one is named by where it sits in project
    if (!primitive)
    {
        return BulletRender::scene::ModelLoader::upload(meshes, project::Project::instance().getKey(path));
    }

    const std::vector<float> args = parseNumbers(std::string_view(path).substr(std::string_view(primitive->prefix).size()));

    // key short of numbers still names shape, one of default size
    const Primitive::Model model = args.size() >= primitive->arity ? primitive->build(args) : primitive->fallback();
    model->setName(path);

    return model;
}

// capture names asset by where it sits in project, worker only knows disk path
template<class T>
static T& toKey(T& pixels)
{
    pixels.path = project::Project::instance().getKey(pixels.path);

    return pixels;
}

// nothing read means slot failed
static std::shared_ptr<BulletRender::render::TexturePixels> keepPixels(BulletRender::render::TexturePixels& pixels)
{
    if (pixels.empty())
    {
        return nullptr;
    }

    return std::make_shared<BulletRender::render::TexturePixels>(std::move(pixels));
}

static std::shared_ptr<script::Script> loadScript(const std::string& key)
{
    std::ifstream file(project::Project::instance().getPath(key));

    if (!file)
    {
        std::cerr << "script load failed: " << key << '\n';
        return nullptr;
    }

    std::ostringstream buffer;
    buffer << file.rdbuf();

    return std::make_shared<script::Script>(script::Script{std::move(buffer).str()});
}

// trimmed, so proportions that come out the same share one model
std::string quadKey(const glm::vec2& size)
{
    char key[64];
    std::snprintf(key, sizeof(key), "%s%.4f,%.4f", QUAD_PREFIX, size.x, size.y);

    return key;
}

std::string toLabel(const std::string& key)
{
    if (key.empty())
    {
        return "None";
    }

    if (const Primitive* primitive = findPrimitive(key))
    {
        return primitive->label;
    }

    const size_t slash = key.find_last_of("/\\");
    return slash == std::string::npos ? key : key.substr(slash + 1);
}

void registerLoaders()
{
    Registry& registry = Registry::instance();

    registry.setResolver([](const std::string& key) {
        return findPrimitive(key) ? key : project::Project::instance().getPath(key);
    });

    registry.setLoader<BulletRender::scene::Model>(
        [](const std::string& key) { return std::any(std::make_pair(key, readModel(key))); },
        [](std::any& ready) {
            auto& [path, meshes] = std::any_cast<std::pair<std::string, std::vector<BulletRender::scene::MeshData>>&>(ready);
            return buildModel(path, meshes);
        });

    registry.setLoader<BulletRender::render::Texture2D>(
        [](const std::string& path) { return std::any(BulletRender::render::TextureLoader::read(path)); },
        [](std::any& ready) { return BulletRender::render::TextureLoader::upload(toKey(std::any_cast<BulletRender::render::TexturePixels&>(ready))); });

    // face stays pixels, gl sees it only as part of set
    registry.setLoader<BulletRender::render::TexturePixels>(
        [](const std::string& path) { return std::any(BulletRender::render::TextureLoader::read(path)); },
        [](std::any& ready) { return keepPixels(toKey(std::any_cast<BulletRender::render::TexturePixels&>(ready))); });

    registry.setLoader<BulletRender::render::Font>(
        [](const std::string& path) { return std::any(BulletRender::render::FontLoader::read(path)); },
        [](std::any& ready) { return BulletRender::render::FontLoader::upload(std::move(std::any_cast<std::vector<unsigned char>&>(ready))); });

    // faces given one by one are their own assets, so this key names cross
    registry.setLoader<BulletRender::render::CubeMap>(
        [](const std::string& path) { return std::any(BulletRender::render::CubeMapLoader::readCross(path)); },
        [](std::any& ready) { return BulletRender::render::CubeMapLoader::upload(toKey(std::any_cast<BulletRender::render::CubeMapPixels&>(ready))); });

    registry.setLoader<script::Script>(loadScript);
}

} // namespace assets
} // namespace BulletEngine
