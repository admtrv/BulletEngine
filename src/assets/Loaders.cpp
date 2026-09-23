/*
 * Loaders.cpp
 */

#include "Loaders.h"

#include "assets/Registry.h"
#include "project/Project.h"
#include "script/Script.h"

#include "render/text/FontLoader.h"
#include "render/textures/TextureLoader.h"
#include "scene/models/Model.h"
#include "scene/models/ModelLoader.h"

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
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

static std::shared_ptr<BulletRender::scene::Model> loadModel(const std::string& key)
{
    const Primitive* primitive = findPrimitive(key);

    if (!primitive)
    {
        return BulletRender::scene::ModelLoader::instance().load(project::Project::instance().getPath(key));
    }

    const std::vector<float> args = parseNumbers(std::string_view(key).substr(std::string_view(primitive->prefix).size()));

    // key short of numbers still names shape, one of default size
    return args.size() >= primitive->arity ? primitive->build(args) : primitive->fallback();
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

    registry.setLoader<BulletRender::scene::Model>(loadModel);

    registry.setLoader<BulletRender::render::Texture2D>([](const std::string& key) {
        // image files run top down, gl reads bottom up
        BulletRender::render::TextureLoadOptions options;
        options.flipVertically = true;

        return BulletRender::render::TextureLoader::instance().load(project::Project::instance().getPath(key), options);
    });

    registry.setLoader<BulletRender::render::Font>([](const std::string& key) {
        return BulletRender::render::FontLoader::instance().load(project::Project::instance().getPath(key));
    });

    registry.setLoader<script::Script>(loadScript);
}

} // namespace assets
} // namespace BulletEngine
