/*
 * CanvasApi.cpp
 */

#include "Api.h"

#include "assets/Registry.h"
#include "render/passes/Canvas.h"
#include "render/Renderer.h"

#include <string>
#include <unordered_map>

namespace BulletEngine {
namespace script {

using BulletRender::render::Canvas;
using BulletRender::render::Font;

// fonts scripts named, held so their atlases outlive one frame
static std::unordered_map<std::string, assets::Handle<Font>> s_fonts;

static Font* findFont(const std::string& key)
{
    auto it = s_fonts.find(key);

    if (it == s_fonts.end())
    {
        it = s_fonts.emplace(key, assets::Registry::instance().load<Font>(key)).first;
    }

    return it->second.get();
}

void releaseFonts()
{
    s_fonts.clear();
}

// one usertype for whole state, so canvas handed to callback carries its methods
void bindCanvas(sol::state& lua)
{
    sol::usertype<Canvas> type = lua.new_usertype<Canvas>("Canvas", sol::no_constructor);

    type["rect"] = [](Canvas& canvas, float x, float y, float width, float height, float r, float g, float b, sol::optional<float> a) {
        canvas.addRect({x, y}, {width, height}, {r, g, b, a.value_or(1.0f)});
    };

    type["image"] = [](Canvas& canvas, const std::string& key, float x, float y, float width, float height, sol::optional<float> a) {
        const auto texture = assets::Registry::instance().load<BulletRender::render::Texture2D>(key);
        canvas.addImage(texture.get(), {x, y}, {width, height}, {1.0f, 1.0f, 1.0f, a.value_or(1.0f)});
    };

    type["text"] = [](Canvas& canvas, const std::string& key, const std::string& text, float x, float y, float size, float r, float g, float b, sol::optional<float> a) {
        if (Font* font = findFont(key))
        {
            canvas.addText(*font, text, {x, y}, size, {r, g, b, a.value_or(1.0f)});
        }
    };

    // what text would take, so script can place it itself
    type["measure"] = [](Canvas&, const std::string& key, const std::string& text, float size) {
        Font* font = findFont(key);
        const glm::vec2 measured = font ? Canvas::measureText(*font, text, size) : glm::vec2(0.0f);

        return std::make_tuple(measured.x, measured.y);
    };

    // panel being drawn into, what anchoring is measured against
    type["size"] = [](Canvas&) {
        const glm::ivec2 viewport = BulletRender::render::Renderer::getViewport();
        return std::make_tuple(float(viewport.x), float(viewport.y));
    };
}

} // namespace script
} // namespace BulletEngine
