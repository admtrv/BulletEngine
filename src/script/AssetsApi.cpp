/*
 * AssetsApi.cpp
 */

#include "Api.h"
#include "Binding.h"

#include "assets/Registry.h"

#include "render/textures/Texture2D.h"

#include <string>

namespace BulletEngine {
namespace script {

// global, so module a script requires reaches it too
void bindAssets(sol::state& lua)
{
    sol::table table = lua.create_named_table("assets");

    table["loadTexture"] = [](const std::string& key) {
        return ScriptTexture{assets::Registry::instance().load<BulletRender::render::Texture2D>(key)};
    };
}

} // namespace script
} // namespace BulletEngine
