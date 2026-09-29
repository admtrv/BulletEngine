/*
 * SceneApi.cpp
 */

#include "Api.h"

#include "interface/Editor.h"

#include <string>

namespace BulletEngine {
namespace script {

void installScene(sol::environment& environment, interface::Editor& editor)
{
    sol::table scene = environment.create_named("scene");

    // world is swapped between frames, so script that asked runs to its end
    scene["load"] = [&editor](const std::string& key) {
        editor.requestScene(key);
    };

    // what is playing, so one script may serve every scene it knows of
    scene["current"] = [&editor]() {
        return editor.getSceneKey();
    };
}

} // namespace script
} // namespace BulletEngine
