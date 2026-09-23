/*
 * Settings.cpp
 */

#include "Settings.h"

#include "scene/Archive.h"
#include "Version.h"

namespace BulletEngine {
namespace project {

constexpr const char* VERSION_NODE = "version";
constexpr const char* NAME_NODE = "name";
constexpr const char* SCENE_NODE = "startScene";
constexpr const char* MODE_NODE = "mode";

bool writeSettings(const Settings& settings, const std::string& path)
{
    scene::Node root;

    root.add(VERSION_NODE).setValue(VERSION);
    root.add(NAME_NODE).setValue(settings.name);
    root.add(SCENE_NODE).setValue(settings.startScene);
    root.add(MODE_NODE).setValue(toString(settings.mode));

    return scene::write(root, path);
}

bool readSettings(Settings& settings, const std::string& path)
{
    scene::Node root;

    if (!scene::read(root, path))
    {
        return false;
    }

    if (const scene::Node* node = root.find(NAME_NODE))
    {
        settings.name = node->getValue();
    }

    if (const scene::Node* node = root.find(SCENE_NODE))
    {
        settings.startScene = node->getValue();
    }

    if (const scene::Node* node = root.find(MODE_NODE))
    {
        settings.mode = node->getValue() == toString(Mode::Mode2D) ? Mode::Mode2D : Mode::Mode3D;
    }

    return true;
}

} // namespace project
} // namespace BulletEngine
