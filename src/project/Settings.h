/*
 * Settings.h
 */

#pragma once

#include <cstdint>
#include <string>

namespace BulletEngine {
namespace project {

enum class Mode : uint8_t {
    Mode2D,
    Mode3D
};

inline const char* toString(Mode mode)
{
    return mode == Mode::Mode2D ? "2D" : "3D";
}

struct Settings {
    std::string name;
    std::string startScene;
    Mode mode = Mode::Mode3D;
};

bool writeSettings(const Settings& settings, const std::string& path);
bool readSettings(Settings& settings, const std::string& path);

} // namespace project
} // namespace BulletEngine
