/*
 * Loaders.h
 */

#pragma once

#include <string>

namespace BulletEngine {
namespace assets {

// primitives are spelled out in key itself, no file behind them
constexpr const char* BOX_PREFIX = "box:";
constexpr const char* SPHERE_PREFIX = "sphere:";

// what a preset spawns with, sizes follow in key
constexpr const char* BOX_KEY = "box:1,1,1";
constexpr const char* SPHERE_KEY = "sphere:0.5,32,16";

// teaches the registry to build engine asset types
void registerLoaders();

// asset key as the editor shows it, "box:1,1,1" -> "Box", a path -> its file name
std::string toLabel(const std::string& key);

} // namespace assets
} // namespace BulletEngine
