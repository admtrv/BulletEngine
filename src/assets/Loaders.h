/*
 * Loaders.h
 */

#pragma once

#include <string>

namespace BulletEngine {
namespace assets {

// teaches the registry to build engine asset types
void registerLoaders();

// asset key as the editor shows it, "box:1,1,1" -> "Box", a path -> its file name
std::string toLabel(const std::string& key);

} // namespace assets
} // namespace BulletEngine
