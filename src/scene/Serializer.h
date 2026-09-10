/*
 * Serializer.h
 */

#pragma once

#include "scene/Archive.h"

#include <string>

namespace BulletEngine {

// fwd
namespace ecs { class World; }

namespace reflect { class Type; }

namespace scene {

bool save(const ecs::World& world, const std::string& path);
bool load(ecs::World& world, const std::string& path);

void saveObject(Node& node, const reflect::Type& type, const void* instance);
void loadObject(const Node& node, const reflect::Type& type, void* instance);

} // namespace scene
} // namespace BulletEngine
