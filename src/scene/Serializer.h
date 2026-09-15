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

// world as a tree, what a file is written from and read into
Node toNode(const ecs::World& world);
void fromNode(ecs::World& world, const Node& root);

bool save(const ecs::World& world, const std::string& path);
bool load(ecs::World& world, const std::string& path);

void saveObject(Node& node, const reflect::Type& type, const void* instance);
void loadObject(const Node& node, const reflect::Type& type, void* instance);

} // namespace scene
} // namespace BulletEngine
