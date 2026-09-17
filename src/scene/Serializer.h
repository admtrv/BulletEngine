/*
 * Serializer.h
 */

#pragma once

#include "ecs/Ecs.h"
#include "scene/Archive.h"

#include <string>

namespace BulletEngine {

// fwd
namespace reflect { class Type; }

namespace scene {

// one entity as tree, what prefabs and clones are made of
void saveEntity(Node& node, const ecs::World& world, ecs::Entity entity);
ecs::Entity loadEntity(ecs::World& world, const Node& node);

// world as a tree, what a file is written from and read into
Node toNode(const ecs::World& world);
void fromNode(ecs::World& world, const Node& root);

bool save(const ecs::World& world, const std::string& path);
bool load(ecs::World& world, const std::string& path);

// one entity on its own, what script spawns copies from
bool savePrefab(const ecs::World& world, ecs::Entity entity, const std::string& path);
ecs::Entity loadPrefab(ecs::World& world, const std::string& path);

// same entity again, built from what it holds right now
ecs::Entity clone(ecs::World& world, ecs::Entity entity);

void saveObject(Node& node, const reflect::Type& type, const void* instance);
void loadObject(const Node& node, const reflect::Type& type, void* instance);

} // namespace scene
} // namespace BulletEngine
