/*
 * HierarchySystem.h
 */

#pragma once

#include "ecs/Ecs.h"

namespace BulletEngine {
namespace ecs {

// fwd
class TransformComponent;

namespace systems {

// keeps transform links in step with the parent each component names
class HierarchySystem {
public:
    void update(World& world);

    // refuses links that would close a cycle
    static bool canAttach(World& world, Entity child, Entity parent);
    static void attach(World& world, Entity child, Entity parent);

private:
    static TransformComponent* find(World& world, Entity entity);
};

} // namespace systems
} // namespace ecs
} // namespace BulletEngine
