/*
 * ReloadSystem.h
 */

#pragma once

#include "ecs/Ecs.h"

#include <string>
#include <vector>

namespace BulletEngine {
namespace ecs {
namespace systems {

// puts files edited outside editor back into world
class ReloadSystem {
public:
    void update(World& world, float dt);

private:
    static void reload(World& world, const std::vector<std::string>& keys);
};

} // namespace systems
} // namespace ecs
} // namespace BulletEngine
