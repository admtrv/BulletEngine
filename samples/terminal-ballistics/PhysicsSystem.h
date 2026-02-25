/*
 * PhysicsSystem.h
 */

#pragma once

#include "ecs/systems/PhysicsSystem.h"
#include "collision/BoxCollider.h"
#include "common/Components.h"

namespace BulletEngine {
namespace ecs {
namespace systems {

class PhysicsSystem : public PhysicsSystemBase {
public:
    using PhysicsSystemBase::PhysicsSystemBase;

protected:
    bool beforeIntegrate(World& world, Entity entity, RigidBodyComponent& rigidBody, float dt) override;
    void afterIntegrate(World& world, Entity entity, RigidBodyComponent& rigidBody, float dt) override;
};

} // namespace systems
} // namespace ecs
} // namespace BulletEngine
