/*
* Components.h
 */

#pragma once

// BulletEngine
#include "ecs/Ecs.h"

// BulletPhysic
#include "dynamics/PhysicsBody.h"
#include "math/Vec3.h"

#include <vector>

namespace BulletEngine {
namespace ecs {

class ProjectileRigidBodyComponent : public RigidBodyComponent {
public:
    ProjectileRigidBodyComponent() = default;
    explicit ProjectileRigidBodyComponent(const BulletPhysic::dynamics::projectile::ProjectileSpecs& specs) {
        // replace inherited body with projectile body polymorphically
        body = std::make_unique<BulletPhysic::dynamics::projectile::ProjectileRigidBody>(specs);
    }

    // helper to access body as projectile body
    BulletPhysic::dynamics::projectile::ProjectileRigidBody& getProjectileBody()
    {
        return static_cast<BulletPhysic::dynamics::projectile::ProjectileRigidBody&>(*body);
    }

    bool isGrounded = false;
};

class TrajectoryComponent : public Component {
public:
    TrajectoryComponent() = default;
    explicit TrajectoryComponent(const BulletPhysic::math::Vec3& color) : color(color) {}

    std::vector<BulletPhysic::math::Vec3> points;
    BulletPhysic::math::Vec3 color{1.0f, 1.0f, 1.0f};
    float minSegment = 0.02f;
};

} // namespace ecs
} // namespace BulletEngine
