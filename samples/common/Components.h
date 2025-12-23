/*
* Components.h
 */

#pragma once

// BulletEngine
#include "ecs/Ecs.h"

// BulletPhysics
#include "dynamics/PhysicsBody.h"
#include "math/Vec3.h"

#include <vector>

namespace BulletEngine {
namespace ecs {

class ProjectileRigidBodyComponent : public RigidBodyComponent {
public:
    ProjectileRigidBodyComponent() = default;
    explicit ProjectileRigidBodyComponent(const BulletPhysics::dynamics::projectile::ProjectileSpecs& specs) {
        // replace inherited body with projectile body polymorphically
        body = std::make_unique<BulletPhysics::dynamics::projectile::ProjectileRigidBody>(specs);
    }

    // helper to access body as projectile body
    BulletPhysics::dynamics::projectile::ProjectileRigidBody& getProjectileBody()
    {
        return static_cast<BulletPhysics::dynamics::projectile::ProjectileRigidBody&>(*body);
    }

    bool isGrounded = false;
};

class TrajectoryComponent : public Component {
public:
    TrajectoryComponent() = default;
    explicit TrajectoryComponent(const BulletPhysics::math::Vec3& color) : color(color) {}

    std::vector<BulletPhysics::math::Vec3> points;
    BulletPhysics::math::Vec3 color{1.0f, 1.0f, 1.0f};
    float minSegment = 0.02f;
};

} // namespace ecs
} // namespace BulletEngine
