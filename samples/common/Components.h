/*
* Components.h
 */

#pragma once

// BulletEngine
#include "ecs/Ecs.h"

// BulletPhysics
#include "builtin/bodies/RigidBody.h"
#include "math/Vec3.h"

#include <vector>

namespace BulletEngine {
namespace ecs {

class ProjectileRigidBodyComponent : public RigidBodyComponent {
public:
    ProjectileRigidBodyComponent() = default;
    explicit ProjectileRigidBodyComponent(const BulletPhysics::projectile::ProjectileSpecs& specs) {
        // replace inherited body with projectile body polymorphically
        body = std::make_unique<BulletPhysics::builtin::bodies::ProjectileRigidBody>(specs);
    }

    // helper to access body as projectile body
    BulletPhysics::builtin::bodies::ProjectileRigidBody& getProjectileBody()
    {
        return static_cast<BulletPhysics::builtin::bodies::ProjectileRigidBody&>(*body);
    }

    bool isGrounded = false;
};

class TrajectoryComponent : public Component {
public:
    TrajectoryComponent() = default;
    explicit TrajectoryComponent(const BulletPhysics::math::Vec3& color) : color(color) {}

    std::vector<BulletPhysics::math::Vec3> points;
    BulletPhysics::math::Vec3 color{1.0, 1.0, 1.0};
    double minSegment = 0.02;
};

} // namespace ecs
} // namespace BulletEngine
