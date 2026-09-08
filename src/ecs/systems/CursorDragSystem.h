/*
 * CursorDragSystem.h
 */

#pragma once

#include "ecs/systems/PhysicsSystem.h"

#include "scene/Camera.h"

#include "collision/Query.h"

namespace BulletEngine {
namespace ecs {
namespace systems {

// picks a body under the cursor and pulls it around
class CursorDragSystem {
public:
    CursorDragSystem(BulletRender::scene::Camera& camera, const PhysicsSystem& physics);

    void update();

    bool isDragging() const { return m_body != nullptr; }

    // how hard the body is pulled towards the cursor and how quickly it calms down
    void setStiffness(double stiffness) { m_stiffness = stiffness; }
    void setDamping(double damping) { m_damping = damping; }

private:
    BulletPhysics::collision::Ray rayThroughCursor(double cursorX, double cursorY) const;

    void grab(const BulletPhysics::collision::Ray& ray);
    void pull(const BulletPhysics::collision::Ray& ray);
    void release();

    BulletRender::scene::Camera& m_camera;
    const PhysicsSystem& m_physics;

    BulletPhysics::dynamics::RigidBody* m_body = nullptr;
    BulletPhysics::math::Vec3 m_localGrab{};   // where the body was grabbed, in its own frame
    double m_grabDistance = 0.0;               // how far along the ray the body hangs

    double m_stiffness = 40.0;
    double m_damping = 8.0;

    bool m_wasPressed = false;
};

} // namespace systems
} // namespace ecs
} // namespace BulletEngine
