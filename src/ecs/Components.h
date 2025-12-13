/*
 * Components.h
 */

#pragma once

#include "ecs/Ecs.h"

#include "scene/Transform.h"
#include "scene/Model.h"
#include "render/Material.h"

#include "dynamics/PhysicsBody.h"
#include "collision/Collider.h"
#include "math/Vec3.h"

#include <memory>
#include <vector>

namespace BulletEngine {
namespace ecs {

class TransformComponent : public Component {
public:
    BulletRender::scene::Transform transform;
};

class RenderableComponent : public Component {
public:
    BulletRender::scene::Model* model = nullptr;
    BulletRender::render::Material material;
};

class RigidBodyComponent : public Component {
public:
    RigidBodyComponent() : body(std::make_unique<BulletPhysic::dynamics::RigidBody>()) {}
    virtual ~RigidBodyComponent() = default;

    std::unique_ptr<BulletPhysic::dynamics::RigidBody> body;
};

class ColliderComponent : public Component {
public:
    std::shared_ptr<BulletPhysic::collision::Collider> collider;

    // debug visualization
    bool isVisible = false;
    BulletRender::scene::Model* model = nullptr;
    BulletRender::render::Material material;
};

} // namespace ecs
} // namespace BulletEngine
