/*
 * Components.h
 */

#pragma once

#include "ecs/Ecs.h"

#include "scene/Transform.h"
#include "scene/Model.h"
#include "render/Material.h"

#include "dynamics/body/RigidBody.h"
#include "collision/collider/Collider.h"

#include <memory>

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
    BulletPhysics::dynamics::RigidBody body;
};

class ColliderComponent : public Component {
public:
    std::unique_ptr<BulletPhysics::collision::collider::Collider> collider;
};

} // namespace ecs
} // namespace BulletEngine
