/*
 * Components.h
 */

#pragma once

#include "Ecs.h"
#include "scene/Transform.h"
#include "scene/Model.h"
#include "render/Material.h"

#include "dynamics/RigidBody.h"

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
    BulletPhysic::dynamics::RigidBody body;
};

} // namespace ecs
} // namespace BulletEngine