/*
 * Components.h
 */

#pragma once

#include "Ecs.h"

#include "scene/Transform.h"
#include "scene/Model.h"
#include "render/Material.h"

#include "dynamics/RigidBody.h"
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
    BulletPhysic::dynamics::RigidBody body;
};

class TrajectoryComponent : public Component {
public:
    std::vector<BulletPhysic::math::Vec3> points;
    BulletPhysic::math::Vec3 color{1.0f, 1.0f, 1.0f};
    float minSegment = 0.02f;
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