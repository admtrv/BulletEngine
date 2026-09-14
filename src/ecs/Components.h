/*
 * Components.h
 */

#pragma once

#include "assets/Handle.h"
#include "ecs/Ecs.h"

#include "scene/Transform.h"
#include "scene/models/Model.h"
#include "render/Material.h"
#include "render/textures/Texture2D.h"

#include "dynamics/body/RigidBody.h"
#include "collision/collider/Collider.h"

#include <memory>
#include <string>

namespace BulletEngine {
namespace ecs {

class NameComponent : public Component {
public:
    std::string name = "Entity";
};

class TransformComponent : public Component {
public:
    BulletRender::scene::Transform transform;

    Entity parent = INVALID_ENTITY;
};

class RenderableComponent : public Component {
public:
    assets::Handle<BulletRender::scene::Model> model;
    assets::Handle<BulletRender::render::Texture2D> texture;
    BulletRender::render::Material material;

    // the key is what a scene file carries, the handle follows it
    const std::string& getModelKey() const { return model.getKey(); }
    void setModelKey(const std::string& key);

    const std::string& getTextureKey() const { return texture.getKey(); }
    void setTextureKey(const std::string& key);
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
