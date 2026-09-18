/*
 * Components.h
 */

#pragma once

#include "assets/Handle.h"
#include "ecs/Ecs.h"
#include "script/Script.h"

#include "scene/Camera.h"
#include "scene/Light.h"
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

// what an entity is called and what group it belongs to
class IdentityComponent : public Component {
public:
    std::string name = "Entity";
    std::string tag;                // empty for an entity nothing looks for
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

// what the game looks through, entity transform places it
class CameraComponent : public Component {
public:
    BulletRender::scene::Projection projection = BulletRender::scene::Projection::Perspective;

    float fov = 60.0f;          // perspective only, vertical angle
    float height = 10.0f;       // orthographic only, world units view spans

    float nearPlane = 0.1f;
    float farPlane = 500.0f;

    // one camera renders, first found when none is marked
    bool main = false;
};

class LightComponent : public Component {
public:
    std::shared_ptr<BulletRender::scene::Light> light;
};

// behaviour of entity, one lua file
class ScriptComponent : public Component {
public:
    assets::Handle<script::Script> script;

    const std::string& getScriptKey() const { return script.getKey(); }
    void setScriptKey(const std::string& key);
};

// surface script draws interface on, entity script fills it every frame
class CanvasComponent : public Component {
public:
    // lower draws first, so higher ends up over it
    int order = 0;

    bool visible = true;
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
