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

#include <cstdint>
#include <memory>
#include <string>

namespace BulletEngine {
namespace ecs {

// what entity is called and what group it belongs to
class IdentityComponent : public Component {
public:
    std::string name = "Entity";
    std::string tag;                // empty for entity nothing looks for
};

class TransformComponent : public Component {
public:
    BulletRender::scene::Transform transform;

    Entity parent = INVALID_ENTITY;
};

class Renderable {
public:
    virtual ~Renderable() = default;

    virtual const assets::Handle<BulletRender::scene::Model>& getModel() const = 0;

    const std::string& getTextureKey() const { return m_texture.getKey(); }
    void setTextureKey(const std::string& key);

    BulletRender::render::Material material;

protected:
    virtual void onTextureChanged() {}

    assets::Handle<BulletRender::render::Texture2D> m_texture;
};

class Mesh : public Renderable {
public:
    const assets::Handle<BulletRender::scene::Model>& getModel() const override { return m_model; }

    const std::string& getModelKey() const { return m_model.getKey(); }
    void setModelKey(const std::string& key);

private:
    assets::Handle<BulletRender::scene::Model> m_model;
};

class Sprite : public Renderable {
public:
    enum class Shape : uint8_t {
        Quad,
        Circle
    };

    Sprite();

    const assets::Handle<BulletRender::scene::Model>& getModel() const override { return m_shape; }

    int getShape() const { return int(m_kind); }
    void setShape(int kind);

protected:
    void onTextureChanged() override;

private:
    Shape m_kind = Shape::Quad;
    assets::Handle<BulletRender::scene::Model> m_shape;
};

class RenderableComponent : public Component {
public:
    std::unique_ptr<Renderable> renderable;
};

class CameraComponent : public Component {
public:
    BulletRender::scene::Projection projection = BulletRender::scene::Projection::Perspective;

    float fov = 60.0f;          // perspective only, vertical angle
    float height = 10.0f;       // orthographic only, world units view spans

    float nearPlane = 0.1f;
    float farPlane = 500.0f;

    bool main = false;          // one camera renders, first found when none is marked
};

class LightComponent : public Component {
public:
    std::shared_ptr<BulletRender::scene::Light> light;
};

class ScriptComponent : public Component {
public:
    assets::Handle<script::Script> script;

    const std::string& getScriptKey() const { return script.getKey(); }
    void setScriptKey(const std::string& key);
};

class CanvasComponent : public Component {
public:
    int order = 0;              // lower draws first, so higher ends up over it

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
