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
#include "Colors.h"
#include "render/Material.h"
#include "render/textures/CubeMap.h"
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

// what stands behind scene
class EnvironmentComponent : public Component {
public:
    enum class Background : uint8_t {
        Color,
        Skybox
    };

    enum class Layout : uint8_t {
        Cross,
        Faces
    };

    // background
    int getBackground() const { return int(m_background); }
    void setBackground(int background) { m_background = Background(background); }

    const glm::vec3& getColor() const { return m_color; }
    void setColor(const glm::vec3& color) { m_color = color; }

    // sky, one image laid out as cross or six faces given one by one
    int getLayout() const { return int(m_layout); }
    void setLayout(int layout);

    const std::string& getCrossKey() const { return m_cross.getKey(); }
    void setCrossKey(const std::string& key);

    // faces, in the order gl reads them
    const std::string& getRightKey() const { return m_faceKeys[0]; }
    void setRightKey(const std::string& key) { setFaceKey(0, key); }

    const std::string& getLeftKey() const { return m_faceKeys[1]; }
    void setLeftKey(const std::string& key) { setFaceKey(1, key); }

    const std::string& getTopKey() const { return m_faceKeys[2]; }
    void setTopKey(const std::string& key) { setFaceKey(2, key); }

    const std::string& getBottomKey() const { return m_faceKeys[3]; }
    void setBottomKey(const std::string& key) { setFaceKey(3, key); }

    const std::string& getFrontKey() const { return m_faceKeys[4]; }
    void setFrontKey(const std::string& key) { setFaceKey(4, key); }

    const std::string& getBackKey() const { return m_faceKeys[5]; }
    void setBackKey(const std::string& key) { setFaceKey(5, key); }

    // what is chosen, editor asks before it draws a field
    bool isSkybox() const { return m_background == Background::Skybox; }
    bool isCross() const { return isSkybox() && m_layout == Layout::Cross; }
    bool isFaces() const { return isSkybox() && m_layout == Layout::Faces; }

    // what renderer takes
    glm::vec3 getClearColor() const;
    std::shared_ptr<BulletRender::render::CubeMap> getSkybox() const;

private:
    void setFaceKey(int face, const std::string& key);
    void buildFaces();      // set stands only whole, one face short draws nothing

    // choice
    Background m_background = Background::Color;
    Layout m_layout = Layout::Cross;

    // what each one holds
    glm::vec3 m_color = BulletRender::colors::Background;
    assets::Handle<BulletRender::render::CubeMap> m_cross;

    std::string m_faceKeys[6];
    std::shared_ptr<BulletRender::render::CubeMap> m_faces;
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
