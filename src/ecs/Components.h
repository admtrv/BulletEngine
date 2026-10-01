/*
 * Components.h
 */

#pragma once

#include "assets/Handle.h"
#include "reflect/Annotations.h"
#include "ecs/Ecs.h"
#include "script/Script.h"

#include "scene/Camera.h"
#include "scene/Light.h"
#include "scene/Transform.h"
#include "scene/models/Model.h"
#include "Colors.h"
#include "render/Material.h"
#include "render/MaterialImport.h"
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
    [[= reflect::Skip{}]] BulletRender::scene::Transform transform;

    [[= reflect::Hidden{}]] Entity parent = INVALID_ENTITY;      // hierarchy is dragged, not typed as id
};

// named by asset key, so scene can be written down
class MaterialSlot {
public:
    const std::string& getTextureKey() const { return m_texture.getKey(); }
    void setTextureKey(const std::string& key);

    // sampling
    int getFilter() const { return int(m_slot.sampler.filter); }
    void setFilter(int filter) { m_slot.sampler.filter = BulletRender::render::TextureFilter(filter); }

    int getWrapU() const { return int(m_slot.sampler.wrapU); }
    void setWrapU(int wrap) { m_slot.sampler.wrapU = BulletRender::render::TextureWrap(wrap); }

    int getWrapV() const { return int(m_slot.sampler.wrapV); }
    void setWrapV(int wrap) { m_slot.sampler.wrapV = BulletRender::render::TextureWrap(wrap); }

    bool isFlippedU() const { return m_slot.sampler.flipU; }
    void setFlippedU(bool flipped) { m_slot.sampler.flipU = flipped; }

    bool isFlippedV() const { return m_slot.sampler.flipV; }
    void setFlippedV(bool flipped) { m_slot.sampler.flipV = flipped; }

    // window into the picture
    const glm::vec2& getTiling() const { return m_slot.uvScale; }
    void setTiling(const glm::vec2& tiling) { m_slot.uvScale = tiling; }

    const glm::vec2& getOffset() const { return m_slot.uvOffset; }
    void setOffset(const glm::vec2& offset) { m_slot.uvOffset = offset; }

    // what renderer and editor take
    bool isFilled() const { return !m_slot.empty(); }
    const BulletRender::render::TextureSlot& get() const { return m_slot; }
    const assets::Handle<BulletRender::render::Texture2D>& getTexture() const { return m_texture; }

private:
    assets::Handle<BulletRender::render::Texture2D> m_texture;
    BulletRender::render::TextureSlot m_slot;
};

struct ColorTerm {
    explicit ColorTerm(const glm::vec3& tint = glm::vec3(1.0f)) : color(tint) {}

    [[= reflect::Color{}]] glm::vec3 color;
    [[= reflect::Inline{}]] MaterialSlot texture;
};

struct SpecularTerm {
    [[= reflect::Color{}]] glm::vec3 color{0.5f};
    [[= reflect::Inline{}]] MaterialSlot texture;

    [[= reflect::Range{1.0f, 256.0f}]] float shininess = 32.0f;     // phong ns, higher draws tighter highlight
};

// picture with nothing to tint, detail alone
struct NormalTerm {
    [[= reflect::Inline{}]] MaterialSlot texture;
};

// alpha and which way faces are seen
struct SettingsTerm {
    BulletRender::render::AlphaMode alphaMode = BulletRender::render::AlphaMode::Opaque;

    [[= reflect::Range{0.0f, 1.0f}]] float alphaCutoff = 0.5f;   // mask only, below it pixel is dropped
    bool doubleSided = false;

    bool isMasked() const { return alphaMode == BulletRender::render::AlphaMode::Mask; }
};

class Material {
public:
    void importFrom(const std::string& mtlPath);    // once, from what model brought, never read again

    void fill(BulletRender::render::Material& material) const;      // what renderer draws with

    bool isLit() const { return shading == BulletRender::render::Shading::Lit; }

    BulletRender::render::Shading shading = BulletRender::render::Shading::Lit;

    // terms
    ColorTerm diffuse;
    SpecularTerm specular;
    NormalTerm normal;                      // light shapes it, unlit has no use for it
    ColorTerm emissive{glm::vec3(0.0f)};    // black gives off nothing, which is usual

    SettingsTerm settings;
};

class Renderable {
public:
    virtual ~Renderable() = default;

    virtual const assets::Handle<BulletRender::scene::Model>& getModel() const = 0;

    [[= reflect::Speed{0.01f}]] glm::vec3 origin{0.0f};     // point of model that sits where entity does, middle unless moved

    Material material;
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

    // whole picture, or one frame of a sheet cut into equal cells
    enum class Source : uint8_t {
        Single,
        Sheet
    };

    Sprite();

    const assets::Handle<BulletRender::scene::Model>& getModel() const override { return m_shape; }

    int getShape() const { return int(m_shapeType); }
    void setShape(int kind);

    int getSource() const { return int(m_source); }
    void setSource(int source);
    bool isSheet() const { return m_source == Source::Sheet; }

    // sheet, cells across and down, script walks frames to animate
    const glm::ivec2& getFrames() const { return m_frames; }
    void setFrames(const glm::ivec2& frames);

    int getFrame() const { return m_frame; }
    void setFrame(int frame);

    glm::vec2 getSheetSize() const;     // whole picture in pixels, zero until it arrives
    glm::vec2 getFrameSize() const;     // one cell of it, same thing when there is one cell

    void fitFrame();    // rebuilds shape when picture it is cut from changed size

    // uv window onto cell shown, whole picture when there is no sheet
    glm::vec2 getFrameScale() const;
    glm::vec2 getFrameOffset() const;

private:
    Shape m_shapeType = Shape::Quad;
    assets::Handle<BulletRender::scene::Model> m_shape;

    Source m_source = Source::Single;
    glm::ivec2 m_frames{1, 1};
    int m_frame = 0;

    // what shape was built for, picture may arrive long after frame was chosen
    glm::vec2 m_fitted{0.0f};
    Shape m_fittedShape = Shape::Quad;
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

    [[= reflect::Name{"near"}]] float nearPlane = 0.1f;
    [[= reflect::Name{"far"}]] float farPlane = 500.0f;

    bool main = false;          // one camera renders, first found when none is marked
};

class LightComponent : public Component {
public:
    std::shared_ptr<BulletRender::scene::Light> light;
};

class ScriptComponent : public Component {
public:
    [[= reflect::Skip{}]] assets::Handle<script::Script> script;

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

    // in order gl reads them
    enum Face {
        Right,
        Left,
        Top,
        Bottom,
        Front,
        Back,

        FACE_COUNT
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

    // faces
    const std::string& getRightKey() const { return m_faceKeys[Right]; }
    void setRightKey(const std::string& key) { setFaceKey(Right, key); }

    const std::string& getLeftKey() const { return m_faceKeys[Left]; }
    void setLeftKey(const std::string& key) { setFaceKey(Left, key); }

    const std::string& getTopKey() const { return m_faceKeys[Top]; }
    void setTopKey(const std::string& key) { setFaceKey(Top, key); }

    const std::string& getBottomKey() const { return m_faceKeys[Bottom]; }
    void setBottomKey(const std::string& key) { setFaceKey(Bottom, key); }

    const std::string& getFrontKey() const { return m_faceKeys[Front]; }
    void setFrontKey(const std::string& key) { setFaceKey(Front, key); }

    const std::string& getBackKey() const { return m_faceKeys[Back]; }
    void setBackKey(const std::string& key) { setFaceKey(Back, key); }

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

    std::string m_faceKeys[FACE_COUNT];
    std::shared_ptr<BulletRender::render::CubeMap> m_faces;
};

class CanvasComponent : public Component {
public:
    int order = 0;              // lower draws first, so higher ends up over it

    bool visible = true;
};

class RigidBodyComponent : public Component {
public:
    [[= reflect::Skip{}]] BulletPhysics::dynamics::RigidBody body;
};

class ColliderComponent : public Component {
public:
    [[= reflect::Name{"shape"}]] std::unique_ptr<BulletPhysics::collision::collider::Collider> collider;
};

} // namespace ecs
} // namespace BulletEngine
