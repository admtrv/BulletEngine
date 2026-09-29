/*
 * Components.cpp
 */

#include "Components.h"

#include "assets/Loaders.h"
#include "assets/Registry.h"
#include "project/Project.h"
#include "scene/Serializer.h"

#include <filesystem>
#include "reflect/Reflect.h"

#include <array>

#include "collision/collider/BoxCollider.h"
#include "collision/collider/CylinderCollider.h"
#include "collision/collider/GroundCollider.h"
#include "collision/collider/SphereCollider.h"

using namespace BulletEngine::ecs;
using namespace BulletPhysics::collision;
using namespace BulletPhysics::collision::collider;
using namespace BulletRender::render;
// scene namespace is left out, its Mesh is gpu geometry rather than what entity shows
using BulletRender::scene::Light;
using BulletRender::scene::AmbientLight;
using BulletRender::scene::DirectionalLight;
using BulletRender::scene::PointLight;
using BulletRender::scene::SpotLight;

// empty key clears slot, key that fails to load leaves what is there
template<class T>
static bool reload(BulletEngine::assets::Handle<T>& handle, const std::string& key)
{
    if (key.empty())
    {
        handle.reset();
        return true;
    }

    if (auto loaded = BulletEngine::assets::Registry::instance().load<T>(key))
    {
        handle = std::move(loaded);
        return true;
    }

    return false;
}

void ScriptComponent::setScriptKey(const std::string& key)
{
    reload(script, key);
}

// environment

void EnvironmentComponent::setLayout(int layout)
{
    m_layout = layout == int(Layout::Faces) ? Layout::Faces : Layout::Cross;
}

void EnvironmentComponent::setCrossKey(const std::string& key)
{
    reload(m_cross, key);
}

void EnvironmentComponent::setFaceKey(int face, const std::string& key)
{
    m_faceKeys[face] = key;

    buildFaces();
}

void EnvironmentComponent::buildFaces()
{
    std::array<std::string, FACE_COUNT> paths;

    for (int face = 0; face < FACE_COUNT; face++)
    {
        if (m_faceKeys[face].empty())
        {
            m_faces.reset();
            return;
        }

        paths[face] = project::Project::instance().getPath(m_faceKeys[face]);
    }

    m_faces = std::make_shared<BulletRender::render::CubeMap>(paths);
}

// sky covers whole view, what lies under it is never seen
glm::vec3 EnvironmentComponent::getClearColor() const
{
    return isSkybox() ? glm::vec3(0.0f) : m_color;
}

std::shared_ptr<BulletRender::render::CubeMap> EnvironmentComponent::getSkybox() const
{
    if (isCross())
    {
        return m_cross.getShared();
    }

    return isFaces() ? m_faces : nullptr;
}

// material

void MaterialSlot::setTextureKey(const std::string& key)
{
    if (reload(m_texture, key))
    {
        m_slot.texture = m_texture.getShared();
    }
}

// only what the file names is taken, an empty entry leaves its slot as it stands
void MaterialComponent::importFrom(const std::string& mtlPath)
{
    const project::Project& project = project::Project::instance();
    const std::vector<BulletRender::render::MaterialImport> imported = BulletRender::render::readMtl(mtlPath);

    if (imported.empty())
    {
        return;
    }

    const BulletRender::render::MaterialImport& source = imported.front();

    diffuse.color = source.diffuse;
    specular.color = source.specular;
    specular.shininess = source.shininess;
    emissive.color = source.emissive;

    const std::pair<MaterialSlot*, const std::string*> slots[SLOT_COUNT] = {
        {&diffuse.texture, &source.diffuseTexture},
        {&specular.texture, &source.specularTexture},
        {&normal.texture, &source.normalTexture},
        {&emissive.texture, &source.emissiveTexture}
    };

    for (const auto& [slot, path] : slots)
    {
        // file naming no texture says nothing about slot, it does not empty it
        if (!path->empty())
        {
            slot->setTextureKey(project.getKey(*path));
        }
    }
}

std::array<MaterialSlot*, MaterialComponent::SLOT_COUNT> MaterialComponent::getSlots()
{
    return {&diffuse.texture, &specular.texture, &normal.texture, &emissive.texture};
}

// what the component holds, handed over as the renderer reads it
void MaterialComponent::fill(BulletRender::render::Material& material) const
{
    material.shading = m_shading;

    material.diffuse = diffuse.color;
    material.diffuseTexture = diffuse.texture.get();

    material.specular = specular.color;
    material.specularTexture = specular.texture.get();
    material.shininess = specular.shininess;

    material.normalTexture = normal.texture.get();

    material.emissive = emissive.color;
    material.emissiveTexture = emissive.texture.get();

    material.alphaMode = settings.getAlphaModeValue();
    material.alphaCutoff = settings.alphaCutoff;
    material.doubleSided = settings.doubleSided;
}

// renderables

void Mesh::setModelKey(const std::string& key)
{
    if (!reload(m_model, key))
    {
        return;
    }

    // wavefront keeps the look beside the shape, taken once so the fields below hold it all
    std::filesystem::path path = project::Project::instance().getPath(key);

    if (path.extension() == ".obj")
    {
        material.importFrom(path.replace_extension(".mtl").string());
    }
}

Sprite::Sprite()
{
    // flat picture carries own shading, and holes in it are point of it
    material.setShading(int(BulletRender::render::Shading::Unlit));
    material.settings.setAlphaMode(int(BulletRender::render::AlphaMode::Blend));

    // quad carries model uv, a picture read over it hangs upside down without this
    material.diffuse.texture.setFlippedV(true);

    setShape(int(Shape::Quad));
}

void Sprite::setShape(int kind)
{
    m_shapeType = kind == int(Shape::Circle) ? Shape::Circle : Shape::Quad;

    fitFrame();
}

void Sprite::setSource(int source)
{
    m_source = source == int(Source::Sheet) ? Source::Sheet : Source::Single;

    fitFrame();
}

void Sprite::setFrames(const glm::ivec2& frames)
{
    m_frames = glm::max(frames, glm::ivec2(1));

    fitFrame();
}

void Sprite::setFrame(int frame)
{
    const int count = m_frames.x * m_frames.y;

    // reading runs along the sheet, so counting past its end wraps to the start
    m_frame = count > 0 ? ((frame % count) + count) % count : 0;

    fitFrame();
}

glm::vec2 Sprite::getSheetSize() const
{
    const assets::Handle<BulletRender::render::Texture2D>& texture = material.diffuse.texture.getTexture();

    return texture ? glm::vec2(float(texture->getWidth()), float(texture->getHeight())) : glm::vec2(0.0f);
}

glm::vec2 Sprite::getFrameSize() const
{
    const glm::vec2 cells = isSheet() ? glm::vec2(m_frames) : glm::vec2(1.0f);

    return getSheetSize() / cells;
}

glm::vec2 Sprite::getFrameScale() const
{
    return isSheet() ? glm::vec2(1.0f) / glm::vec2(m_frames) : glm::vec2(1.0f);
}

glm::vec2 Sprite::getFrameOffset() const
{
    if (!isSheet())
    {
        return glm::vec2(0.0f);
    }

    // sheet is read left to right, top down, while the flipped uv climbs from the bottom
    const glm::ivec2 cell{m_frame % m_frames.x, m_frames.y - 1 - m_frame / m_frames.x};

    return glm::vec2(cell) * getFrameScale();
}

// called every frame, so rebuilds nothing until what it is cut from moves
void Sprite::fitFrame()
{
    const glm::vec2 frame = getFrameSize();

    if (frame == m_fitted && m_shapeType == m_fittedShape && m_shape)
    {
        return;
    }

    m_fitted = frame;
    m_fittedShape = m_shapeType;

    // circle stays round whatever picture is, only quad takes its proportions
    if (m_shapeType == Shape::Circle)
    {
        m_shape = assets::Registry::instance().load<BulletRender::scene::Model>(assets::CIRCLE_KEY);
        return;
    }

    // picture may not have arrived yet, square stands in until it does
    if (frame.x <= 0.0f || frame.y <= 0.0f)
    {
        m_shape = assets::Registry::instance().load<BulletRender::scene::Model>(assets::QUAD_KEY);
        return;
    }

    // longer side stays one unit, so sprite takes room square one would
    const float longest = std::max(frame.x, frame.y);

    m_shape = assets::Registry::instance().load<BulletRender::scene::Model>(assets::quadKey(frame / longest));
}

// shapes

REFLECT(PhysicsMaterial)
    FIELD("friction", friction)
    FIELD("restitution", restitution)
END_REFLECT()

REFLECT(Collider)
    // where the shape sits on the entity, so it need not be centred or square to it
    PROPERTY("offset", getLocalPosition, setLocalPosition)
    PROPERTY("rotation", getLocalRotation, setLocalRotation)
    PROPERTY("trigger", isTrigger, setTrigger)
    // bits, what collider is and what it meets
    PROPERTY("layer", getLayer, setLayer)
    BITS()
    PROPERTY("mask", getMask, setMask)
    BITS()
    OBJECT_REF("material", getMaterial)
END_REFLECT()

REFLECT(BoxCollider)
    LABEL("Box")
    BASE(Collider)
    PROPERTY("size", getSize, setSize)
END_REFLECT()

REFLECT(SphereCollider)
    LABEL("Sphere")
    BASE(Collider)
    PROPERTY("radius", getRadius, setRadius)
END_REFLECT()

REFLECT(CylinderCollider)
    LABEL("Cylinder")
    BASE(Collider)
    PROPERTY("radius", getRadius, setRadius)
    PROPERTY("height", getHeight, setHeight)
END_REFLECT()

REFLECT(GroundCollider)
    LABEL("Ground")
    BASE(Collider)
    PROPERTY("level", getGroundY, setGroundY)
END_REFLECT()

// lights, pose comes from entity transform

REFLECT(Light)
    PROPERTY("color", getColor, setColor)
    COLOR()
    PROPERTY("intensity", getIntensity, setIntensity)
    PROPERTY("shadow", getCastsShadow, setCastsShadow)
END_REFLECT()

REFLECT(AmbientLight)
    LABEL("Ambient")
    BASE(Light)
END_REFLECT()

REFLECT(DirectionalLight)
    LABEL("Directional")
    BASE(Light)
END_REFLECT()

REFLECT(PointLight)
    LABEL("Point")
    BASE(Light)
    PROPERTY("range", getRange, setRange)
END_REFLECT()

REFLECT(SpotLight)
    LABEL("Spot")
    BASE(Light)
    PROPERTY("range", getRange, setRange)
END_REFLECT()

// components

REFLECT(Component)
    HIDE_TYPE()
END_REFLECT()

REFLECT(IdentityComponent)
    BASE(Component)
    HIDE_TYPE()
    FIELD("name", name)
    FIELD("tag", tag)
END_REFLECT()

REFLECT(TransformComponent)
    BASE(Component)
    FIELD("parent", parent)
    HIDE_FIELD()
    NESTED("position", transform, getPosition, setPosition)
    NESTED("rotation", transform, getRotation, setRotation)
    NESTED_AS("scale", transform, getLocalScale, setLocalScale, const glm::vec3&)
    SPEED(0.01f)
END_REFLECT()

// one picture, with how it is read and what part of it is taken
REFLECT(MaterialSlot)
    PROPERTY("texture", getTextureKey, setTextureKey)
    ASSET()
    PROPERTY("filter", getFilter, setFilter)
    OPTIONS("Smooth", "Pixel")
    SHOWN_WHEN(self.isFilled())
    PROPERTY("wrapU", getWrapU, setWrapU)
    OPTIONS("Repeat", "Clamp", "Mirror")
    SHOWN_WHEN(self.isFilled())
    PROPERTY("wrapV", getWrapV, setWrapV)
    OPTIONS("Repeat", "Clamp", "Mirror")
    SHOWN_WHEN(self.isFilled())
    PROPERTY("flipU", isFlippedU, setFlippedU)
    SHOWN_WHEN(self.isFilled())
    PROPERTY("flipV", isFlippedV, setFlippedV)
    SHOWN_WHEN(self.isFilled())
    PROPERTY("tiling", getTiling, setTiling)
    SHOWN_WHEN(self.isFilled())
    PROPERTY("offset", getOffset, setOffset)
    SHOWN_WHEN(self.isFilled())
END_REFLECT()

// the one place the look of a surface is answered, no file speaks for it
REFLECT(ColorTerm)
    FIELD("color", color)
    COLOR()
    OBJECT_VALUE("texture", texture)
    INLINE()
END_REFLECT()

REFLECT(SpecularTerm)
    FIELD("color", color)
    COLOR()
    OBJECT_VALUE("texture", texture)
    INLINE()
    FIELD("shininess", shininess)
    RANGE(1.0f, 256.0f)
END_REFLECT()

REFLECT(NormalTerm)
    OBJECT_VALUE("texture", texture)
    INLINE()
END_REFLECT()

REFLECT(SettingsTerm)
    PROPERTY("alphaMode", getAlphaMode, setAlphaMode)
    OPTIONS("Opaque", "Mask", "Blend")
    FIELD("alphaCutoff", alphaCutoff)
    RANGE(0.0f, 1.0f)
    SHOWN_WHEN(self.isMasked())
    FIELD("doubleSided", doubleSided)
END_REFLECT()

REFLECT(MaterialComponent)
    PROPERTY("shading", getShading, setShading)
    OPTIONS("Lit", "Unlit")
    OBJECT_VALUE("diffuse", diffuse)
    // light shapes surface, flat picture it passes by
    OBJECT_VALUE("specular", specular)
    SHOWN_WHEN(self.isLit())
    OBJECT_VALUE("normal", normal)
    SHOWN_WHEN(self.isLit())
    OBJECT_VALUE("emissive", emissive)
    OBJECT_VALUE("settings", settings)
END_REFLECT()

REFLECT(Renderable)
    FIELD("origin", origin)
    SPEED(0.01f)
    OBJECT_VALUE("material", material)
END_REFLECT()

REFLECT(Mesh)
    LABEL("Mesh")
    BASE(Renderable)
    PROPERTY("model", getModelKey, setModelKey)
    ASSET()
END_REFLECT()

REFLECT(Sprite)
    LABEL("Sprite")
    BASE(Renderable)
    // set when it is made, like cube never turns into sphere
    PROPERTY("shape", getShape, setShape)
    HIDE_FIELD()
    PROPERTY("source", getSource, setSource)
    OPTIONS("Single", "Sheet")
    PROPERTY("frames", getFrames, setFrames)
    SHOWN_WHEN(self.isSheet())
    PROPERTY("frame", getFrame, setFrame)
    SHOWN_WHEN(self.isSheet())
    READONLY("sheetSize", getSheetSize)
    HIDE_FIELD()
    READONLY("frameSize", getFrameSize)
    HIDE_FIELD()
END_REFLECT()

REFLECT(RenderableComponent)
    BASE(Component)
    OBJECT("renderable", renderable)
END_REFLECT()

REFLECT(CameraComponent)
    BASE(Component)
    FIELD("projection", projection)
    OPTIONS("Perspective", "Orthographic")
    FIELD("fov", fov)
    FIELD("height", height)
    FIELD("near", nearPlane)
    FIELD("far", farPlane)
    FIELD("main", main)
END_REFLECT()

REFLECT(LightComponent)
    BASE(Component)
    OBJECT("light", light)
END_REFLECT()

REFLECT(ScriptComponent)
    BASE(Component)
    PROPERTY("script", getScriptKey, setScriptKey)
    ASSET()
END_REFLECT()

REFLECT(EnvironmentComponent)
    BASE(Component)
    PROPERTY("background", getBackground, setBackground)
    OPTIONS("Color", "Skybox")
    PROPERTY("color", getColor, setColor)
    COLOR()
    SHOWN_WHEN(!self.isSkybox())
    PROPERTY("layout", getLayout, setLayout)
    OPTIONS("Cross", "Faces")
    SHOWN_WHEN(self.isSkybox())
    PROPERTY("texture", getCrossKey, setCrossKey)
    ASSET()
    SHOWN_WHEN(self.isCross())
    PROPERTY("right", getRightKey, setRightKey)
    ASSET()
    SHOWN_WHEN(self.isFaces())
    PROPERTY("left", getLeftKey, setLeftKey)
    ASSET()
    SHOWN_WHEN(self.isFaces())
    PROPERTY("top", getTopKey, setTopKey)
    ASSET()
    SHOWN_WHEN(self.isFaces())
    PROPERTY("bottom", getBottomKey, setBottomKey)
    ASSET()
    SHOWN_WHEN(self.isFaces())
    PROPERTY("front", getFrontKey, setFrontKey)
    ASSET()
    SHOWN_WHEN(self.isFaces())
    PROPERTY("back", getBackKey, setBackKey)
    ASSET()
    SHOWN_WHEN(self.isFaces())
END_REFLECT()

REFLECT(CanvasComponent)
    BASE(Component)
    FIELD("order", order)
    FIELD("visible", visible)
END_REFLECT()

REFLECT(RigidBodyComponent)
    BASE(Component)
    NESTED("motion", body, getMotionType, setMotionType)
    OPTIONS("Dynamic", "Kinematic", "Static")
    NESTED("mass", body, getMass, setMass)
    NESTED("velocity", body, getVelocity, setVelocity)
    NESTED("angularVelocity", body, getAngularVelocity, setAngularVelocity)
    NESTED("linearDamping", body, getLinearDamping, setLinearDamping)
    NESTED("angularDamping", body, getAngularDamping, setAngularDamping)
    NESTED("continuous", body, isContinuous, setContinuous)
    // axes body may not move or turn along
    FLAG("freezePositionX", body, getConstraints, setConstraints, BulletPhysics::dynamics::FREEZE_POSITION_X)
    AXES("Freeze Position")
    FLAG("freezePositionY", body, getConstraints, setConstraints, BulletPhysics::dynamics::FREEZE_POSITION_Y)
    FLAG("freezePositionZ", body, getConstraints, setConstraints, BulletPhysics::dynamics::FREEZE_POSITION_Z)
    FLAG("freezeRotationX", body, getConstraints, setConstraints, BulletPhysics::dynamics::FREEZE_ROTATION_X)
    AXES("Freeze Rotation")
    FLAG("freezeRotationY", body, getConstraints, setConstraints, BulletPhysics::dynamics::FREEZE_ROTATION_Y)
    FLAG("freezeRotationZ", body, getConstraints, setConstraints, BulletPhysics::dynamics::FREEZE_ROTATION_Z)
END_REFLECT()

REFLECT(ColliderComponent)
    BASE(Component)
    OBJECT("shape", collider)
END_REFLECT()
