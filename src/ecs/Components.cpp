/*
 * Components.cpp
 */

#include "Components.h"

#include "assets/Loaders.h"
#include "assets/Registry.h"
#include "project/Project.h"
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

// key that loads nothing leaves what is already there
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
    std::array<std::string, 6> paths;

    for (int face = 0; face < 6; face++)
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

// renderables

void Renderable::setTextureKey(const std::string& key)
{
    if (!reload(m_texture, key))
    {
        return;
    }

    m_texture ? material.setTexture(ALBEDO_UNIFORM, m_texture.getShared(), ALBEDO_UNIT) : material.clearTexture(ALBEDO_UNIFORM);

    onTextureChanged();
}

void Mesh::setModelKey(const std::string& key)
{
    reload(m_model, key);
}

Sprite::Sprite()
{
    material.setUnlit(true);
    setShape(int(Shape::Quad));
}

// only picture may have holes in it, plain colour fills its shape whole
void Sprite::onTextureChanged()
{
    material.setTransparent(!getTextureKey().empty());
}

void Sprite::setShape(int kind)
{
    m_kind = kind == int(Shape::Circle) ? Shape::Circle : Shape::Quad;
    m_shape = BulletEngine::assets::Registry::instance().load<BulletRender::scene::Model>(m_kind == Shape::Circle ? BulletEngine::assets::CIRCLE_KEY : BulletEngine::assets::QUAD_KEY);
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

REFLECT(Renderable)
    PROPERTY("texture", getTextureKey, setTextureKey)
    ASSET()
    // unset terms leave whatever model brought from its mtl
    NESTED("color", material, getColor, setColor)
    COLOR()
    OPTIONAL(material, hasColor, clearColor)
    NESTED("specular", material, getSpecular, setSpecular)
    COLOR()
    OPTIONAL(material, hasSpecular, clearSpecular)
    NESTED("emissive", material, getEmissive, setEmissive)
    COLOR()
    OPTIONAL(material, hasEmissive, clearEmissive)
    NESTED("shininess", material, getShininess, setShininess)
    RANGE(1.0f, 256.0f)
    OPTIONAL(material, hasShininess, clearShininess)
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
