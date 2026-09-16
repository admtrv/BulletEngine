/*
 * Components.cpp
 */

#include "Components.h"

#include "assets/Registry.h"
#include "reflect/Reflect.h"

#include "collision/collider/BoxCollider.h"
#include "collision/collider/GroundCollider.h"
#include "collision/collider/SphereCollider.h"

using namespace BulletEngine::ecs;
using namespace BulletPhysics::collision;
using namespace BulletPhysics::collision::collider;
using namespace BulletRender::render;
using namespace BulletRender::scene;

// assets

void RenderableComponent::setModelKey(const std::string& key)
{
    if (key.empty())
    {
        model.reset();
        return;
    }

    // a key that loads nothing leaves what is already there
    if (auto loaded = BulletEngine::assets::Registry::instance().load<BulletRender::scene::Model>(key))
    {
        model = std::move(loaded);
    }
}

void RenderableComponent::setTextureKey(const std::string& key)
{
    if (key.empty())
    {
        texture.reset();
        material.clearTexture(ALBEDO_UNIFORM);
        return;
    }

    // a key that loads nothing leaves what is already there
    if (auto loaded = BulletEngine::assets::Registry::instance().load<BulletRender::render::Texture2D>(key))
    {
        texture = std::move(loaded);
        material.setTexture(ALBEDO_UNIFORM, texture.getShared(), ALBEDO_UNIT);
    }
}

void ScriptComponent::setScriptKey(const std::string& key)
{
    if (key.empty())
    {
        script.reset();
        return;
    }

    // a key that loads nothing leaves what is already there
    if (auto loaded = BulletEngine::assets::Registry::instance().load<BulletEngine::script::Script>(key))
    {
        script = std::move(loaded);
    }
}

// shapes

REFLECT(PhysicsMaterial)
    FIELD("friction", friction)
    FIELD("restitution", restitution)
END_REFLECT()

REFLECT(Collider)
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

REFLECT(GroundCollider)
    LABEL("Ground")
    BASE(Collider)
    PROPERTY("level", getGroundY, setGroundY)
END_REFLECT()

// lights, pose comes from the entity transform

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

REFLECT(NameComponent)
    BASE(Component)
    HIDE_TYPE()
    FIELD("name", name)
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

REFLECT(RenderableComponent)
    BASE(Component)
    NESTED("color", material, getColor, setColor)
    COLOR()
    PROPERTY("model", getModelKey, setModelKey)
    ASSET()
    PROPERTY("texture", getTextureKey, setTextureKey)
    ASSET()
END_REFLECT()

REFLECT(CameraComponent)
    BASE(Component)
    FIELD("fov", fov)
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

REFLECT(RigidBodyComponent)
    BASE(Component)
    NESTED("motion", body, getMotionType, setMotionType)
    OPTIONS("Dynamic", "Kinematic", "Static")
    NESTED("mass", body, getMass, setMass)
    NESTED("velocity", body, getVelocity, setVelocity)
    NESTED("angularVelocity", body, getAngularVelocity, setAngularVelocity)
    NESTED("linearDamping", body, getLinearDamping, setLinearDamping)
    NESTED("angularDamping", body, getAngularDamping, setAngularDamping)
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
