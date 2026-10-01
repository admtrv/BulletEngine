/*
 * Reflection.cpp
 */

#include "ecs/Reflection.h"

#include "ecs/Components.h"
#include "reflect/Reflector.h"

#include "collision/collider/BoxCollider.h"
#include "collision/collider/CylinderCollider.h"
#include "collision/collider/GroundCollider.h"
#include "collision/collider/SphereCollider.h"

namespace BulletEngine {
namespace ecs {

using reflect::Field;
using reflect::Type;
using reflect::flag;
using reflect::nested;
using reflect::property;
using reflect::options;
using reflect::readOnly;

namespace collision = BulletPhysics::collision;
namespace collider = BulletPhysics::collision::collider;
namespace physics = BulletPhysics::dynamics;
namespace render = BulletRender::render;
namespace view = BulletRender::scene;

using Body = physics::RigidBody;
using Env = EnvironmentComponent;
using Transform = view::Transform;

// setLocalScale is overloaded, vector one is meant
using ScaleSetter = void (Transform::*)(const glm::vec3&);

// helpers

// object getter hands out, edited where it stands
template<class T, class Getter>
static void reference(Type& type, std::string name, Getter getter, const Type& held)
{
    type.addField(Field(std::move(name),
        [getter, &held](const void* instance, const Type** outType) -> void* {
            T* owner = const_cast<T*>(static_cast<const T*>(instance));

            if (outType)
            {
                *outType = &held;
            }

            return &(owner->*getter)();
        }));
}

template<class T, class Query>
static reflect::Field::Condition when(Query query)
{
    return [query](const void* instance) { return (static_cast<const T*>(instance)->*query)(); };
}

template<class T, class Query>
static reflect::Field::Condition whenNot(Query query)
{
    return [query](const void* instance) { return !(static_cast<const T*>(instance)->*query)(); };
}

// submodules

// nothing may be written on type this project does not own
static void registerShapes()
{
    const Type& material = reflect::reflect<collision::PhysicsMaterial>("PhysicsMaterial");

    Type& base = reflect::reflect<collider::Collider>("Collider");

    // where shape sits on entity, it need not be centred on it
    property<collider::Collider>(base, "offset", &collider::Collider::getLocalPosition, &collider::Collider::setLocalPosition);
    property<collider::Collider>(base, "rotation", &collider::Collider::getLocalRotation, &collider::Collider::setLocalRotation);
    property<collider::Collider>(base, "trigger", &collider::Collider::isTrigger, &collider::Collider::setTrigger);

    // what collider is, and what it meets
    property<collider::Collider>(base, "layer", &collider::Collider::getLayer, &collider::Collider::setLayer).metadata(reflect::Bits{});
    property<collider::Collider>(base, "mask", &collider::Collider::getMask, &collider::Collider::setMask).metadata(reflect::Bits{});

    reference<collider::Collider>(base, "material", static_cast<collision::PhysicsMaterial& (collider::Collider::*)()>(&collider::Collider::getMaterial), material);

    Type& box = reflect::reflect<collider::BoxCollider>("BoxCollider");
    box.setLabel("Box");
    property<collider::BoxCollider>(box, "size", &collider::BoxCollider::getSize, &collider::BoxCollider::setSize);

    Type& sphere = reflect::reflect<collider::SphereCollider>("SphereCollider");
    sphere.setLabel("Sphere");
    property<collider::SphereCollider>(sphere, "radius", &collider::SphereCollider::getRadius, &collider::SphereCollider::setRadius);

    Type& cylinder = reflect::reflect<collider::CylinderCollider>("CylinderCollider");
    cylinder.setLabel("Cylinder");
    property<collider::CylinderCollider>(cylinder, "radius", &collider::CylinderCollider::getRadius, &collider::CylinderCollider::setRadius);
    property<collider::CylinderCollider>(cylinder, "height", &collider::CylinderCollider::getHeight, &collider::CylinderCollider::setHeight);

    Type& ground = reflect::reflect<collider::GroundCollider>("GroundCollider");
    ground.setLabel("Ground");
    property<collider::GroundCollider>(ground, "level", &collider::GroundCollider::getGroundY, &collider::GroundCollider::setGroundY);
}

static void registerLights()
{
    Type& base = reflect::reflect<view::Light>("Light");

    property<view::Light>(base, "color", &view::Light::getColor, &view::Light::setColor).metadata(reflect::Color{});
    property<view::Light>(base, "intensity", &view::Light::getIntensity, &view::Light::setIntensity);
    property<view::Light>(base, "shadow", &view::Light::getCastsShadow, &view::Light::setCastsShadow);

    reflect::reflect<view::AmbientLight>("AmbientLight").setLabel("Ambient");
    reflect::reflect<view::DirectionalLight>("DirectionalLight").setLabel("Directional");

    Type& point = reflect::reflect<view::PointLight>("PointLight");
    point.setLabel("Point");
    property<view::PointLight>(point, "range", &view::PointLight::getRange, &view::PointLight::setRange);

    Type& spot = reflect::reflect<view::SpotLight>("SpotLight");
    spot.setLabel("Spot");
    property<view::SpotLight>(spot, "range", &view::SpotLight::getRange, &view::SpotLight::setRange);
}

// engine types, where only private state needs saying

static void registerMaterial()
{
    const Field::Condition filled = when<MaterialSlot>(&MaterialSlot::isFilled);

    Type& slot = reflect::reflect<MaterialSlot>("MaterialSlot");
    property<MaterialSlot>(slot, "texture", &MaterialSlot::getTextureKey, &MaterialSlot::setTextureKey).metadata(reflect::Asset{});
    options<render::TextureFilter>(property<MaterialSlot>(slot, "filter", &MaterialSlot::getFilter, &MaterialSlot::setFilter)).setCondition(filled);
    options<render::TextureWrap>(property<MaterialSlot>(slot, "wrapU", &MaterialSlot::getWrapU, &MaterialSlot::setWrapU)).setCondition(filled);
    options<render::TextureWrap>(property<MaterialSlot>(slot, "wrapV", &MaterialSlot::getWrapV, &MaterialSlot::setWrapV)).setCondition(filled);
    property<MaterialSlot>(slot, "flipU", &MaterialSlot::isFlippedU, &MaterialSlot::setFlippedU).setCondition(filled);
    property<MaterialSlot>(slot, "flipV", &MaterialSlot::isFlippedV, &MaterialSlot::setFlippedV).setCondition(filled);
    property<MaterialSlot>(slot, "tiling", &MaterialSlot::getTiling, &MaterialSlot::setTiling).setCondition(filled);
    property<MaterialSlot>(slot, "offset", &MaterialSlot::getOffset, &MaterialSlot::setOffset).setCondition(filled);

    reflect::reflect<ColorTerm>("ColorTerm");
    reflect::reflect<SpecularTerm>("SpecularTerm");
    reflect::reflect<NormalTerm>("NormalTerm");

    // cutoff is what mask cuts at, other modes have nothing to cut
    reflect::reflect<SettingsTerm>("SettingsTerm").field("alphaCutoff").setCondition(when<SettingsTerm>(&SettingsTerm::isMasked));

    // light shapes surface, flat picture passes it by
    const Field::Condition lit = when<Material>(&Material::isLit);

    Type& material = reflect::reflect<Material>("Material");
    material.field("specular").setCondition(lit);
    material.field("normal").setCondition(lit);
}

static void registerRenderables()
{
    const Field::Condition sheet = when<Sprite>(&Sprite::isSheet);

    reflect::reflect<Renderable>("Renderable");

    Type& mesh = reflect::reflect<Mesh>("Mesh");
    mesh.setLabel("Mesh");
    property<Mesh>(mesh, "model", &Mesh::getModelKey, &Mesh::setModelKey).metadata(reflect::Asset{});

    Type& sprite = reflect::reflect<Sprite>("Sprite");
    sprite.setLabel("Sprite");

    // set when sprite is made, cube never turns into sphere
    property<Sprite>(sprite, "shape", &Sprite::getShape, &Sprite::setShape).metadata(reflect::Hidden{});
    options<Sprite::Source>(property<Sprite>(sprite, "source", &Sprite::getSource, &Sprite::setSource));
    property<Sprite>(sprite, "frames", &Sprite::getFrames, &Sprite::setFrames).setCondition(sheet);
    property<Sprite>(sprite, "frame", &Sprite::getFrame, &Sprite::setFrame).setCondition(sheet);

    // what script measures placement against
    readOnly<Sprite>(sprite, "sheetSize", &Sprite::getSheetSize).metadata(reflect::Hidden{});
    readOnly<Sprite>(sprite, "frameSize", &Sprite::getFrameSize).metadata(reflect::Hidden{});
}

static void registerTransforms()
{
    Type& type = reflect::reflect<TransformComponent>("TransformComponent");

    nested(type, "position", &TransformComponent::transform, &Transform::getPosition, &Transform::setPosition);
    nested(type, "rotation", &TransformComponent::transform, &Transform::getRotation, &Transform::setRotation);
    nested(type, "scale", &TransformComponent::transform, &Transform::getLocalScale, static_cast<ScaleSetter>(&Transform::setLocalScale)).metadata(reflect::Speed{0.01f});
}

static void registerBodies()
{
    Type& type = reflect::reflect<RigidBodyComponent>("RigidBodyComponent");

    options<physics::MotionType>(nested(type, "motion", &RigidBodyComponent::body, &Body::getMotionType, &Body::setMotionType));
    nested(type, "mass", &RigidBodyComponent::body, &Body::getMass, &Body::setMass);
    nested(type, "velocity", &RigidBodyComponent::body, &Body::getVelocity, &Body::setVelocity);
    nested(type, "angularVelocity", &RigidBodyComponent::body, &Body::getAngularVelocity, &Body::setAngularVelocity);
    nested(type, "linearDamping", &RigidBodyComponent::body, &Body::getLinearDamping, &Body::setLinearDamping);
    nested(type, "angularDamping", &RigidBodyComponent::body, &Body::getAngularDamping, &Body::setAngularDamping);
    nested(type, "continuous", &RigidBodyComponent::body, &Body::isContinuous, &Body::setContinuous);

    // axes body may not move or turn along
    flag(type, "freezePositionX", &RigidBodyComponent::body, &Body::getConstraints, &Body::setConstraints, physics::FREEZE_POSITION_X).metadata(reflect::Axes{"Freeze Position"});
    flag(type, "freezePositionY", &RigidBodyComponent::body, &Body::getConstraints, &Body::setConstraints, physics::FREEZE_POSITION_Y);
    flag(type, "freezePositionZ", &RigidBodyComponent::body, &Body::getConstraints, &Body::setConstraints, physics::FREEZE_POSITION_Z);
    flag(type, "freezeRotationX", &RigidBodyComponent::body, &Body::getConstraints, &Body::setConstraints, physics::FREEZE_ROTATION_X).metadata(reflect::Axes{"Freeze Rotation"});
    flag(type, "freezeRotationY", &RigidBodyComponent::body, &Body::getConstraints, &Body::setConstraints, physics::FREEZE_ROTATION_Y);
    flag(type, "freezeRotationZ", &RigidBodyComponent::body, &Body::getConstraints, &Body::setConstraints, physics::FREEZE_ROTATION_Z);
}

static void registerEnvironment()
{
    const Field::Condition plain = whenNot<Env>(&Env::isSkybox);
    const Field::Condition skybox = when<Env>(&Env::isSkybox);
    const Field::Condition cross = when<Env>(&Env::isCross);
    const Field::Condition faces = when<Env>(&Env::isFaces);

    Type& type = reflect::reflect<Env>("EnvironmentComponent");

    options<Env::Background>(property<Env>(type, "background", &Env::getBackground, &Env::setBackground));
    property<Env>(type, "color", &Env::getColor, &Env::setColor).metadata(reflect::Color{}).setCondition(plain);
    options<Env::Layout>(property<Env>(type, "layout", &Env::getLayout, &Env::setLayout)).setCondition(skybox);
    property<Env>(type, "texture", &Env::getCrossKey, &Env::setCrossKey).metadata(reflect::Asset{}).setCondition(cross);

    property<Env>(type, "right", &Env::getRightKey, &Env::setRightKey).metadata(reflect::Asset{}).setCondition(faces);
    property<Env>(type, "left", &Env::getLeftKey, &Env::setLeftKey).metadata(reflect::Asset{}).setCondition(faces);
    property<Env>(type, "top", &Env::getTopKey, &Env::setTopKey).metadata(reflect::Asset{}).setCondition(faces);
    property<Env>(type, "bottom", &Env::getBottomKey, &Env::setBottomKey).metadata(reflect::Asset{}).setCondition(faces);
    property<Env>(type, "front", &Env::getFrontKey, &Env::setFrontKey).metadata(reflect::Asset{}).setCondition(faces);
    property<Env>(type, "back", &Env::getBackKey, &Env::setBackKey).metadata(reflect::Asset{}).setCondition(faces);
}

static void registerComponents()
{
    reflect::reflect<IdentityComponent>("IdentityComponent").setHidden(true);
    reflect::reflect<RenderableComponent>("RenderableComponent");
    reflect::reflect<CameraComponent>("CameraComponent");
    reflect::reflect<LightComponent>("LightComponent");
    reflect::reflect<CanvasComponent>("CanvasComponent");
    reflect::reflect<ColliderComponent>("ColliderComponent");

    Type& script = reflect::reflect<ScriptComponent>("ScriptComponent");
    property<ScriptComponent>(script, "script", &ScriptComponent::getScriptKey, &ScriptComponent::setScriptKey).metadata(reflect::Asset{});
}

// base stands before what derives from it, or inherited fields are not found
void registerTypes()
{
    registerShapes();
    registerLights();
    registerMaterial();
    registerRenderables();

    reflect::reflect<Component>("Component").setHidden(true);

    registerTransforms();
    registerBodies();
    registerEnvironment();
    registerComponents();
}

} // namespace ecs
} // namespace BulletEngine
