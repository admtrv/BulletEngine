/*
 * Components.cpp
 */

#include "Components.h"

#include "reflect/Reflect.h"

#include "collision/collider/BoxCollider.h"
#include "collision/collider/GroundCollider.h"
#include "collision/collider/SphereCollider.h"

using namespace BulletEngine::ecs;
using namespace BulletPhysics::collision;
using namespace BulletPhysics::collision::collider;

// shapes

REFLECT(PhysicsMaterial)
    FIELD("friction", friction)
    FIELD("restitution", restitution)
END_REFLECT()

REFLECT(Collider)
    PROPERTY("trigger", isTrigger, setTrigger)
    OBJECT_REF("material", getMaterial)
END_REFLECT()

REFLECT(BoxCollider)
    BASE(Collider)
    PROPERTY("size", getSize, setSize)
END_REFLECT()

REFLECT(SphereCollider)
    BASE(Collider)
    PROPERTY("radius", getRadius, setRadius)
END_REFLECT()

REFLECT(GroundCollider)
    BASE(Collider)
    PROPERTY("level", getGroundY, setGroundY)
END_REFLECT()

// components

REFLECT(TransformComponent)
    NESTED("position", transform, getPosition, setPosition)
    NESTED("rotation", transform, getRotation, setRotation)
    NESTED_AS("scale", transform, getLocalScale, setLocalScale, const glm::vec3&)
END_REFLECT()

REFLECT(RenderableComponent)
    NESTED("color", material, getColor, setColor)
END_REFLECT()

REFLECT(RigidBodyComponent)
    NESTED("motion", body, getMotionType, setMotionType)
    NESTED("mass", body, getMass, setMass)
    NESTED("velocity", body, getVelocity, setVelocity)
    NESTED("angularVelocity", body, getAngularVelocity, setAngularVelocity)
    NESTED("linearDamping", body, getLinearDamping, setLinearDamping)
    NESTED("angularDamping", body, getAngularDamping, setAngularDamping)
END_REFLECT()

REFLECT(ColliderComponent)
    OBJECT("collider", collider)
END_REFLECT()
