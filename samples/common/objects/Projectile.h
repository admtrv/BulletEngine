/*
 * Projectile.h
 */

#pragma once

#include "ecs/Ecs.h"
#include "ecs/Components.h"
#include "scene/Model.h"
#include "render/Shader.h"
#include "builtin/collision/collider/BoxCollider.h"
#include "common/Components.h"
#include "PhysicsBody.h"

#include <vector>
#include <string>

namespace BulletEngine {
namespace objects {

// model defaults
static constexpr double MODEL_DIAMETER = 1.0;
static constexpr double MODEL_LENGTH = 3.32;

static const std::string MODEL_PATH = "assets/models/g7.obj";
static const std::string VERTEX_SHADER_PATH = "assets/shaders/normal.vert.glsl";
static const std::string FRAGMENT_SHADER_PATH = "assets/shaders/normal.frag.glsl";

// material defaults
static constexpr float COLOR_R = 1.0f;
static constexpr float COLOR_G = 0.5f;
static constexpr float COLOR_B = 0.0f;

// projectile entity factory
class Projectile {
public:
    static ecs::Entity launch(ecs::World& world, const BulletPhysics::projectile::ProjectileSpecs& specs, const BulletPhysics::math::Vec3& position, double elevationDeg, double azimuthDeg, bool showCollider = false);

    static std::vector<ecs::Entity> fired;

private:
    static void setupTransform(ecs::World& world, ecs::Entity entity, double diameter);
    static void setupRigidBody(ecs::World& world, ecs::Entity entity, const BulletPhysics::projectile::ProjectileSpecs& specs, const BulletPhysics::math::Vec3& position, double elevationDeg, double azimuthDeg);
    static void setupTrajectory(ecs::World& world, ecs::Entity entity, const BulletPhysics::math::Vec3& position);
    static void setupRenderable(ecs::World& world, ecs::Entity entity);
    static void setupCollider(ecs::World& world, ecs::Entity entity, double diameter, bool showCollider);
};

} // namespace objects
} // namespace BulletEngine
