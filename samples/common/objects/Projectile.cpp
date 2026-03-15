/*
 * Projectile.cpp
 */

#include "Projectile.h"

namespace BulletEngine {
namespace objects {

std::vector<ecs::Entity> Projectile::fired;

ecs::Entity Projectile::launch(ecs::World& world, const BulletPhysics::projectile::ProjectileSpecs& specs, const BulletPhysics::math::Vec3& position, double elevationDeg, double azimuthDeg, bool showCollider)
{
    ecs::Entity entity = world.create();

    setupTransform(world, entity, specs.diameter);
    setupRigidBody(world, entity, specs, position, elevationDeg, azimuthDeg);
    setupTrajectory(world, entity, position);
    setupRenderable(world, entity);
    setupCollider(world, entity, specs.diameter, showCollider);

    fired.push_back(entity);
    return entity;
}

void Projectile::setupTransform(ecs::World& world, ecs::Entity entity, double diameter)
{
    auto& transform = world.add<ecs::TransformComponent>(entity);

    float modelScale = static_cast<float>(diameter / MODEL_DIAMETER);
    transform.transform.setScale({modelScale, modelScale, modelScale});
}

void Projectile::setupRigidBody(ecs::World& world, ecs::Entity entity, const BulletPhysics::projectile::ProjectileSpecs& specs, const BulletPhysics::math::Vec3& position, double elevationDeg, double azimuthDeg)
{
    auto& rigidBody = world.add<ecs::ProjectileRigidBodyComponent>(entity, specs);
    rigidBody.getProjectileBody().setPosition(position);
    rigidBody.getProjectileBody().setAngles(elevationDeg, azimuthDeg);
}

void Projectile::setupTrajectory(ecs::World& world, ecs::Entity entity, const BulletPhysics::math::Vec3& position)
{
    auto& trajectory = world.add<ecs::TrajectoryComponent>(entity);
    trajectory.points.push_back(position);
}

void Projectile::setupRenderable(ecs::World& world, ecs::Entity entity)
{
    auto model = std::make_unique<BulletRender::scene::Model>(MODEL_PATH);
    auto shader = std::make_shared<BulletRender::render::Shader>(VERTEX_SHADER_PATH, FRAGMENT_SHADER_PATH);

    auto& renderable = world.add<ecs::RenderableComponent>(entity);
    renderable.model = model.release();
    renderable.material.setShader(shader);
    renderable.material.setColor({COLOR_R, COLOR_G, COLOR_B});
}

void Projectile::setupCollider(ecs::World& world, ecs::Entity entity, double diameter, bool showCollider)
{
    float modelScale = static_cast<float>(diameter / MODEL_DIAMETER);
    float length = static_cast<float>(MODEL_LENGTH * modelScale);
    float d = static_cast<float>(diameter);

    auto& collider = world.add<ecs::ColliderComponent>(entity);
    collider.collider = std::make_shared<BulletPhysics::builtin::collision::collider::BoxCollider>(BulletPhysics::math::Vec3{d, length, d});

    if (showCollider)
    {
        auto shader = std::make_shared<BulletRender::render::Shader>(VERTEX_SHADER_PATH, FRAGMENT_SHADER_PATH);

        collider.isVisible = true;
        collider.model = new BulletRender::scene::Box(MODEL_DIAMETER, MODEL_LENGTH, MODEL_DIAMETER);
        collider.material.setShader(shader);
        collider.material.setColor({0.0f, 1.0f, 0.0f});
    }
}

} // namespace objects
} // namespace BulletEngine
