/*
 * Projectile.cpp
 */

#include "Projectile.h"

namespace BulletEngine {
namespace objects {

std::vector<ecs::Entity> Projectile::fired;

ecs::Entity Projectile::launch(ecs::World& world)
{
    Projectile projectile;

    ecs::Entity entity = world.create();

    // transform
    auto& transformComponent = world.add<ecs::TransformComponent>(entity);
    float modelScale = projectile.m_diameter / projectile.m_modelDiameter;
    transformComponent.transform.setScale({modelScale, modelScale, modelScale});

    float length = projectile.m_modelLength * modelScale;

    // trajectory
    world.add<ecs::TrajectoryComponent>(entity);

    // projectile specs
    BulletPhysic::dynamics::projectile::ProjectileSpecs specs{};
    specs.mass = projectile.m_mass;
    specs.diameter = projectile.m_diameter;
    specs.dragModel = projectile.m_dragModel;

    // rigid body
    auto& rigidBodyComponent = world.add<ecs::ProjectileRigidBodyComponent>(entity);
    rigidBodyComponent.body = BulletPhysic::dynamics::projectile::ProjectileRigidBody(specs);
    rigidBodyComponent.body.setPosition({projectile.m_initialPosX, projectile.m_initialPosY, projectile.m_initialPosZ});
    rigidBodyComponent.body.setVelocityFromAngles(projectile.m_launchSpeed, projectile.m_launchElevationDeg, projectile.m_launchAzimuthDeg);

    // assets
    auto model = std::make_unique<BulletRender::scene::Model>(projectile.m_modelPath);
    auto shader = std::make_shared<BulletRender::render::Shader>(projectile.m_vertexShaderPath, projectile.m_fragmentShaderPath);

    // renderable
    auto& renderableComponent = world.add<ecs::RenderableComponent>(entity);
    renderableComponent.model = model.release();
    renderableComponent.material.setShader(shader);
    renderableComponent.material.setColor({projectile.m_colorR, projectile.m_colorG, projectile.m_colorB});

    // collider
    auto& colliderComponent = world.add<ecs::ColliderComponent>(entity);
    colliderComponent.collider = std::make_shared<BulletPhysic::collision::BoxCollider>(BulletPhysic::math::Vec3{projectile.m_diameter, length, projectile.m_diameter});

    // collider debug visibility
    // colliderComponent.isVisible = true;
    // colliderComponent.model = new BulletRender::scene::Box(projectile.m_modelDiameter, projectile.m_modelLength,projectile.m_modelDiameter);
    // colliderComponent.material.setShader(shader);
    // colliderComponent.material.setColor({0.0f, 1.0f, 0.0f}); // green

    fired.push_back(entity);

    return entity;
}

} // namespace objects
} // namespace BulletEngine
