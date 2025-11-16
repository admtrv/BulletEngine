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
    transformComponent.transform.setPosition({projectile.m_initialPosX, projectile.m_initialPosY, projectile.m_initialPosZ});

    // trajectory
    world.add<ecs::TrajectoryComponent>(entity);

    // rigid body
    auto& rigidBodyComponent = world.add<ecs::RigidBodyComponent>(entity);
    rigidBodyComponent.body.setMass(projectile.m_mass);
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
    colliderComponent.collider = std::make_shared<BulletPhysic::collision::BoxCollider>(BulletPhysic::math::Vec3{projectile.m_colliderX, projectile.m_colliderY, projectile.m_colliderZ});

    // collider debug visibility
    // colliderComponent.isVisible = true;
    // colliderComponent.model = new BulletRender::scene::BoxModel(projectile.m_colliderX, projectile.m_colliderY, projectile.m_colliderZ);
    // colliderComponent.material.setShader(shader);
    // colliderComponent.material.setColor({0.0f, 1.0f, 0.0f}); // green

    fired.push_back(entity);

    return entity;
}

} // namespace objects
} // namespace BulletEngine
