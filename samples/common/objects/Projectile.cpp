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
    return projectile.create(world);
}

ecs::Entity Projectile::create(ecs::World& world)
{
    ecs::Entity entity = world.create();

    // transform
    auto& transformComponent = world.add<ecs::TransformComponent>(entity);

    float modelScale = m_diameter / m_modelDiameter;
    float length = m_modelLength * modelScale;

    transformComponent.transform.setScale({modelScale, modelScale, modelScale});

    // projectile specs
    BulletPhysics::dynamics::projectile::ProjectileSpecs specs{};
    specs.mass = m_mass;
    specs.diameter = m_diameter;
    specs.dragModel = m_dragModel;

    // rigid body
    auto& rigidBodyComponent = world.add<ecs::ProjectileRigidBodyComponent>(entity, specs);
    rigidBodyComponent.getProjectileBody().setPosition({m_initialPosX, m_initialPosY, m_initialPosZ});
    rigidBodyComponent.getProjectileBody().setVelocityFromAngles(m_launchSpeed, m_launchElevationDeg, m_launchAzimuthDeg);

    // trajectory
    auto& trajectoryComponent = world.add<ecs::TrajectoryComponent>(entity);
    trajectoryComponent.points.push_back({m_initialPosX, m_initialPosY, m_initialPosZ});

    // assets
    auto model = std::make_unique<BulletRender::scene::Model>(m_modelPath);
    auto shader = std::make_shared<BulletRender::render::Shader>(m_vertexShaderPath, m_fragmentShaderPath);

    // renderable
    auto& renderableComponent = world.add<ecs::RenderableComponent>(entity);
    renderableComponent.model = model.release();
    renderableComponent.material.setShader(shader);
    renderableComponent.material.setColor({m_colorR, m_colorG, m_colorB});

    // collider
    auto& colliderComponent = world.add<ecs::ColliderComponent>(entity);
    colliderComponent.collider = std::make_shared<BulletPhysics::collision::BoxCollider>(BulletPhysics::math::Vec3{m_diameter, length, m_diameter});

    // collider debug visibility
    if (m_showCollider)
    {
        colliderComponent.isVisible = true;
        colliderComponent.model = new BulletRender::scene::Box(m_modelDiameter, m_modelLength, m_modelDiameter);
        colliderComponent.material.setShader(shader);
        colliderComponent.material.setColor({0.0f, 1.0f, 0.0f});
    }

    fired.push_back(entity);

    return entity;
}

ecs::Entity Projectile::launch(ecs::World& world, const BulletPhysics::math::Vec3& position, float speed, float elevation, float azimuth)
{
    Projectile projectile;

    projectile.m_initialPosX = position.x;
    projectile.m_initialPosY = position.y;
    projectile.m_initialPosZ = position.z;

    projectile.m_launchSpeed = speed;
    projectile.m_launchElevationDeg = elevation;
    projectile.m_launchAzimuthDeg = azimuth;

    return projectile.create(world);
}

} // namespace objects
} // namespace BulletEngine
