/*
 * PhysicSystem.cpp
 */

#include "PhysicSystem.h"

namespace BulletEngine {
namespace ecs {
namespace systems {

PhysicSystem::PhysicSystem(BulletPhysic::math::IIntegrator& integrator) : m_integrator(integrator)
{
    setRealismLevel(BulletPhysic::preset::RealismLevel::BASIC); // start with basic simulation
}

void PhysicSystem::update(World& world, float dt)
{
    for (auto entity : world.entities())
    {
        auto* transformComponent = world.get<TransformComponent>(entity);
        auto* rigidBodyComponent = world.get<RigidBodyComponent>(entity);

        if (!rigidBodyComponent)
        {
            continue;
        }

        // apply forces
        if (!rigidBodyComponent->body.isGrounded())
        {
            m_integrator.step(rigidBodyComponent->body, &m_forceRegistry, dt);
        }

        // update transform if entity has one
        if (transformComponent)
        {
            const auto& p = rigidBodyComponent->body.position();
            const auto& v = rigidBodyComponent->body.velocity();

            // update position
            transformComponent->transform.setPosition({p.x, p.y, p.z});

            // update orientation based on velocity
            float velLen2 = v.x*v.x + v.y*v.y + v.z*v.z;
            if (velLen2 > 1e-6f && !rigidBodyComponent->body.isGrounded())
            {
                transformComponent->transform.rotateFromDirection({0.0f, 1.0f, 0.0f}, {v.x, v.y, v.z});
            }
        }

        // update collider position and orientation if entity has one
        auto* colliderComponent = world.get<ColliderComponent>(entity);
        if (colliderComponent && colliderComponent->collider && transformComponent)
        {
            const auto& p = rigidBodyComponent->body.position();
            colliderComponent->collider->setPosition(p);

            // sync orientation for box colliders
            if (colliderComponent->collider->getShape() == BulletPhysic::collision::CollisionShape::Box)
            {
                auto* boxCollider = static_cast<BulletPhysic::collision::BoxCollider*>(colliderComponent->collider.get());

                auto axisX = reinterpret_cast<const BulletPhysic::math::Vec3&>(transformComponent->transform.getMatrix()[0]);
                auto axisY = reinterpret_cast<const BulletPhysic::math::Vec3&>(transformComponent->transform.getMatrix()[1]);
                auto axisZ = reinterpret_cast<const BulletPhysic::math::Vec3&>(transformComponent->transform.getMatrix()[2]);

                boxCollider->setAxes(axisX, axisY, axisZ);

            }
        }
    }
}

void PhysicSystem::setRealismLevel(BulletPhysic::preset::RealismLevel level, float projectileArea)
{
    BulletPhysic::preset::PresetManager::configure(m_forceRegistry, level, projectileArea);
}

void PhysicSystem::setWindVelocity(const BulletPhysic::math::Vec3& windVel)
{
    auto wind = m_forceRegistry.getByName(std::string("Wind Drag"));
    if (wind)
    {
        static_cast<BulletPhysic::dynamics::forces::WindDragForce*>(wind)->setWindVelocity(windVel);
    }
}

} // namespace systems
} // namespace ecs
} // namespace BulletEngine
