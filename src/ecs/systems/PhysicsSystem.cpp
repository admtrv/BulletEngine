/*
 * PhysicsSystem.cpp
 */

#include "PhysicsSystem.h"
#include <cstdio>
#include <vector>

namespace BulletEngine {
namespace ecs {
namespace systems {

PhysicsSystem::PhysicsSystem(BulletPhysic::dynamics::PhysicsWorld& physicsWorld, BulletPhysic::math::IIntegrator& integrator)
    : m_physicsWorld(physicsWorld)
    , m_integrator(integrator)
{}

void printForces(const BulletPhysic::dynamics::PhysicsWorld& world)
{
    std::printf("\r");

    // collect active forces
    std::vector<std::pair<std::string, float>> forces;
    for (const auto& force : world.getForces())
    {
        if (force && force->isActive())
        {
            float magnitude = force->getForce().length();
            forces.push_back({force->getSymbol(), magnitude});
        }
    }

    // print forces
    for (size_t i = 0; i < forces.size(); i++)
    {
        std::printf("%s = %.8f N", forces[i].first.c_str(), forces[i].second);
        if (i < forces.size() - 1)
        {
            std::printf(", ");
        }
    }
    std::fflush(stdout);
}

void PhysicsSystem::update(World& world, float dt)
{
    for (auto entity : world.entities())
    {
        auto* transformComponent = world.get<TransformComponent>(entity);
        auto* rigidBodyComponent = world.get<ProjectileRigidBodyComponent>(entity);

        if (!rigidBodyComponent)
        {
            continue;
        }

        // apply forces
        if (!rigidBodyComponent->body.isGrounded())
        {
            m_integrator.step(rigidBodyComponent->body, &m_physicsWorld, dt);

            // print all forces
            printForces(m_physicsWorld);
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

} // namespace systems
} // namespace ecs
} // namespace BulletEngine
