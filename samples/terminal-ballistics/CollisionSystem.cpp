/*
 * CollisionSystem.cpp
 */

#include "CollisionSystem.h"

namespace BulletEngine {
namespace ecs {
namespace systems {

void TerminalCollisionSystem::onCollision(World& world, Entity entityA, Entity entityB, const BulletPhysics::collision::Manifold& manifold)
{
    Entity targetEntity = 0;
    Entity projectileEntity = 0;

    auto* colliderA = world.get<ColliderComponent>(entityA);
    auto* colliderB = world.get<ColliderComponent>(entityB);

    if (!colliderA || !colliderB)
    {
        return;
    }

    bool aIsProjectile = world.has<ProjectileRigidBodyComponent>(entityA);
    bool bIsProjectile = world.has<ProjectileRigidBodyComponent>(entityB);

    if (aIsProjectile && !bIsProjectile)
    {
        projectileEntity = entityA;
        targetEntity = entityB;
    }
    else if (bIsProjectile && !aIsProjectile)
    {
        projectileEntity = entityB;
        targetEntity = entityA;
    }
    else
    {
        return;
    }

    auto* rigidBodyComponent = world.get<ProjectileRigidBodyComponent>(projectileEntity);
    if (!rigidBodyComponent)
    {
        return;
    }

    auto* targetCollider = world.get<ColliderComponent>(targetEntity);
    auto& projectileBody = rigidBodyComponent->getProjectileBody();

    if (targetCollider->collider->getMaterial().has_value())
    {
        BulletPhysics::collision::terminal::ImpactInfo impactInfo;
        impactInfo.normal = manifold.info.normal;
        impactInfo.material = targetCollider->collider->getMaterial().value();
        impactInfo.thickness = targetCollider->collider->computeThickness(projectileBody.getPosition(), projectileBody.getVelocity());

        auto result = BulletPhysics::collision::terminal::Impact::resolve(projectileBody, impactInfo);

        switch (result.outcome)
        {
            case BulletPhysics::collision::terminal::ImpactOutcome::Ricochet:
            {
                projectileBody.setVelocity(result.residualVelocity);

                auto pos = projectileBody.getPosition();
                pos += manifold.info.normal * (manifold.info.penetration + 0.001f);
                projectileBody.setPosition(pos);

                auto* impactState = world.get<ImpactStateComponent>(projectileEntity);
                if (impactState)
                    impactState->hasImpacted = true;

                break;
            }
            case BulletPhysics::collision::terminal::ImpactOutcome::Penetration:
            {
                projectileBody.setVelocity(result.residualVelocity);

                auto pos = projectileBody.getPosition();
                auto vel = result.residualVelocity.normalized();
                pos += vel * (manifold.info.penetration + 0.05f);
                projectileBody.setPosition(pos);

                auto* impactState = world.get<ImpactStateComponent>(projectileEntity);
                if (impactState)
                    impactState->hasImpacted = true;

                break;
            }
            case BulletPhysics::collision::terminal::ImpactOutcome::Embed:
            {
                projectileBody.setVelocity({0.0f, 0.0f, 0.0f});
                rigidBodyComponent->isGrounded = true;

                auto* impactState = world.get<ImpactStateComponent>(projectileEntity);
                if (impactState)
                    impactState->hasImpacted = true;

                break;
            }
        }
    }
    else
    {
        // no material, fallback (stop projectile)
        projectileBody.setVelocity({0.0f, 0.0f, 0.0f});
        rigidBodyComponent->isGrounded = true;
    }
}

} // namespace systems
} // namespace ecs
} // namespace BulletEngine
