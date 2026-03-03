/*
 * CollisionSystem.cpp
 */

#include "CollisionSystem.h"

#include <iostream>
#include <cmath>

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

    // skip repeated collisions with same target
    auto* impactState = world.get<ImpactStateComponent>(projectileEntity);
    if (impactState && impactState->lastImpactTarget == targetEntity)
    {
        return;
    }

    if (targetCollider->collider->getMaterial().has_value())
    {
        BulletPhysics::collision::terminal::ImpactInfo impactInfo;
        impactInfo.normal = manifold.info.normal;
        impactInfo.material = targetCollider->collider->getMaterial().value();
        impactInfo.thickness = targetCollider->collider->computeThickness(projectileBody.getPosition(), projectileBody.getVelocity());

        double speedBefore = projectileBody.getVelocity().length();
        double mass = projectileBody.getMass();
        double keBefore = 0.5 * mass * speedBefore * speedBefore;

        auto result = BulletPhysics::collision::terminal::Impact::resolve(projectileBody, impactInfo);

        double speedAfter = result.residualVelocity.length();
        double keAfter = 0.5 * mass * speedAfter * speedAfter;
        double keLoss = (keBefore > 0.0) ? (1.0 - keAfter / keBefore) * 100.0 : 0.0;

        const char* outcomeName = "";
        switch (result.outcome)
        {
            case BulletPhysics::collision::terminal::ImpactOutcome::Ricochet:
                outcomeName = "Ricochet";
                break;
            case BulletPhysics::collision::terminal::ImpactOutcome::Penetration:
                outcomeName = "Penetration";
                break;
            case BulletPhysics::collision::terminal::ImpactOutcome::Embed:
                outcomeName = "Embed";
                break;
        }

        std::cout << "[Impact] " << outcomeName << " | "
                  << "Material: " << impactInfo.material.name << " | "
                  << "Speed: " << speedBefore << " -> " << speedAfter << " m/s | "
                  << "Ek: " << keBefore << " -> " << keAfter << " J | "
                  << "Loss: " << keLoss << "%\n";

        switch (result.outcome)
        {
            case BulletPhysics::collision::terminal::ImpactOutcome::Ricochet:
            {
                projectileBody.setVelocity(result.residualVelocity);

                auto pos = projectileBody.getPosition();
                pos += manifold.info.normal * (manifold.info.penetration + 0.001);
                projectileBody.setPosition(pos);

                if (impactState)
                {
                    impactState->hasImpacted = true;
                    impactState->lastImpactTarget = targetEntity;
                }

                break;
            }
            case BulletPhysics::collision::terminal::ImpactOutcome::Penetration:
            {
                projectileBody.setVelocity(result.residualVelocity);

                auto pos = projectileBody.getPosition();
                auto vel = result.residualVelocity.normalized();
                pos += vel * (manifold.info.penetration + 0.05);
                projectileBody.setPosition(pos);

                if (impactState)
                {
                    impactState->hasImpacted = true;
                    impactState->lastImpactTarget = targetEntity;
                }

                break;
            }
            case BulletPhysics::collision::terminal::ImpactOutcome::Embed:
            {
                projectileBody.setVelocity({0.0, 0.0, 0.0});
                rigidBodyComponent->isGrounded = true;

                if (impactState)
                {
                    impactState->hasImpacted = true;
                    impactState->lastImpactTarget = targetEntity;
                }

                break;
            }
        }
    }
    else
    {
        // no material, fallback (stop projectile)
        projectileBody.setVelocity({0.0, 0.0, 0.0});
        rigidBodyComponent->isGrounded = true;
    }
}

} // namespace systems
} // namespace ecs
} // namespace BulletEngine
