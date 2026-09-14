/*
 * DebugDrawSystem.cpp
 */

#include "DebugDrawSystem.h"

#include "Colors.h"

#include "collision/collider/BoxCollider.h"
#include "collision/collider/SphereCollider.h"

namespace BulletEngine {
namespace ecs {
namespace systems {

namespace bpc = BulletPhysics::collision::collider;

static constexpr float GROUND_RADIUS = 1.5f;    // the plane is endless, only a patch of it is hinted

static glm::vec3 toGlm(const BulletPhysics::math::Vec3& v)
{
    return {static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z)};
}

static glm::quat toGlmQuat(const BulletPhysics::math::Quat& q)
{
    return {static_cast<float>(q.w), static_cast<float>(q.x), static_cast<float>(q.y), static_cast<float>(q.z)};
}

DebugDrawSystem::DebugDrawSystem(std::shared_ptr<BulletRender::render::Lines> lines) : m_debugDraw(std::move(lines)) {}

void DebugDrawSystem::draw(World& world, Entity selected)
{
    if (!m_enabled)
    {
        return;
    }

    drawColliders(world);
    drawAxes(world, selected);
}

void DebugDrawSystem::drawColliders(World& world)
{
    for (auto entity : world.getEntities())
    {
        auto* colliderComponent = world.get<ColliderComponent>(entity);
        if (!colliderComponent || !colliderComponent->collider)
        {
            continue;
        }

        const auto* collider = colliderComponent->collider.get();

        if (collider->getShape() == bpc::CollisionShape::Sphere)
        {
            const auto* sphere = static_cast<const bpc::SphereCollider*>(collider);

            const auto& orientation = sphere->getOrientation();

            m_debugDraw.drawSphere(toGlm(sphere->getPosition()), static_cast<float>(sphere->getRadius()), toGlmQuat(orientation), BulletRender::colors::White);
        }
        else if (collider->getShape() == bpc::CollisionShape::Box)
        {
            const auto* box = static_cast<const bpc::BoxCollider*>(collider);

            const auto half = box->getSize() * 0.5;
            const auto* axes = box->getAxes();

            // sign of each axis picked by the bits of the corner index
            glm::vec3 corners[8];

            for (int i = 0; i < 8; i++)
            {
                auto corner = box->getPosition();

                corner += axes[0] * ((i & 1) ? half.x : -half.x);
                corner += axes[1] * ((i & 2) ? half.y : -half.y);
                corner += axes[2] * ((i & 4) ? half.z : -half.z);

                corners[i] = toGlm(corner);
            }

            m_debugDraw.drawBox(corners, BulletRender::colors::White);
        }
        else if (collider->getShape() == bpc::CollisionShape::Ground)
        {
            m_debugDraw.drawPlane(toGlm(collider->getPosition()), {0.0f, 1.0f, 0.0f}, GROUND_RADIUS, BulletRender::colors::White);
        }
    }
}

void DebugDrawSystem::drawAxes(World& world, Entity selected)
{
    if (selected == INVALID_ENTITY || !world.isAlive(selected))
    {
        return;
    }

    if (const auto* transform = world.get<TransformComponent>(selected))
    {
        m_debugDraw.drawTransform(transform->transform);
    }
}

} // namespace systems
} // namespace ecs
} // namespace BulletEngine
