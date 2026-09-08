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

static constexpr float VELOCITY_SCALE = 0.2f;
static constexpr float CONTACT_LENGTH = 0.5f;
static constexpr float SLOWEST_SHOWN = 0.1f;

static glm::vec3 toGlm(const BulletPhysics::math::Vec3& v)
{
    return {static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z)};
}

static glm::quat toGlmQuat(const BulletPhysics::math::Quat& q)
{
    return {static_cast<float>(q.w), static_cast<float>(q.x), static_cast<float>(q.y), static_cast<float>(q.z)};
}

DebugDrawSystem::DebugDrawSystem(std::shared_ptr<BulletRender::render::Lines> lines) : m_debugDraw(std::move(lines)) {}

void DebugDrawSystem::draw(World& world, const std::vector<BulletPhysics::collision::Manifold>& contacts)
{
    if (!m_enabled)
    {
        return;
    }

    if (m_showColliders)
    {
        drawColliders(world);
    }

    if (m_showVelocities)
    {
        drawVelocities(world);
    }

    if (m_showContacts)
    {
        drawContacts(contacts);
    }
}

void DebugDrawSystem::drawColliders(World& world)
{
    for (auto entity : world.entities())
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
    }
}

void DebugDrawSystem::drawVelocities(World& world)
{
    for (auto entity : world.entities())
    {
        auto* rigidBodyComponent = world.get<RigidBodyComponent>(entity);
        if (!rigidBodyComponent)
        {
            continue;
        }

        const auto& body = rigidBodyComponent->body;
        if (body.getVelocity().length() < SLOWEST_SHOWN)
        {
            continue;
        }

        const glm::vec3 from = toGlm(body.getPosition());
        const glm::vec3 to = from + toGlm(body.getVelocity()) * VELOCITY_SCALE;

        m_debugDraw.drawArrow(from, to, BulletRender::colors::Green);
    }
}

void DebugDrawSystem::drawContacts(const std::vector<BulletPhysics::collision::Manifold>& contacts)
{
    for (const auto& manifold : contacts)
    {
        for (int i = 0; i < manifold.info.pointCount; i++)
        {
            const glm::vec3 point = toGlm(manifold.info.points[i].position);

            m_debugDraw.drawArrow(point, point + toGlm(manifold.info.normal) * CONTACT_LENGTH, BulletRender::colors::Red);
        }
    }
}

} // namespace systems
} // namespace ecs
} // namespace BulletEngine
