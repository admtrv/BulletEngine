/*
 * DebugDrawSystem.cpp
 */

#include "DebugDrawSystem.h"

#include "Colors.h"
#include "render/Renderer.h"

#include "collision/collider/BoxCollider.h"
#include "collision/collider/CylinderCollider.h"
#include "collision/collider/SphereCollider.h"

#include <glm/gtc/matrix_transform.hpp>

namespace BulletEngine {
namespace ecs {
namespace systems {

namespace bpc = BulletPhysics::collision::collider;

static constexpr float GROUND_RADIUS = 1.5f;    // plane is endless, only patch of it is hinted
static constexpr float VELOCITY_SCALE = 0.2f;   // metres per second to arrow length
static constexpr float CONTACT_LENGTH = 0.5f;
static constexpr float SLOWEST_SHOWN = 0.1f;    // slower ones clutter view

static const glm::vec3 UP{0.0f, 1.0f, 0.0f};

static glm::vec3 toGlm(const BulletPhysics::math::Vec3& v)
{
    return {static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z)};
}

static glm::quat toGlmQuat(const BulletPhysics::math::Quat& q)
{
    return {static_cast<float>(q.w), static_cast<float>(q.x), static_cast<float>(q.y), static_cast<float>(q.z)};
}

DebugDrawSystem::DebugDrawSystem(std::shared_ptr<BulletRender::render::Lines> lines) : m_debugDraw(std::move(lines)) {}

void DebugDrawSystem::draw(World& world, Entity selected, const std::vector<BulletPhysics::collision::Manifold>& contacts)
{
    if (m_showColliders)
    {
        drawColliders(world);
    }

    if (m_showPhysics)
    {
        drawPhysics(world, contacts);
    }

    if (m_showLights)
    {
        drawLights(world);
    }

    if (m_showCameras)
    {
        drawCameras(world);
    }

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

            // sign of each axis picked by bits of corner index
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
        else if (collider->getShape() == bpc::CollisionShape::Cylinder)
        {
            const auto* cylinder = static_cast<const bpc::CylinderCollider*>(collider);

            m_debugDraw.drawCylinder(toGlm(cylinder->getPosition()), toGlm(cylinder->getAxis()),
                                     static_cast<float>(cylinder->getRadius()), static_cast<float>(cylinder->getHeight()),
                                     BulletRender::colors::White);
        }
        else if (collider->getShape() == bpc::CollisionShape::Ground)
        {
            m_debugDraw.drawPlane(toGlm(collider->getPosition()), UP, GROUND_RADIUS, BulletRender::colors::White);
        }
    }
}

// where bodies head and where they touch
void DebugDrawSystem::drawPhysics(World& world, const std::vector<BulletPhysics::collision::Manifold>& contacts)
{
    for (Entity entity : world.getEntities())
    {
        const auto* component = world.get<RigidBodyComponent>(entity);

        if (!component || component->body.getVelocity().length() < SLOWEST_SHOWN)
        {
            continue;
        }

        const glm::vec3 from = toGlm(component->body.getPosition());
        const glm::vec3 to = from + toGlm(component->body.getVelocity()) * VELOCITY_SCALE;

        m_debugDraw.drawArrow(from, to, BulletRender::colors::Green);
    }

    for (const auto& manifold : contacts)
    {
        for (int i = 0; i < manifold.info.pointCount; i++)
        {
            const glm::vec3 point = toGlm(manifold.info.points[i].position);

            m_debugDraw.drawArrow(point, point + toGlm(manifold.info.normal) * CONTACT_LENGTH, BulletRender::colors::Red);
        }
    }
}

void DebugDrawSystem::drawLights(World& world)
{
    for (Entity entity : world.getEntities())
    {
        const auto* component = world.get<LightComponent>(entity);

        if (component && component->light)
        {
            m_debugDraw.drawLight(*component->light);
        }
    }
}

void DebugDrawSystem::drawCameras(World& world)
{
    for (Entity entity : world.getEntities())
    {
        const auto* camera = world.get<CameraComponent>(entity);
        const auto* transform = world.get<TransformComponent>(entity);

        if (!camera || !transform)
        {
            continue;
        }

        // view is built from pose alone, scale would bend frustum
        const glm::vec3 position = transform->transform.getPosition();

        m_debugDraw.drawFrustum(glm::lookAt(position, position + transform->transform.getForward(), UP),
                                BulletRender::render::Renderer::getAspect());
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
