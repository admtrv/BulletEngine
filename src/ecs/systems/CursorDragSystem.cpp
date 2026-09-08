/*
 * CursorDragSystem.cpp
 */

#include "CursorDragSystem.h"

#include "app/Window.h"
#include "render/Renderer.h"

namespace BulletEngine {
namespace ecs {
namespace systems {

namespace bp = BulletPhysics;

// how far a pick may reach
static constexpr double PICK_RANGE = 1000.0;

// a fast flick of the mouse would fling the body away without a cap
static constexpr double MAX_PULL = 60.0;

static bp::math::Vec3 toPhysics(const glm::vec3& v)
{
    return {v.x, v.y, v.z};
}

CursorDragSystem::CursorDragSystem(BulletRender::scene::Camera& camera, const PhysicsSystem& physics)
    : m_camera(camera), m_physics(physics) {}

void CursorDragSystem::update()
{
    const bool pressed = BulletRender::app::Window::isMouseDown(BulletRender::app::MouseButton::Left);

    if (!pressed)
    {
        release();

        m_wasPressed = false;
        return;
    }

    double cursorX = 0.0;
    double cursorY = 0.0;
    BulletRender::app::Window::getCursorPos(cursorX, cursorY);

    const bp::collision::Ray ray = rayThroughCursor(cursorX, cursorY);

    if (!m_wasPressed)
    {
        grab(ray);
    }
    else
    {
        pull(ray);
    }

    m_wasPressed = true;
}

bp::collision::Ray CursorDragSystem::rayThroughCursor(double cursorX, double cursorY) const
{
    int width = 0;
    int height = 0;
    BulletRender::app::Window::getSize(width, height);

    // cursor to clip space, y runs down the screen and up in clip space
    const float clipX = 2.0f * static_cast<float>(cursorX) / static_cast<float>(width) - 1.0f;
    const float clipY = 1.0f - 2.0f * static_cast<float>(cursorY) / static_cast<float>(height);

    const glm::mat4 inverse = glm::inverse(m_camera.getProj(BulletRender::render::Renderer::getAspect()) * m_camera.getView());

    // unproject the near and far ends of the pixel, the line between them is the ray
    glm::vec4 near = inverse * glm::vec4(clipX, clipY, -1.0f, 1.0f);
    glm::vec4 far = inverse * glm::vec4(clipX, clipY, 1.0f, 1.0f);

    near /= near.w;
    far /= far.w;

    const glm::vec3 direction = glm::normalize(glm::vec3(far - near));

    return {toPhysics(glm::vec3(near)), toPhysics(direction), PICK_RANGE};
}

void CursorDragSystem::grab(const bp::collision::Ray& ray)
{
    bp::collision::RayHit hit;
    if (!m_physics.raycast(ray, hit))
    {
        return;
    }

    // only dynamic bodies are worth dragging, the ground would just eat the click
    bp::dynamics::RigidBody* body = hit.collider->getBody();
    if (!body || !body->isDynamic())
    {
        return;
    }

    m_body = body;
    m_grabDistance = hit.distance;

    // remember the grab point in body space so it follows the body as it turns
    m_localGrab = body->getOrientation().conjugated().rotate(hit.point - body->getPosition());
}

void CursorDragSystem::pull(const bp::collision::Ray& ray)
{
    if (!m_body)
    {
        return;
    }

    const bp::math::Vec3 grabPoint = m_body->getPosition() + m_body->getOrientation().rotate(m_localGrab);
    const bp::math::Vec3 target = ray.pointAt(m_grabDistance);

    // spring towards the cursor, damped by how fast the grabbed point already moves
    bp::math::Vec3 pull = (target - grabPoint) * m_stiffness - m_body->getVelocityAt(grabPoint) * m_damping;

    const double strength = pull.length();
    if (strength > MAX_PULL)
    {
        pull = pull * (MAX_PULL / strength);
    }

    m_body->addForceAtPoint(pull * m_body->getMass(), grabPoint);
}

void CursorDragSystem::release()
{
    m_body = nullptr;
}

} // namespace systems
} // namespace ecs
} // namespace BulletEngine
