/*
 * RenderSystem.cpp
 */

#include "RenderSystem.h"

namespace BulletEngine {
namespace ecs {
namespace systems {

RenderSystemBase::RenderSystemBase(BulletRender::scene::Scene& scene) : m_scene(scene) {}

void RenderSystemBase::render(World& world)
{
    m_scene.clear();

    for (auto entity : world.entities())
    {
        auto* transformComponent = world.get<TransformComponent>(entity);

        // render normal objects
        auto* renderableComponent = world.get<RenderableComponent>(entity);
        if (transformComponent && renderableComponent && renderableComponent->model)
        {
            auto* object = m_scene.addObject(renderableComponent->model);

            object->getMaterial().setShader(renderableComponent->material.getShader());
            object->getMaterial().setColor(renderableComponent->material.getColor());

            object->getTransform().setMatrix(transformComponent->transform.getMatrix());

            onObjectRender(world, entity, *object);
        }

        // render colliders
        auto* colliderComponent = world.get<ColliderComponent>(entity);
        if (transformComponent && colliderComponent && colliderComponent->isVisible && colliderComponent->model)
        {
            auto* collider = m_scene.addObject(colliderComponent->model);

            collider->getMaterial().setShader(colliderComponent->material.getShader());
            collider->getMaterial().setColor(colliderComponent->material.getColor());

            collider->getTransform().setMatrix(transformComponent->transform.getMatrix());

            onColliderRender(world, entity, *collider);
        }
    }
}

} // namespace systems
} // namespace ecs
} // namespace BulletEngine
