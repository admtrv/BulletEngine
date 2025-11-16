/*
 * RenderSystem.cpp
 */

#include "RenderSystem.h"

namespace BulletEngine {
namespace ecs {
namespace systems {

void RenderSystem::rebuild(World& world)
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
        }

        // render colliders
        auto* colliderComponent = world.get<ColliderComponent>(entity);
        if (transformComponent && colliderComponent && colliderComponent->isVisible && colliderComponent->model)
        {
            auto* colliderObject = m_scene.addObject(colliderComponent->model);

            colliderObject->getMaterial().setShader(colliderComponent->material.getShader());
            colliderObject->getMaterial().setColor(colliderComponent->material.getColor());

            colliderObject->getTransform().setMatrix(transformComponent->transform.getMatrix());
        }
    }
}

} // namespace systems
} // namespace ecs
} // namespace BulletEngine
