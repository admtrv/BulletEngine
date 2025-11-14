/*
 * RenderSystem.cpp
 */

#include "RenderSystem.h"

namespace BulletEngine {
namespace ecs {
namespace systems {

void RenderSystem::rebuild(ecs::World& world)
{
    m_scene.clear();

    for (auto entity : world.entities())
    {
        auto* transformComponent = world.get<ecs::TransformComponent>(entity);
        auto* renderableComponent = world.get<ecs::RenderableComponent>(entity);

        if (!transformComponent || !renderableComponent)
        {
            continue;
        }

        if (!renderableComponent->model)
        {
            continue;
        }

        auto* object = m_scene.addObject(renderableComponent->model);

        object->getMaterial().setShader(renderableComponent->material.getShader());
        object->getMaterial().setColor(renderableComponent->material.getColor());

        object->getTransform().setMatrix(transformComponent->transform.getMatrix());
    }
}

} // namespace systems
} // namespace ecs
} // namespace BulletEngine
