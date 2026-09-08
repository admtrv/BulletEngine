/*
 * RenderSystem.cpp
 */

#include "RenderSystem.h"

namespace BulletEngine {
namespace ecs {
namespace systems {

RenderSystem::RenderSystem(BulletRender::scene::Scene& scene) : m_scene(scene) {}

void RenderSystem::render(World& world)
{
    m_scene.clearObjects();

    for (auto entity : world.entities())
    {
        auto* transformComponent = world.get<TransformComponent>(entity);
        auto* renderableComponent = world.get<RenderableComponent>(entity);

        if (!transformComponent || !renderableComponent || !renderableComponent->model)
        {
            continue;
        }

        auto* object = m_scene.addObject(renderableComponent->model);

        object->getMaterial() = renderableComponent->material;
        object->getTransform().setMatrix(transformComponent->transform.getMatrix());
    }
}

} // namespace systems
} // namespace ecs
} // namespace BulletEngine
