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
    m_scene.clearLights();

    for (Entity entity : world.getEntities())
    {
        auto* transformComponent = world.get<TransformComponent>(entity);

        if (!transformComponent)
        {
            continue;
        }

        const glm::mat4& matrix = transformComponent->transform.getMatrix();

        if (auto* renderable = world.get<RenderableComponent>(entity); renderable && renderable->model)
        {
            auto* object = m_scene.addObject(renderable->model.getShared());

            object->getMaterial() = renderable->material;
            object->getTransform().setMatrix(matrix);
        }

        // scene shares the light component holds, entity pose places it
        if (auto* lightComponent = world.get<LightComponent>(entity); lightComponent && lightComponent->light)
        {
            lightComponent->light->getTransform().setMatrix(matrix);
            m_scene.addLight(lightComponent->light);
        }
    }
}

} // namespace systems
} // namespace ecs
} // namespace BulletEngine
