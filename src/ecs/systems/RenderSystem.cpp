/*
 * RenderSystem.cpp
 */

#include "RenderSystem.h"

#include "render/Renderer.h"

#include <glm/gtc/matrix_transform.hpp>

namespace BulletEngine {
namespace ecs {
namespace systems {

RenderSystem::RenderSystem(BulletRender::scene::Scene& scene, std::shared_ptr<BulletRender::render::SkyBox> skybox)
    : m_scene(scene), m_skybox(std::move(skybox)) {}

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

        // renderable without geometry has nothing to show yet, asset may still be coming
        if (auto* component = world.get<RenderableComponent>(entity); component && component->renderable && component->renderable->getModel())
        {
            Renderable& renderable = *component->renderable;
            auto* object = m_scene.addObject(renderable.getModel().getShared());

            renderable.material.fill(object->getMaterial());

            // picture sprite shows may have arrived since frame was chosen
            if (auto* sprite = dynamic_cast<Sprite*>(&renderable))
            {
                sprite->fitFrame();
                object->setFrame(sprite->getFrameScale(), sprite->getFrameOffset());
            }

            // model slides under the entity so its origin lands where the entity stands
            object->getTransform().setMatrix(glm::translate(matrix, -renderable.origin));
        }

        // scene shares light component holds, entity pose places it
        if (auto* lightComponent = world.get<LightComponent>(entity); lightComponent && lightComponent->light)
        {
            lightComponent->light->getTransform().setMatrix(matrix);
            m_scene.addLight(lightComponent->light);
        }
    }

    applyEnvironment(world);
}

// scene paints its own backdrop, one without environment keeps what stood there
void RenderSystem::applyEnvironment(World& world)
{
    for (Entity entity : world.getEntities())
    {
        const auto* component = world.get<EnvironmentComponent>(entity);

        if (!component)
        {
            continue;
        }

        const std::shared_ptr<BulletRender::render::CubeMap> sky = component->getSkybox();

        BulletRender::render::Renderer::setBackgroundColor(glm::vec4(component->getClearColor(), 1.0f));

        if (m_skybox)
        {
            m_skybox->setCubeMap(sky);
            m_skybox->setEnabled(sky != nullptr);
        }

        return;
    }

    if (m_skybox)
    {
        m_skybox->setEnabled(false);
    }
}

} // namespace systems
} // namespace ecs
} // namespace BulletEngine
