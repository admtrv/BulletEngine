/*
 * RenderSystem.cpp
 */

#include "RenderSystem.h"

namespace BulletEngine {
namespace ecs {
namespace systems {

// what the component overrides lands on the model, what it leaves alone keeps its mtl value
static void apply(const BulletRender::render::Material& from, BulletRender::render::Material& to)
{
    if (from.hasColor())      { to.setColor(from.getColor()); }
    if (from.hasSpecular())   { to.setSpecular(from.getSpecular()); }
    if (from.hasShininess())  { to.setShininess(from.getShininess()); }
    if (from.hasEmissive())   { to.setEmissive(from.getEmissive()); }

    for (const BulletRender::render::TextureSlot& slot : from.getTextures())
    {
        to.setTexture(slot.uniformName, slot.texture, slot.unit);
    }
}

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

            // only terms the component set reach the object, rest stays as the model loaded it
            apply(renderable->material, object->getMaterial());
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
