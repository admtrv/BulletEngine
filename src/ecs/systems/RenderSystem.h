/*
 * RenderSystem.h
 */

#pragma once

#include "ecs/Ecs.h"
#include "ecs/Components.h"

#include "render/passes/SkyBox.h"
#include "scene/Scene.h"

namespace BulletEngine {
namespace ecs {
namespace systems {

class RenderSystem {
public:
    RenderSystem(BulletRender::scene::Scene& scene, std::shared_ptr<BulletRender::render::SkyBox> skybox);

    void render(World& world);

private:
    void applyEnvironment(World& world);

    BulletRender::scene::Scene& m_scene;
    std::shared_ptr<BulletRender::render::SkyBox> m_skybox;
};

} // namespace systems
} // namespace ecs
} // namespace BulletEngine
