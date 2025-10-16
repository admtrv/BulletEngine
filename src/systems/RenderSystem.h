/*
 * RenderSystem.h
 */

#pragma once

#include "ecs/Ecs.h"
#include "ecs/Components.h"

#include "scene/Scene.h"

namespace BulletEngine {
namespace systems {

// render adapter
class RenderSystem {
public:
    explicit RenderSystem(BulletRender::scene::Scene& scene) : m_scene(scene) {}

    void rebuild(ecs::World& world);

private:
    BulletRender::scene::Scene& m_scene;
};

} // namespace systems
} // namespace BulletEngine
