/*
 * RenderSystem.h
 */

#pragma once

#include "ecs/Ecs.h"
#include "ecs/Components.h"

#include "scene/Scene.h"

namespace luchengine {
namespace systems {

// adapter ECS -> luchrender::scene::Scene
class RenderSystem {
public:
    explicit RenderSystem(luchrender::scene::Scene& scene) : m_scene(scene) {}

    void rebuild(ecs::World& world);

private:
    luchrender::scene::Scene& m_scene;
};

} // namespace systems
} // namespace luchengine
