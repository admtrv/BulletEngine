/*
 * render_system.h
 */
#pragma once

#include "ecs/ecs.h"
#include "scene/scene.h"
#include "ecs/components.h"

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
