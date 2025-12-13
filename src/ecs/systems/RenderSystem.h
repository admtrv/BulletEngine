/*
 * RenderSystem.h
 */

#pragma once

#include "ecs/Ecs.h"
#include "ecs/Components.h"

#include "scene/Scene.h"

namespace BulletEngine {
namespace ecs {
namespace systems {

class RenderSystemBase {
public:
    explicit RenderSystemBase(BulletRender::scene::Scene& scene);
    virtual ~RenderSystemBase() = default;

    void render(World& world);

protected:
    // hooks
    virtual void onObjectRender(World&, Entity, BulletRender::scene::SceneObject&) {}
    virtual void onColliderRender(World&, Entity, BulletRender::scene::SceneObject&) {}

    BulletRender::scene::Scene& m_scene;
};


} // namespace systems
} // namespace ecs
} // namespace BulletEngine
