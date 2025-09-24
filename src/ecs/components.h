/*
 * components.h
 */

#pragma once

#include "ecs.h"
#include "scene/transform.h"
#include "scene/model.h"
#include "render/material.h"

namespace luchengine {
namespace ecs {

class TransformComponent : public Component {
public:
    luchrender::scene::Transform transform;
};

class RenderableComponent : public Component {
public:
    luchrender::scene::Model* model = nullptr;
    luchrender::render::Material material;
};

} // namespace ecs
} // namespace luchengine