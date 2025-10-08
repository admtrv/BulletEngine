/*
 * Components.h
 */

#pragma once

#include "Ecs.h"
#include "scene/Transform.h"
#include "scene/Model.h"
#include "render/Material.h"

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