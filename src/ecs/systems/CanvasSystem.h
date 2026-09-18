/*
 * CanvasSystem.h
 */

#pragma once

#include "ecs/Ecs.h"

// fwd
namespace BulletRender::render { class Canvas; }

namespace BulletEngine {
namespace ecs {
namespace systems {

class ScriptSystem;

// lets scripted entities draw their interface, lowest order first
class CanvasSystem {
public:
    explicit CanvasSystem(ScriptSystem& scripts);

    void draw(World& world, BulletRender::render::Canvas& canvas);

private:
    ScriptSystem& m_scripts;
};

} // namespace systems
} // namespace ecs
} // namespace BulletEngine
