/*
 * Phases.h
 */

#pragma once

#include <cstdint>

namespace BulletEngine {
namespace app {

// slot inside one frame
enum class Phase : uint8_t {
    PreUpdate,      // input, camera
    FixedUpdate,    // physics
    Update,         // logic
    PostUpdate,     // followers catch up
    Render,         // scene, gizmos
    RenderUi        // game ui, hud
};

} // namespace app
} // namespace BulletEngine
