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
    RenderUi,       // game ui, hud

    Count
};

inline const char* toString(Phase phase)
{
    switch (phase)
    {
        case Phase::PreUpdate:   return "PreUpdate";
        case Phase::FixedUpdate: return "FixedUpdate";
        case Phase::Update:      return "Update";
        case Phase::PostUpdate:  return "PostUpdate";
        case Phase::Render:      return "Render";
        case Phase::RenderUi:    return "RenderUi";
        default:                 return "Unknown";
    }
}

} // namespace app
} // namespace BulletEngine
