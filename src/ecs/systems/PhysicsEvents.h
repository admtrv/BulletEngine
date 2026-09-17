/*
 * PhysicsEvents.h
 */

#pragma once

#include "ecs/Ecs.h"

#include <glm/glm.hpp>

#include <cstdint>

namespace BulletEngine {
namespace ecs {
namespace systems {

// what happened between two entities, as simulation saw it
enum class ContactPhase : uint8_t {
    Begin,
    End
};

// one side of contact, pair is reported to both entities in turn
struct ContactEvent {
    ContactPhase phase = ContactPhase::Begin;

    Entity self = INVALID_ENTITY;
    Entity other = INVALID_ENTITY;

    // where they met and which way they pushed apart, zero once contact ends
    glm::vec3 point{};
    glm::vec3 normal{};      // points away from self
    float depth = 0.0f;      // how far they overlapped

    // either collider may be trigger, which reports but never pushes
    bool trigger = false;
};

// what ray met, empty when it met nothing
struct RayResult {
    bool hit = false;

    Entity entity = INVALID_ENTITY;

    glm::vec3 point{};
    glm::vec3 normal{};     // faces ray
    float distance = 0.0f;
};

} // namespace systems
} // namespace ecs
} // namespace BulletEngine
