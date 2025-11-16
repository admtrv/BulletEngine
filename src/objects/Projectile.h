/*
 * Projectile.h
 */

#pragma once

#include "ecs/Ecs.h"
#include "ecs/Components.h"

#include "scene/Model.h"
#include "render/Shader.h"

#include "collision/BoxCollider.h"

#include <vector>

namespace BulletEngine {
namespace objects {


// projectile abstract entity factory
class Projectile {
public:
    static ecs::Entity launch(ecs::World& world);

    static std::vector<ecs::Entity> fired;

private:
    // physics
    float m_mass = 0.05f;

    // initial position
    float m_initialPosX = 0.0f;
    float m_initialPosY = 1.5f;
    float m_initialPosZ = 0.0f;

    // launch parameters
    float m_launchSpeed = 25.0f;
    float m_launchElevationDeg = 25.0f;
    float m_launchAzimuthDeg = 90.0f;


    // collider dimensions
    float m_colliderX = 0.41f;
    float m_colliderY = 2.42f;
    float m_colliderZ = 0.41f;

    // material color
    float m_colorR = 1.0f;
    float m_colorG = 0.5f;
    float m_colorB = 0.0f;

    // assets
    std::string m_modelPath = "assets/models/bullet.obj";
    std::string m_vertexShaderPath = "assets/shaders/normal.vert.glsl";
    std::string m_fragmentShaderPath = "assets/shaders/normal.frag.glsl";
};

} // namespace objects
} // namespace BulletEngine
