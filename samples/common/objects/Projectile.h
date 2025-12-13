/*
 * Projectile.h
 */

#pragma once

#include "ecs/Ecs.h"
#include "ecs/Components.h"
#include "scene/Model.h"
#include "render/Shader.h"
#include "collision/BoxCollider.h"
#include "dynamics/PhysicsWorld.h"
#include "dynamics/forces/drag/DragModel.h"
#include "common/Components.h"

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
    float m_mass = 0.01f;
    float m_diameter = 0.00762f;
    BulletPhysic::dynamics::forces::drag::DragCurveModel m_dragModel = BulletPhysic::dynamics::forces::drag::DragCurveModel::G7;

    float m_modelDiameter = 1.0f;
    float m_modelLength = 3.32f;

    // initial position
    float m_initialPosX = 0.0f;
    float m_initialPosY = 1.5f;
    float m_initialPosZ = 0.0f;

    // launch parameters
    float m_launchSpeed = 15.0f;
    float m_launchElevationDeg = 25.0f;
    float m_launchAzimuthDeg = 90.0f;

    // material color
    float m_colorR = 1.0f;
    float m_colorG = 0.5f;
    float m_colorB = 0.0f;

    // assets
    std::string m_modelPath = "assets/models/g7.obj";
    std::string m_vertexShaderPath = "assets/shaders/normal.vert.glsl";
    std::string m_fragmentShaderPath = "assets/shaders/normal.frag.glsl";

    // collider
    bool m_showCollider = false;
};

} // namespace objects
} // namespace BulletEngine
