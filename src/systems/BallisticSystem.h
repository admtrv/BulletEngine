/*
 * BallisticSystem.h
 */

#pragma once

#include "ecs/Ecs.h"
#include "ecs/Components.h"
#include "math/Integrator.h"

// BulletPhysic
#include "PresetManager.h"

#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <cmath>

namespace BulletEngine {
namespace systems {

// physics adapter
class BallisticSystem {
public:
    explicit BallisticSystem(BulletPhysic::math::IIntegrator& integrator);

    void update(ecs::World& world, float dt);

    BulletPhysic::dynamics::forces::ForceRegistry& getForceRegistry() { return m_forceRegistry;}

    void setRealismLevel(BulletPhysic::preset::RealismLevel level, float projectileArea = 0.01f);
    void setWindVelocity(const BulletPhysic::math::Vec3& windVel);

private:
    BulletPhysic::math::IIntegrator& m_integrator;
    BulletPhysic::dynamics::forces::ForceRegistry m_forceRegistry;
};

} // namespace systems
} // namespace BulletEngine
