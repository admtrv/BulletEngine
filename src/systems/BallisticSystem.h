/*
 * BallisticSystem.h
 */

#pragma once

#include "ecs/Ecs.h"
#include "ecs/Components.h"
#include "math/Integrator.h"

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

private:
    BulletPhysic::math::IIntegrator& m_integrator;
};

} // namespace systems
} // namespace BulletEngine
