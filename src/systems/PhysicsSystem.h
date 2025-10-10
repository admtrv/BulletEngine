/*
 * PhysicsSystem.h
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

namespace luchengine {
namespace systems {

// physics adapter
class PhysicsSystem {
public:
    explicit PhysicsSystem(luchphysic::math::IIntegrator& integrator);

    void update(ecs::World& world, float dt);

private:
    luchphysic::math::IIntegrator& m_integrator;
};

} // namespace systems
} // namespace luchengine
