/*
 * PhysicsApi.cpp
 */

#include "Api.h"

#include "ecs/Components.h"

#include <glm/glm.hpp>

namespace BulletEngine {
namespace script {

// body of this entity, none while it carries no rigid body
static BulletPhysics::dynamics::RigidBody* bodyOf(ecs::World& world, ecs::Entity entity)
{
    auto* component = world.get<ecs::RigidBodyComponent>(entity);
    return component ? &component->body : nullptr;
}

static BulletPhysics::math::Vec3 toPhysics(const glm::vec3& value)
{
    return {value.x, value.y, value.z};
}

void installPhysics(sol::environment& environment, ecs::World& world, ecs::Entity entity)
{
    sol::table body = environment.create_named("body");

    // steady push, meant for onFixedUpdate
    body["addForce"] = [&world, entity](const glm::vec3& force) {
        if (auto* rigid = bodyOf(world, entity))
        {
            rigid->addForce(toPhysics(force));
        }
    };

    body["addTorque"] = [&world, entity](const glm::vec3& torque) {
        if (auto* rigid = bodyOf(world, entity))
        {
            rigid->addTorque(toPhysics(torque));
        }
    };

    // one off kick, jumps and hits
    body["addImpulse"] = [&world, entity](const glm::vec3& impulse) {
        auto* rigid = bodyOf(world, entity);

        // inverse mass is zero for bodies the simulation never moves
        if (!rigid || rigid->getInverseMass() == 0.0)
        {
            return;
        }

        rigid->setVelocity(rigid->getVelocity() + toPhysics(impulse) * rigid->getInverseMass());
    };
}

} // namespace script
} // namespace BulletEngine
