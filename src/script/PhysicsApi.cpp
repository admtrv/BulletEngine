/*
 * PhysicsApi.cpp
 */

#include "Api.h"

#include "ecs/Components.h"
#include "ecs/systems/PhysicsSystem.h"

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

void installPhysics(sol::environment& environment, ecs::World& world, ecs::Entity entity, ecs::systems::PhysicsSystem& simulation)
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

        // inverse mass is zero for bodies simulation never moves
        if (!rigid || rigid->getInverseMass() == 0.0)
        {
            return;
        }

        rigid->setVelocity(rigid->getVelocity() + toPhysics(impulse) * rigid->getInverseMass());
    };

    sol::table physics = environment.create_named("physics");

    // what ray met first, or nothing, layers narrow search
    physics["raycast"] = [&simulation](const glm::vec3& origin, const glm::vec3& direction, float distance, sol::optional<unsigned> mask, sol::this_state state) -> sol::object {
        const ecs::systems::RayResult result = simulation.raycast(origin, glm::normalize(direction), distance, mask.value_or(~0u));

        if (!result.hit)
        {
            return sol::lua_nil;
        }

        sol::table found = sol::state_view(state).create_table();

        found["entity"] = result.entity;
        found["point"] = result.point;
        found["normal"] = result.normal;
        found["distance"] = result.distance;

        return found;
    };
}

} // namespace script
} // namespace BulletEngine
