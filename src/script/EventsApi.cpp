/*
 * EventsApi.cpp
 */

#include "Api.h"

#include "script/Events.h"

#include <string>

namespace BulletEngine {
namespace script {

void installEvents(sol::environment& environment, EventBus& events, ecs::Entity entity)
{
    sol::table table = environment.create_named("events");

    // handler belongs to this entity, so it goes when entity does
    table["on"] = [&events, entity](const std::string& name, sol::protected_function handler) {
        events.listen(name, entity, std::move(handler));
    };

    // everyone listening runs before this returns
    table["emit"] = [&events](const std::string& name, sol::variadic_args args) {
        events.emit(name, args);
    };
}

} // namespace script
} // namespace BulletEngine
