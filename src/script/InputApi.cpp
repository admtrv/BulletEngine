/*
 * InputApi.cpp
 */

#include "Api.h"

#include "utils/Input.h"

#include <iostream>
#include <string>
#include <unordered_set>

namespace BulletEngine {
namespace script {

void installInput(sol::environment& environment)
{
    sol::table input = environment.create_named("input");

    // key by name, as the enum spells it
    input["isKeyDown"] = [](const std::string& name) {
        const std::optional<BulletRender::utils::InputKey> key = BulletRender::utils::toKey(name);

        if (!key)
        {
            // asked every frame, so the name is reported once
            static std::unordered_set<std::string> reported;

            if (reported.insert(name).second)
            {
                std::cerr << "script asked for unknown key: " << name << '\n';
            }

            return false;
        }

        return BulletRender::utils::Input::isKeyDown(*key);
    };
}

} // namespace script
} // namespace BulletEngine
