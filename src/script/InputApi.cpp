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

// name script spelled, complained about once so typo cannot flood console
static std::optional<BulletRender::utils::InputKey> findKey(const std::string& name)
{
    const std::optional<BulletRender::utils::InputKey> key = BulletRender::utils::toKey(name);

    if (!key)
    {
        static std::unordered_set<std::string> reported;

        if (reported.insert(name).second)
        {
            std::cerr << "script asked for unknown key: " << name << '\n';
        }
    }

    return key;
}

void installInput(sol::environment& environment)
{
    sol::table input = environment.create_named("input");

    // key by name, as enum spells it, asked every frame so bad name is reported once
    input["isKeyDown"] = [](const std::string& name) {
        const std::optional<BulletRender::utils::InputKey> key = findKey(name);
        return key && BulletRender::utils::Input::isKeyDown(*key);
    };

    // down this frame only, what script uses for one shot actions
    input["isKeyPressed"] = [](const std::string& name) {
        const std::optional<BulletRender::utils::InputKey> key = findKey(name);
        return key && BulletRender::utils::Input::isKeyPressed(*key);
    };
}

} // namespace script
} // namespace BulletEngine
