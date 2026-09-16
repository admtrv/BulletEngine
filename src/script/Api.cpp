/*
 * Api.cpp
 */

#include "Api.h"

#include "reflect/Registry.h"

#include <iostream>
#include <unordered_set>

namespace BulletEngine {
namespace script {

const reflect::Type* findType(const std::string& name)
{
    const reflect::Type* type = reflect::Registry::instance().find(name);

    if (!type)
    {
        static std::unordered_set<std::string> reported;

        if (reported.insert(name).second)
        {
            std::cerr << "script asked for unknown component: " << name << '\n';
        }
    }

    return type;
}

} // namespace script
} // namespace BulletEngine
