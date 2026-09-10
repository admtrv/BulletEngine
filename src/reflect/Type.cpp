/*
 * Type.cpp
 */

#include "Type.h"

#include <algorithm>

namespace BulletEngine {
namespace reflect {

std::vector<const Field*> Type::getAllFields() const
{
    std::vector<const Field*> fields;

    if (m_base)
    {
        fields = m_base->getAllFields();
    }

    for (const Field& field : m_fields)
    {
        fields.push_back(&field);
    }

    return fields;
}

const Field* Type::findField(std::string_view name) const
{
    const auto it = std::find_if(m_fields.begin(), m_fields.end(),
        [name](const Field& field) { return field.getName() == name; });

    if (it != m_fields.end())
    {
        return &(*it);
    }

    return m_base ? m_base->findField(name) : nullptr;
}

bool Type::derivesFrom(const Type& type) const
{
    for (const Type* current = m_base; current; current = current->m_base)
    {
        if (current == &type)
        {
            return true;
        }
    }

    return false;
}

} // namespace reflect
} // namespace BulletEngine
