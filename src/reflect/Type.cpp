/*
 * Type.cpp
 */

#include "Type.h"

#include <algorithm>
#include <cctype>

namespace BulletEngine {
namespace reflect {

constexpr std::string_view COMPONENT_SUFFIX = "Component";

std::string toLabel(std::string_view name)
{
    if (name.size() > COMPONENT_SUFFIX.size() && name.ends_with(COMPONENT_SUFFIX))
    {
        name.remove_suffix(COMPONENT_SUFFIX.size());
    }

    std::string label;
    label.reserve(name.size() + 4);

    for (size_t i = 0; i < name.size(); i++)
    {
        const char c = name[i];

        // a capital starts a new word, unless it continues an acronym
        if (i > 0 && std::isupper(static_cast<unsigned char>(c)) && !std::isupper(static_cast<unsigned char>(name[i - 1])))
        {
            label += ' ';
        }

        label += i == 0 ? static_cast<char>(std::toupper(static_cast<unsigned char>(c))) : c;
    }

    return label;
}

// own fields first, what the type is reads before how it behaves
std::vector<const Field*> Type::getAllFields() const
{
    std::vector<const Field*> fields;

    for (const Field& field : m_fields)
    {
        fields.push_back(&field);
    }

    if (m_base)
    {
        const std::vector<const Field*> inherited = m_base->getAllFields();
        fields.insert(fields.end(), inherited.begin(), inherited.end());
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
