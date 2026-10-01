/*
 * Inspector.cpp
 */

#include "interface/Editor.h"

#include "assets/Loaders.h"
#include "ecs/Components.h"
#include "interface/elements/Widgets.h"
#include "project/Project.h"
#include "reflect/Annotations.h"
#include "reflect/Registry.h"
#include "reflect/Type.h"

#include "imgui.h"

#include <cstdio>
#include <iostream>

#include <typeindex>

namespace BulletEngine {
namespace interface {

constexpr float DRAG_SPEED_DEFAULT = 0.05f;
constexpr float DRAG_SPEED_ROTATION = 0.5f;
constexpr float DRAG_LIMIT = 10000.0f;
constexpr int MASK_BITS = 32;                   // layers physics carries, one per bit
constexpr size_t AXIS_COUNT = 3;                // fields axes row claims

void Editor::drawInspector()
{
    if (!m_showInspector)
    {
        return;
    }

    ImGui::Begin(INSPECTOR_PANEL, &m_showInspector);

    if (m_selection == ecs::INVALID_ENTITY || !m_world.isAlive(m_selection))
    {
        ImGui::End();
        return;
    }

    drawAddMenu();

    if (auto* identity = m_world.get<ecs::IdentityComponent>(m_selection))
    {
        BulletRender::interface::textField("Name", identity->name);
        BulletRender::interface::textField("Tag", identity->tag);
    }

    ImGui::Separator();

    for (const std::unique_ptr<ecs::Component>& component : m_world.getComponents(m_selection))
    {
        const std::type_index index = std::type_index(typeid(*component));
        const reflect::Type* type = reflect::Registry::instance().find(index);

        if (!type || type->isHidden())
        {
            continue;
        }

        if (m_focusComponent == type)
        {
            ImGui::SetNextItemOpen(true);
            ImGui::SetScrollHereY();

            m_focusComponent = nullptr;
        }

        const bool open = ImGui::CollapsingHeader(type->getLabel().c_str(), ImGuiTreeNodeFlags_DefaultOpen);

        // menu opens over header, and components share field names
        ImGui::PushID(type);

        BulletRender::interface::contextMenu("component", [&]() {
            if (ImGui::MenuItem("Delete"))
            {
                removeComponent(m_selection, index);
            }
        });

        const bool edited = open && drawFields(*type, component.get());

        ImGui::PopID();

        // body carries pose, either side has to reach it
        if (edited && dynamic_cast<const ecs::TransformComponent*>(component.get()))
        {
            syncBody(m_selection);
        }
        else if (edited && dynamic_cast<const ecs::ColliderComponent*>(component.get()))
        {
            syncCollider(m_selection);
        }
    }

    ImGui::End();
}

// every component type known to registry
void Editor::drawAddMenu()
{
    const reflect::Type* base = reflect::Registry::instance().find<ecs::Component>();

    if (!base)
    {
        return;
    }

    if (ImGui::Button("Add Component", {ImGui::GetContentRegionAvail().x, 0.0f}))
    {
        ImGui::OpenPopup("components");
    }

    if (!ImGui::BeginPopup("components"))
    {
        return;
    }

    for (const reflect::Type* type : reflect::Registry::instance().getDerived(*base))
    {
        if (type->isHidden())
        {
            continue;
        }

        if (!ImGui::MenuItem(type->getLabel().c_str()))
        {
            continue;
        }

        // type already there scrolls into view instead of being added twice
        if (m_world.has(m_selection, type->getIndex()))
        {
            m_focusComponent = type;
        }
        else
        {
            addComponent(m_selection, *type);
        }
    }

    ImGui::EndPopup();
}

void Editor::addComponent(ecs::Entity entity, const reflect::Type& type)
{
    m_pendingAdd.push_back({entity, &type});
}

void Editor::removeComponent(ecs::Entity entity, std::type_index type)
{
    m_pendingRemove.push_back({entity, type});
}

// physics owns pose of body, edit must reach it
void Editor::syncBody(ecs::Entity entity)
{
    auto* transform = m_world.get<ecs::TransformComponent>(entity);
    auto* rigidBody = m_world.get<ecs::RigidBodyComponent>(entity);

    if (!transform || !rigidBody)
    {
        return;
    }

    const glm::vec3 position = transform->transform.getPosition();
    const glm::quat rotation = transform->transform.getRotation();

    rigidBody->body.setPosition({position.x, position.y, position.z});
    rigidBody->body.setOrientation({rotation.w, rotation.x, rotation.y, rotation.z});

    // shape follows the body, keeping whatever offset of its own it was given
    auto* collider = m_world.get<ecs::ColliderComponent>(entity);

    if (!collider || !collider->collider)
    {
        return;
    }

    collider->collider->place(rigidBody->body.getPosition(), rigidBody->body.getOrientation());

    // shape with no facing keeps neither turn nor size
    if (!collider->collider->isOrientable())
    {
        rigidBody->body.setOrientation({});

        transform->transform.setRotation({1.0f, 0.0f, 0.0f, 0.0f});
        transform->transform.setLocalScale(glm::vec3{1.0f});
    }
}

// shape edited by hand is put back where whatever carries it says it belongs
void Editor::syncCollider(ecs::Entity entity)
{
    auto* collider = m_world.get<ecs::ColliderComponent>(entity);
    auto* transform = m_world.get<ecs::TransformComponent>(entity);

    if (!collider || !collider->collider || !transform)
    {
        return;
    }

    const glm::vec3 position = transform->transform.getPosition();
    const glm::quat rotation = transform->transform.getRotation();

    collider->collider->place({position.x, position.y, position.z}, {rotation.w, rotation.x, rotation.y, rotation.z});
}

// one value of any supported type
bool Editor::drawValue(const reflect::Field& field, void* instance, const char* name)
{
    const reflect::Value value = field.get(instance);

    bool changed = false;

    switch (field.getType())
    {
        case reflect::ValueType::Bool:
        {
            bool v = value.get<bool>();
            if (BulletRender::interface::checkboxField(name, v)) { field.set(instance, v); changed = true; }
            break;
        }
        case reflect::ValueType::Int:
        {
            int v = value.get<int>();

            if (field.isEnum())
            {
                const std::vector<std::string>& options = field.getOptions();

                std::vector<const char*> names;
                names.reserve(options.size());

                for (const std::string& option : options)
                {
                    names.push_back(option.c_str());
                }

                if (BulletRender::interface::comboField(name, v, names.data(), static_cast<int>(names.size())))
                {
                    field.set(instance, v);
                    changed = true;
                }

                break;
            }

            if (field.has<reflect::Bits>())
            {
                unsigned bits = static_cast<unsigned>(v);

                if (BulletRender::interface::bitsField(name, bits, MASK_BITS))
                {
                    field.set(instance, static_cast<int>(bits));
                    changed = true;
                }

                break;
            }

            // field naming no range holds whatever fits
            const reflect::Range* range = field.metadata<reflect::Range>();

            const int intMin = range ? int(range->min) : -int(DRAG_LIMIT);
            const int intMax = range ? int(range->max) : int(DRAG_LIMIT);

            if (BulletRender::interface::dragScalarField(name, v, intMin, intMax, "%d")) { field.set(instance, v); changed = true; }
            break;
        }
        case reflect::ValueType::Float:
        {
            float v = value.get<float>();

            const reflect::Range* range = field.metadata<reflect::Range>();

            const float min = range ? range->min : -DRAG_LIMIT;
            const float max = range ? range->max : DRAG_LIMIT;

            if (BulletRender::interface::dragScalarField(name, v, min, max, "%.3f")) { field.set(instance, v); changed = true; }
            break;
        }
        case reflect::ValueType::String:
        {
            std::string v = value.get<std::string>();

            if (field.has<reflect::Asset>())
            {
                // state types path, it never mirrors key preset carries
                BulletRender::interface::AssetFieldState& slot = m_assetPaths[field.getName()];

                // assets live under project, no sense starting anywhere else
                if (slot.browser.root[0] == '\0')
                {
                    std::snprintf(slot.browser.root, sizeof(slot.browser.root), "%s",
                                  project::Project::instance().getRoot().c_str());
                }

                switch (BulletRender::interface::assetField(name, assets::toLabel(v).c_str(), !v.empty(), slot, ASSET_DRAG_TYPE))
                {
                    case BulletRender::interface::AssetAction::Clear:
                        field.set(instance, std::string{});
                        slot.error.clear();
                        changed = true;
                        break;

                    case BulletRender::interface::AssetAction::Load:
                        field.set(instance, std::string(slot.path));

                        // key only sticks when asset behind it loaded
                        slot.error = field.get(instance).get<std::string>() == slot.path
                            ? std::string{}
                            : "failed to load " + std::string(slot.path);

                        changed = true;
                        break;

                    default:
                        break;
                }

                break;
            }

            if (BulletRender::interface::textField(name, v))
            {
                field.set(instance, v);
                changed = true;
            }

            break;
        }
        case reflect::ValueType::Quat:
        {
            // quaternion edits as euler degrees
            glm::vec3 angles = glm::degrees(glm::eulerAngles(value.get<glm::quat>()));

            if (BulletRender::interface::dragVector3(name, angles, DRAG_SPEED_ROTATION, -360.0f, 360.0f, "%.1f"))
            {
                field.set(instance, glm::quat(glm::radians(angles)));
                changed = true;
            }

            break;
        }
        case reflect::ValueType::Vec2:
        {
            glm::vec2 v = value.get<glm::vec2>();

            const reflect::Speed* step = field.metadata<reflect::Speed>();
            const float speed = step ? step->value : DRAG_SPEED_DEFAULT;

            if (BulletRender::interface::dragVector2(name, v, speed, -DRAG_LIMIT, DRAG_LIMIT, "%.2f")) { field.set(instance, v); changed = true; }
            break;
        }
        case reflect::ValueType::Vec3:
        {
            glm::vec3 v = value.get<glm::vec3>();

            if (field.has<reflect::Color>())
            {
                if (BulletRender::interface::dragColor3(name, v)) { field.set(instance, v); changed = true; }
                break;
            }

            const reflect::Speed* step = field.metadata<reflect::Speed>();
            const float speed = step ? step->value : DRAG_SPEED_DEFAULT;

            if (BulletRender::interface::dragVector3(name, v, speed, -DRAG_LIMIT, DRAG_LIMIT, "%.2f")) { field.set(instance, v); changed = true; }
            break;
        }
        case reflect::ValueType::Vec4:
        {
            glm::vec4 v = value.get<glm::vec4>();
            glm::vec3 rgb{v.x, v.y, v.z};

            if (BulletRender::interface::dragColor3(name, rgb))
            {
                field.set(instance, glm::vec4{rgb.x, rgb.y, rgb.z, v.w});
                changed = true;
            }

            break;
        }
        default:
            BulletRender::interface::statRow(name, "%s", reflect::toString(field.getType()));
            break;
    }

    return changed;
}

// every scalar type declares, keyed by name
void Editor::collectValues(const reflect::Type& type, const void* instance, ValueMap& out) const
{
    for (const reflect::Field* field : type.getAllFields())
    {
        if (field->getKind() == reflect::FieldKind::Object)
        {
            const reflect::Type* nested = nullptr;

            if (const void* object = field->resolve(instance, &nested); nested && object)
            {
                collectValues(*nested, object, out);
            }

            continue;
        }

        if (!field->isReadOnly())
        {
            out.emplace(field->getName(), field->get(instance));
        }
    }
}

void Editor::applyValues(const reflect::Type& type, void* instance, const ValueMap& values) const
{
    for (const reflect::Field* field : type.getAllFields())
    {
        if (field->getKind() == reflect::FieldKind::Object)
        {
            const reflect::Type* nested = nullptr;

            if (void* object = field->resolve(instance, &nested); nested && object)
            {
                applyValues(*nested, object, values);
            }

            continue;
        }

        if (field->isReadOnly())
        {
            continue;
        }

        if (const auto it = values.find(field->getName()); it != values.end())
        {
            field->set(instance, it->second);
        }
    }
}

// picks concrete type polymorphic field holds
bool Editor::drawObjectType(const reflect::Field& field, void* instance, const reflect::Type* current)
{
    const reflect::Type* base = field.getBaseType();

    if (!base || !field.isBuildable())
    {
        ImGui::TextUnformatted(field.getLabel().c_str());
        return false;
    }

    const std::vector<const reflect::Type*> options = reflect::Registry::instance().getDerived(*base);

    std::vector<const char*> names;
    names.reserve(options.size());

    // nothing chosen yet, so row starts empty rather than on first type
    int selected = -1;

    for (size_t i = 0; i < options.size(); i++)
    {
        names.push_back(options[i]->getLabel().c_str());

        if (options[i] == current)
        {
            selected = static_cast<int>(i);
        }
    }

    const int previous = selected;

    if (!BulletRender::interface::comboField(field.getLabel().c_str(), selected, names.data(), static_cast<int>(names.size())))
    {
        return false;
    }

    if (selected == previous || selected < 0)
    {
        return false;
    }

    // physics holds raw pointer, old object leaves world first
    m_physics.detach(m_world, m_selection);
    field.build(instance, *options[selected]);

    return true;
}

bool Editor::drawField(const reflect::Field& field, void* instance, const char* name)
{
    if (field.has<reflect::Hidden>() || !field.isShown(instance))
    {
        return false;
    }

    if (!name)
    {
        name = field.getLabel().c_str();
    }

    if (field.getKind() == reflect::FieldKind::Object)
    {
        const reflect::Type* nested = nullptr;
        void* object = field.resolve(instance, &nested);

        // empty field cannot be built into, only reported
        if (!object && !field.isBuildable())
        {
            BulletRender::interface::statRow(name, "None");
            return false;
        }

        bool changed = false;

        // inline object draws no type row of its own
        if (!field.has<reflect::Inline>() && drawObjectType(field, instance, object ? nested : nullptr))
        {
            changed = true;
            object = field.resolve(instance, &nested);
        }

        if (nested && object)
        {
            // two nested objects of one component may carry the same field names
            ImGui::PushID(name);

            // buildable object splits itself, plain one sits under its label
            if (field.isBuildable())
            {
                changed |= drawFields(*nested, object, true);
            }
            else if (field.has<reflect::Inline>())
            {
                changed |= drawFields(*nested, object, false, name);
            }
            else
            {
                ImGui::Indent();
                changed |= drawFields(*nested, object);
                ImGui::Unindent();
            }

            ImGui::PopID();
        }

        return changed;
    }

    if (field.isReadOnly())
    {
        BulletRender::interface::statRow(name, "read only");
        return false;
    }

    return drawValue(field, instance, name);
}

// fields arrive either as values or as pointers, one walk serves both
static const reflect::Field* at(const reflect::Field* fields, size_t index) { return fields + index; }
static const reflect::Field* at(const reflect::Field* const* fields, size_t index) { return fields[index]; }

// three bool fields first one claims, drawn as one x y z row
bool Editor::drawAxes(const reflect::Field* const axes[AXIS_COUNT], void* instance)
{
    bool axis[AXIS_COUNT] = {};

    for (size_t i = 0; i < AXIS_COUNT; i++)
    {
        axis[i] = axes[i]->get(instance).get<bool>();
    }

    const reflect::Axes* caption = axes[0]->metadata<reflect::Axes>();

    if (!BulletRender::interface::checkboxAxes(caption->text, axis[0], axis[1], axis[2]))
    {
        return false;
    }

    for (size_t i = 0; i < AXIS_COUNT; i++)
    {
        axes[i]->set(instance, reflect::Value(axis[i]));
    }

    return true;
}

template<class F>
bool Editor::drawRange(F fields, size_t count, void* instance, const char* first)
{
    bool changed = false;

    for (size_t i = 0; i < count; i++)
    {
        const reflect::Field* field = at(fields, i);

        // claimed triple leaves only its own row behind, broken one falls back to plain fields
        if (field->has<reflect::Axes>() && i + AXIS_COUNT <= count)
        {
            const reflect::Field* axes[AXIS_COUNT];

            for (size_t axis = 0; axis < AXIS_COUNT; axis++)
            {
                axes[axis] = at(fields, i + axis);
            }

            changed |= drawAxes(axes, instance);
            i += AXIS_COUNT - 1;
            continue;
        }

        // inline object hands its name to first field
        changed |= drawField(*field, instance, i == 0 ? first : nullptr);
    }

    return changed;
}

bool Editor::drawFields(const reflect::Type& type, void* instance, bool splitOwn, const char* first)
{
    if (!splitOwn)
    {
        const std::vector<const reflect::Field*> fields = type.getAllFields();
        return drawRange(fields.data(), fields.size(), instance, first);
    }

    // what only this type has belongs to row above, indented under it
    const std::vector<reflect::Field>& own = type.getFields();

    ImGui::Indent();
    bool changed = drawRange(own.data(), own.size(), instance);
    ImGui::Unindent();

    // what base declares describes component itself, back on its level
    if (const reflect::Type* base = type.getBase())
    {
        changed |= drawFields(*base, instance);
    }

    return changed;
}

} // namespace interface
} // namespace BulletEngine
