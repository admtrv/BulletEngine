/*
 * Hierarchy.cpp
 */

#include "interface/Editor.h"

#include "ecs/Components.h"
#include "ecs/systems/HierarchySystem.h"
#include "interface/elements/Widgets.h"
#include "reflect/Type.h"

#include "assets/Registry.h"
#include "scene/models/Model.h"

#include "collision/collider/BoxCollider.h"
#include "collision/collider/SphereCollider.h"
#include "dynamics/body/Inertia.h"
#include "scene/Serializer.h"

#include "imgui.h"

#include <algorithm>

namespace BulletEngine {
namespace interface {

constexpr const char* ENTITY_DRAG_TYPE = "BE_ENTITY";
constexpr int MAX_TREE_DEPTH = 64;
constexpr double SPHERE_RADIUS = 0.5;

void Editor::drawHierarchy()
{
    if (!m_showHierarchy)
    {
        return;
    }

    ImGui::Begin(HIERARCHY_PANEL, &m_showHierarchy);

    if (ImGui::Button("Add Entity", {ImGui::GetContentRegionAvail().x, 0.0f}))
    {
        ImGui::OpenPopup("entities");
    }

    if (ImGui::BeginPopup("entities"))
    {
        if (ImGui::MenuItem("Box"))
        {
            createEntity(Preset::Box);
        }

        if (ImGui::MenuItem("Sphere"))
        {
            createEntity(Preset::Sphere);
        }

        ImGui::Separator();

        if (ImGui::MenuItem("Empty"))
        {
            createEntity(Preset::Empty);
        }

        ImGui::EndPopup();
    }

    // own tree, guides start fresh
    m_tree.reset();
    m_tree.setRootless(true);

    // dropping on the panel detaches back to the root
    if (ImGui::BeginDragDropTarget())
    {
        if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload(ENTITY_DRAG_TYPE))
        {
            ecs::systems::HierarchySystem::attach(m_world, *static_cast<const ecs::Entity*>(payload->Data), ecs::INVALID_ENTITY);
        }

        ImGui::EndDragDropTarget();
    }

    std::vector<ecs::Entity> roots;

    for (ecs::Entity entity : m_world.getEntities())
    {
        const auto* transform = m_world.get<ecs::TransformComponent>(entity);

        if (!transform || transform->parent == ecs::INVALID_ENTITY)
        {
            roots.push_back(entity);
        }
    }

    for (size_t i = 0; i < roots.size(); i++)
    {
        drawEntityNode(roots[i], i + 1 == roots.size(), 0);
    }

    ImGui::End();
}

void Editor::drawEntityNode(ecs::Entity entity, bool last, int depth)
{
    // a hand edited file may loop parents back on themselves
    if (depth > MAX_TREE_DEPTH)
    {
        return;
    }

    const std::vector<ecs::Entity> children = getChildren(entity);

    const auto* named = m_world.get<ecs::NameComponent>(entity);
    const char* label = named ? named->name.c_str() : "Entity";

    const void* id = reinterpret_cast<const void*>(static_cast<uintptr_t>(entity));

    if (m_tree.row(id, label, last, m_selection == entity))
    {
        m_selection = entity;
    }

    // menus and payloads below share the row, id keeps them apart per entity
    ImGui::PushID(id);

    BulletRender::interface::contextMenu("entity", [&]() {
        if (ImGui::MenuItem("Delete"))
        {
            destroyEntity(entity);
        }
    });

    // drag entity onto another to reparent
    if (ImGui::BeginDragDropSource())
    {
        ImGui::SetDragDropPayload(ENTITY_DRAG_TYPE, &entity, sizeof(entity));
        ImGui::TextUnformatted(label);
        ImGui::EndDragDropSource();
    }

    if (ImGui::BeginDragDropTarget())
    {
        if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload(ENTITY_DRAG_TYPE))
        {
            // attach refuses cycles
            ecs::systems::HierarchySystem::attach(m_world, *static_cast<const ecs::Entity*>(payload->Data), entity);
        }

        ImGui::EndDragDropTarget();
    }

    ImGui::PopID();

    if (children.empty())
    {
        return;
    }

    m_tree.push(last);

    for (size_t i = 0; i < children.size(); i++)
    {
        drawEntityNode(children[i], i + 1 == children.size(), depth + 1);
    }

    m_tree.pop();
}

std::vector<ecs::Entity> Editor::getChildren(ecs::Entity entity) const
{
    std::vector<ecs::Entity> children;

    for (ecs::Entity other : m_world.getEntities())
    {
        const auto* transform = m_world.get<ecs::TransformComponent>(other);

        if (transform && transform->parent == entity)
        {
            children.push_back(other);
        }
    }

    return children;
}

void Editor::createEntity(Preset preset)
{
    m_pendingCreate.push_back(preset);
}

// builds the entity a preset describes, shape and physics included
ecs::Entity Editor::spawnEntity(Preset preset)
{
    const ecs::Entity entity = m_world.create();

    m_world.add<ecs::NameComponent>(entity).name = toString(preset) + std::string(" ") + std::to_string(entity);
    m_world.add<ecs::TransformComponent>(entity);

    if (preset == Preset::Empty)
    {
        return entity;
    }

    const bool box = preset == Preset::Box;

    auto& renderable = m_world.add<ecs::RenderableComponent>(entity);
    renderable.model = assets::Registry::instance().load<BulletRender::scene::Model>(box ? "box:1,1,1" : "sphere:0.5,32,16");
    renderable.material.setShader(m_shader);

    auto& rigidBody = m_world.add<ecs::RigidBodyComponent>(entity);
    rigidBody.body.setMass(1.0);

    auto& collider = m_world.add<ecs::ColliderComponent>(entity);

    if (box)
    {
        const BulletPhysics::math::Vec3 size{1.0, 1.0, 1.0};

        rigidBody.body.setInverseInertiaLocal(BulletPhysics::dynamics::inertia::box(1.0, size));
        collider.collider = std::make_unique<BulletPhysics::collision::collider::BoxCollider>(size);
    }
    else
    {
        rigidBody.body.setInverseInertiaLocal(BulletPhysics::dynamics::inertia::sphere(1.0, SPHERE_RADIUS));
        collider.collider = std::make_unique<BulletPhysics::collision::collider::SphereCollider>(SPHERE_RADIUS);
    }

    return entity;
}

void Editor::destroyEntity(ecs::Entity entity)
{
    m_pendingDestroy.push_back(entity);
}

// children of a dead parent go with it
void Editor::collectSubtree(ecs::Entity entity, std::vector<ecs::Entity>& out) const
{
    // a hand edited file may loop parents back on themselves
    if (std::find(out.begin(), out.end(), entity) != out.end())
    {
        return;
    }

    out.push_back(entity);

    for (ecs::Entity child : getChildren(entity))
    {
        collectSubtree(child, out);
    }
}

void Editor::destroySubtree(ecs::Entity entity)
{
    std::vector<ecs::Entity> subtree;
    collectSubtree(entity, subtree);

    for (ecs::Entity dead : subtree)
    {
        m_world.destroy(dead);

        if (m_selection == dead)
        {
            m_selection = ecs::INVALID_ENTITY;
        }
    }
}

void Editor::applyCommands()
{
    // loading replaces the world, so it clears first
    if (m_pendingClear || m_pendingLoad)
    {
        m_selection = ecs::INVALID_ENTITY;

        for (ecs::Entity entity : m_world.getEntities())
        {
            m_world.destroy(entity);
        }

        m_world.flush();
    }

    if (m_pendingLoad)
    {
        scene::load(m_world, m_scenePath);
    }

    m_pendingClear = false;
    m_pendingLoad = false;

    for (Preset preset : m_pendingCreate)
    {
        m_selection = spawnEntity(preset);
    }

    for (ecs::Entity entity : m_pendingDestroy)
    {
        destroySubtree(entity);
    }

    for (const auto& [entity, type] : m_pendingAdd)
    {
        if (auto* component = static_cast<ecs::Component*>(type->create()))
        {
            m_world.attach(entity, std::unique_ptr<ecs::Component>(component));
        }
    }

    for (const auto& [entity, type] : m_pendingRemove)
    {
        // physics holds raw pointers into the components it was given
        m_physics.detach(m_world, entity);
        m_world.detach(entity, type);
    }

    m_pendingCreate.clear();
    m_pendingDestroy.clear();
    m_pendingAdd.clear();
    m_pendingRemove.clear();
}

} // namespace interface
} // namespace BulletEngine
