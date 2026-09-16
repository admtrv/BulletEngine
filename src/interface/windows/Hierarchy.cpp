/*
 * Hierarchy.cpp
 */

#include "interface/Editor.h"

#include "assets/Loaders.h"
#include "assets/Registry.h"
#include "ecs/Components.h"
#include "ecs/systems/HierarchySystem.h"
#include "project/Project.h"
#include "reflect/Type.h"
#include "scene/Serializer.h"

#include "interface/elements/Widgets.h"
#include "scene/Light.h"
#include "scene/models/Model.h"

#include "collision/collider/BoxCollider.h"
#include "collision/collider/GroundCollider.h"

#include "imgui.h"

#include <algorithm>

namespace BulletEngine {
namespace interface {

constexpr const char* ENTITY_DRAG_TYPE = "BE_ENTITY";
constexpr int MAX_TREE_DEPTH = 64;
constexpr float AMBIENT_INTENSITY = 0.3f;   // fill light, directional one does the shaping
constexpr float LIGHT_HEIGHT = 5.0f;        // sun stands off the scene, gizmo has to clear it

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
        // shapes first, bare entity closes the list
        for (const Preset preset : {Preset::Box, Preset::Sphere, Preset::Empty})
        {
            if (preset == Preset::Empty)
            {
                ImGui::Separator();
            }

            if (ImGui::MenuItem(toString(preset).c_str()))
            {
                createEntity(preset);
            }
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

// builds what a preset describes, physics is left to inspector
ecs::Entity Editor::spawnEntity(Preset preset)
{
    const ecs::Entity entity = m_world.create();

    m_world.add<ecs::NameComponent>(entity).name = toString(preset) + std::string(" ") + std::to_string(entity);
    m_world.add<ecs::TransformComponent>(entity);

    if (preset == Preset::Empty)
    {
        return entity;
    }

    auto& renderable = m_world.add<ecs::RenderableComponent>(entity);
    renderable.model = assets::Registry::instance().load<BulletRender::scene::Model>(
        preset == Preset::Box ? assets::BOX_KEY : assets::SPHERE_KEY);

    return entity;
}

// what empty project starts from, floor, something on it and light
void Editor::fillNewScene()
{
    {
        const ecs::Entity entity = m_world.create();

        m_world.add<ecs::NameComponent>(entity).name = "Ambient Light";
        m_world.add<ecs::TransformComponent>(entity);

        auto light = std::make_shared<BulletRender::scene::AmbientLight>();
        light->setIntensity(AMBIENT_INTENSITY);

        m_world.add<ecs::LightComponent>(entity).light = std::move(light);
    }

    {
        const ecs::Entity entity = m_world.create();

        m_world.add<ecs::NameComponent>(entity).name = "Directional Light";

        // light points the way entity faces, default direction sets that pose
        auto& transform = m_world.add<ecs::TransformComponent>(entity);

        auto light = std::make_shared<BulletRender::scene::DirectionalLight>();
        transform.transform.setRotation(light->getTransform().getRotation());
        transform.transform.setPosition(LIGHT_HEIGHT * light->getDirection());

        m_world.add<ecs::LightComponent>(entity).light = std::move(light);
    }

    {
        const ecs::Entity entity = m_world.create();

        m_world.add<ecs::NameComponent>(entity).name = "Ground";
        m_world.add<ecs::TransformComponent>(entity);

        m_world.add<ecs::RigidBodyComponent>(entity).body.setMotionType(BulletPhysics::dynamics::MotionType::Static);
        m_world.add<ecs::ColliderComponent>(entity).collider =
            std::make_unique<BulletPhysics::collision::collider::GroundCollider>(0.0);
    }

    {
        const BulletPhysics::math::Vec3 size{1.0, 1.0, 1.0};
        const BulletPhysics::math::Vec3 position{0.0, 0.5, 0.0};

        const ecs::Entity entity = m_world.create();

        m_world.add<ecs::NameComponent>(entity).name = "Cube";
        m_world.add<ecs::TransformComponent>(entity).transform.setPosition({0.0f, 0.5f, 0.0f});

        m_world.add<ecs::RenderableComponent>(entity).model =
            assets::Registry::instance().load<BulletRender::scene::Model>(assets::BOX_KEY);

        auto& rigidBody = m_world.add<ecs::RigidBodyComponent>(entity);
        rigidBody.body.setMass(1.0);
        rigidBody.body.setPosition(position);

        m_world.add<ecs::ColliderComponent>(entity).collider =
            std::make_unique<BulletPhysics::collision::collider::BoxCollider>(size);
    }
}

void Editor::openFirstScene()
{
    const std::vector<std::string> scenes = project::Project::instance().getKeys(SCENE_EXTENSION);

    if (scenes.empty())
    {
        fillNewScene();
        return;
    }

    m_sceneKey = scenes.front();
    scene::load(m_world, project::Project::instance().getPath(m_sceneKey));
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
    // opening replaces world, so it clears first
    if (m_pendingClear || !m_pendingOpen.empty())
    {
        m_selection = ecs::INVALID_ENTITY;

        m_world.clear();
        m_world.flush();
    }

    if (!m_pendingOpen.empty())
    {
        m_sceneKey = std::move(m_pendingOpen);
        scene::load(m_world, project::Project::instance().getPath(m_sceneKey));
    }
    else if (m_pendingClear)
    {
        m_sceneKey.clear();
        fillNewScene();
    }

    m_pendingOpen.clear();
    m_pendingClear = false;

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
