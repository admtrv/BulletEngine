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
#include "collision/collider/CylinderCollider.h"
#include "collision/collider/GroundCollider.h"
#include "collision/collider/SphereCollider.h"

#include "imgui.h"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace BulletEngine {
namespace interface {

constexpr const char* ENTITY_DRAG_TYPE = "BE_ENTITY";
constexpr int MAX_TREE_DEPTH = 64;
constexpr float AMBIENT_INTENSITY = 0.3f;   // fill light, directional one does shaping
constexpr float LIGHT_HEIGHT = 5.0f;        // sun stands off scene, gizmo has to clear it
constexpr float FLAT_INTENSITY = 1.0f;      // flat world has no sun, ambient carries it alone

constexpr float BODY_HEIGHT = 0.5f;         // half its size, so it rests on ground

// where fresh scene puts its camera, back enough to hold what fills it
constexpr glm::vec3 CAMERA_SOLID{0.0f, 2.0f, 6.0f};
constexpr glm::vec3 CAMERA_FLAT{0.0f, 0.0f, 10.0f};

// flat world keeps depth for ordering alone, so nothing drifts along it
constexpr BulletPhysics::dynamics::Constraints FLAT_CONSTRAINTS =
    BulletPhysics::dynamics::FREEZE_POSITION_Z | BulletPhysics::dynamics::FREEZE_ROTATION_X | BulletPhysics::dynamics::FREEZE_ROTATION_Y;

// shape matching what the entity shows, a flat one is a solid squashed along depth
static std::unique_ptr<BulletPhysics::collision::collider::Collider> makeCollider(Preset preset)
{
    using namespace BulletPhysics::collision::collider;
    using BulletPhysics::math::Quat;
    using BulletPhysics::math::Vec3;

    switch (preset)
    {
        case Preset::Sphere:
            return std::make_unique<SphereCollider>(0.5);

        case Preset::Circle:
        {
            auto cylinder = std::make_unique<CylinderCollider>(0.5, THICKNESS_2D);

            // laid on its side, so its caps face the flat view
            cylinder->setLocalRotation(Quat::fromAxisAngle({1.0, 0.0, 0.0}, M_PI * 0.5));
            return cylinder;
        }

        case Preset::Square:
        case Preset::Sprite:
            return std::make_unique<BoxCollider>(Vec3{1.0, 1.0, THICKNESS_2D});

        default:
            return std::make_unique<BoxCollider>(Vec3{1.0, 1.0, 1.0});
    }
}

// what a preset lies in, flat ones keep depth for ordering alone
static bool isFlat(Preset preset)
{
    return preset == Preset::Square || preset == Preset::Circle || preset == Preset::Sprite;
}

// shapes world of one kind is built from
void Editor::drawPresetMenu(const char* label, std::initializer_list<Preset> presets)
{
    if (!ImGui::BeginMenu(label))
    {
        return;
    }

    for (const Preset* preset = presets.begin(); preset != presets.end(); preset++)
    {
        // last one waits for asset, ready shapes stand apart from it
        if (preset == presets.end() - 1)
        {
            ImGui::Separator();
        }

        if (ImGui::MenuItem(toString(*preset).c_str()))
        {
            createEntity(*preset);
        }
    }

    ImGui::EndMenu();
}

void Editor::drawHierarchy()
{
    if (!m_showHierarchy)
    {
        return;
    }

    ImGui::Begin(HIERARCHY_PANEL, &m_showHierarchy);

    // dropping on panel detaches back to root, prefab lands there too
    if (ImGui::BeginDragDropTarget())
    {
        if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload(ENTITY_DRAG_TYPE))
        {
            ecs::systems::HierarchySystem::attach(m_world, *static_cast<const ecs::Entity*>(payload->Data), ecs::INVALID_ENTITY);
        }

        acceptPrefabDrop(ecs::INVALID_ENTITY);

        ImGui::EndDragDropTarget();
    }

    if (ImGui::Button("Add Entity", {ImGui::GetContentRegionAvail().x, 0.0f}))
    {
        ImGui::OpenPopup("entities");
    }

    if (ImGui::BeginPopup("entities"))
    {
        // solid and flat live apart, bare entity belongs to neither
        drawPresetMenu("3D", {Preset::Cube, Preset::Sphere, Preset::Model});
        drawPresetMenu("2D", {Preset::Square, Preset::Circle, Preset::Sprite});

        ImGui::Separator();

        if (ImGui::MenuItem(toString(Preset::Empty).c_str()))
        {
            createEntity(Preset::Empty);
        }

        ImGui::EndPopup();
    }

    // own tree, guides start fresh
    m_tree.reset();
    m_tree.setRootless(true);

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
    // hand edited file may loop parents back on themselves
    if (depth > MAX_TREE_DEPTH)
    {
        return;
    }

    const std::vector<ecs::Entity> children = getChildren(entity);

    const auto* identity = m_world.get<ecs::IdentityComponent>(entity);
    const char* label = identity ? identity->name.c_str() : "Entity";

    const void* id = reinterpret_cast<const void*>(static_cast<uintptr_t>(entity));

    if (m_tree.row(id, label, last, m_selection == entity))
    {
        m_selection = entity;
    }

    // menus and payloads below share row, id keeps them apart per entity
    ImGui::PushID(id);

    BulletRender::interface::contextMenu("entity", [&]() {
        if (ImGui::MenuItem("Save As Prefab"))
        {
            m_pendingPrefab = entity;
        }

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

        // prefab dropped on row becomes its child
        acceptPrefabDrop(entity);

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

// prefab dragged out of explorer joins scene, invalid parent leaves it at root
void Editor::acceptPrefabDrop(ecs::Entity parent)
{
    const ImGuiPayload* payload = ImGui::AcceptDragDropPayload(ASSET_DRAG_TYPE);

    if (!payload)
    {
        return;
    }

    std::string key(static_cast<const char*>(payload->Data));

    // explorer drags every file, only prefabs mean anything here
    if (!key.ends_with(PREFAB_EXTENSION))
    {
        return;
    }

    m_pendingInstance.emplace_back(std::move(key), parent);
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

// builds what a preset describes, shape and body matching what it shows
ecs::Entity Editor::spawnEntity(Preset preset)
{
    const ecs::Entity entity = m_world.create();

    m_world.add<ecs::IdentityComponent>(entity).name = toString(preset);
    m_world.add<ecs::TransformComponent>(entity);

    if (preset == Preset::Empty)
    {
        return entity;
    }

    const bool flat = isFlat(preset);

    // flat ones are sprites, whether picture fills them or only colour does
    std::unique_ptr<ecs::Renderable> renderable;

    if (flat)
    {
        auto sprite = std::make_unique<ecs::Sprite>();
        sprite->setShape(int(preset == Preset::Circle ? ecs::Sprite::Shape::Circle : ecs::Sprite::Shape::Quad));

        renderable = std::move(sprite);
    }
    else
    {
        auto mesh = std::make_unique<ecs::Mesh>();

        // one waiting for an asset stays empty until the inspector fills it
        if (preset != Preset::Model)
        {
            mesh->setModelKey(preset == Preset::Cube ? assets::BOX_KEY : assets::SPHERE_KEY);
        }

        renderable = std::move(mesh);
    }

    m_world.add<ecs::RenderableComponent>(entity).renderable = std::move(renderable);
    m_world.add<ecs::ColliderComponent>(entity).collider = makeCollider(preset);

    auto& rigidBody = m_world.add<ecs::RigidBodyComponent>(entity);

    if (flat)
    {
        rigidBody.body.setConstraints(FLAT_CONSTRAINTS);
    }

    return entity;
}

// prefab file read into world, attached where it was dropped
ecs::Entity Editor::spawnPrefab(const std::string& key, ecs::Entity parent)
{
    const ecs::Entity entity = scene::loadPrefab(m_world, project::Project::instance().getPath(key));

    if (entity == ecs::INVALID_ENTITY)
    {
        std::cerr << "prefab load failed: " << key << '\n';
        return ecs::INVALID_ENTITY;
    }

    if (parent != ecs::INVALID_ENTITY)
    {
        ecs::systems::HierarchySystem::attach(m_world, entity, parent);
    }

    return entity;
}

// what empty project starts from, floor, something on it and light
void Editor::fillNewScene()
{
    const bool flat = project::Project::instance().getSettings().mode == project::Mode::Mode2D;

    {
        const ecs::Entity entity = m_world.create();

        m_world.add<ecs::IdentityComponent>(entity).name = "Camera";
        m_world.add<ecs::TransformComponent>(entity).transform.setPosition(flat ? CAMERA_FLAT : CAMERA_SOLID);

        auto& camera = m_world.add<ecs::CameraComponent>(entity);
        camera.projection = flat ? BulletRender::scene::Projection::Orthographic : BulletRender::scene::Projection::Perspective;
        camera.main = true;
    }

    {
        const ecs::Entity entity = m_world.create();

        m_world.add<ecs::IdentityComponent>(entity).name = "Ambient Light";
        m_world.add<ecs::TransformComponent>(entity);

        auto light = std::make_shared<BulletRender::scene::AmbientLight>();
        light->setIntensity(flat ? FLAT_INTENSITY : AMBIENT_INTENSITY);

        m_world.add<ecs::LightComponent>(entity).light = std::move(light);
    }

    // flat world is lit evenly, shaping it would only fight art
    if (!flat)
    {
        const ecs::Entity entity = m_world.create();

        m_world.add<ecs::IdentityComponent>(entity).name = "Directional Light";

        // light points way entity faces, default direction sets that pose
        auto& transform = m_world.add<ecs::TransformComponent>(entity);

        auto light = std::make_shared<BulletRender::scene::DirectionalLight>();
        transform.transform.setRotation(light->getTransform().getRotation());
        transform.transform.setPosition(LIGHT_HEIGHT * light->getDirection());

        m_world.add<ecs::LightComponent>(entity).light = std::move(light);
    }

    {
        const ecs::Entity entity = m_world.create();

        m_world.add<ecs::IdentityComponent>(entity).name = "Ground";
        m_world.add<ecs::TransformComponent>(entity);

        m_world.add<ecs::RigidBodyComponent>(entity).body.setMotionType(BulletPhysics::dynamics::MotionType::Static);
        m_world.add<ecs::ColliderComponent>(entity).collider =
            std::make_unique<BulletPhysics::collision::collider::GroundCollider>(0.0);
    }

    // same thing the menu spawns, only lifted off the ground it rests on
    {
        const ecs::Entity entity = spawnEntity(flat ? Preset::Square : Preset::Cube);
        const glm::vec3 position{0.0f, BODY_HEIGHT, 0.0f};

        m_world.get<ecs::IdentityComponent>(entity)->name = flat ? "Square" : "Cube";
        m_world.get<ecs::TransformComponent>(entity)->transform.setPosition(position);
        m_world.get<ecs::RigidBodyComponent>(entity)->body.setPosition({position.x, position.y, position.z});
    }
}

void Editor::openStartScene()
{
    const project::Project& project = project::Project::instance();
    const std::string& key = project.getSettings().startScene;

    // game names what it starts with, project without it opens empty
    if (key.empty() || !scene::load(m_world, project.getPath(key)))
    {
        fillNewScene();
        return;
    }

    m_sceneKey = key;
}

void Editor::destroyEntity(ecs::Entity entity)
{
    m_pendingDestroy.push_back(entity);
}

// children of dead parent go with it
void Editor::collectSubtree(ecs::Entity entity, std::vector<ecs::Entity>& out) const
{
    // hand edited file may loop parents back on themselves
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

    if (m_pendingPrefab != ecs::INVALID_ENTITY)
    {
        savePrefab(m_pendingPrefab);
        m_pendingPrefab = ecs::INVALID_ENTITY;
    }

    for (Preset preset : m_pendingCreate)
    {
        m_selection = spawnEntity(preset);
    }

    for (const auto& [key, parent] : m_pendingInstance)
    {
        if (const ecs::Entity entity = spawnPrefab(key, parent); entity != ecs::INVALID_ENTITY)
        {
            m_selection = entity;
        }
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
        // physics holds raw pointers into components it was given
        m_physics.detach(m_world, entity);
        m_world.detach(entity, type);
    }

    m_pendingCreate.clear();
    m_pendingInstance.clear();
    m_pendingDestroy.clear();
    m_pendingAdd.clear();
    m_pendingRemove.clear();
}

} // namespace interface
} // namespace BulletEngine
