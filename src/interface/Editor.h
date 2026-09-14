/*
 * Editor.h
 */

#pragma once

#include "ecs/Ecs.h"
#include "ecs/systems/PhysicsSystem.h"

#include "interface/elements/TreeView.h"
#include "interface/elements/Widgets.h"
#include "reflect/Value.h"
#include "render/Shader.h"

#include "imgui.h"

#include <glm/glm.hpp>

#include <string>
#include <typeindex>
#include <unordered_map>
#include <utility>
#include <vector>

namespace BulletEngine {

// fwd
namespace reflect { class Type; class Field; }

namespace interface {

// panel titles, also the keys the dock layout is built with
constexpr const char* HIERARCHY_PANEL = "Hierarchy";
constexpr const char* INSPECTOR_PANEL = "Inspector";
constexpr const char* SCENE_PANEL = "Scene";

// what a new entity comes with
enum class Preset {
    Empty,
    Box,
    Sphere
};

inline std::string toString(Preset preset)
{
    switch (preset)
    {
        case Preset::Box:    return "Box";
        case Preset::Sphere: return "Sphere";
        default:             return "Entity";
    }
}

// what an asset field types into, with the last failure to report
struct AssetPath {
    char text[256] = "";
    std::string error;
};

// docked panels driving the world
class Editor {
public:
    Editor(ecs::World& world, ecs::systems::PhysicsSystem& physics);

    void draw();

    // runs between frames, once imgui context exists
    void beforeFrame();

    // size the scene is rendered at, known one frame ahead
    void applySceneSize();

    void setScenePath(std::string path) { m_scenePath = std::move(path); }

    bool isSceneFocused() const { return m_sceneFocused; }

    ecs::Entity getSelection() const { return m_selection; }
    void setSelection(ecs::Entity entity) { m_selection = entity; }

    // where the scene panel was last clicked, in panel coordinates
    bool hasScenePick() const { return m_scenePick.x >= 0.0f; }
    const glm::vec2& getScenePick() const { return m_scenePick; }
    void clearScenePick() { m_scenePick = {-1.0f, -1.0f}; }

    const glm::vec2& getSceneSize() const { return m_sceneSize; }

    // what a spawned entity is drawn with
    void setDefaultShader(std::shared_ptr<BulletRender::render::GraphicsShader> shader) { m_shader = std::move(shader); }

private:
    // layout
    void drawDockSpace();
    void buildLayout(unsigned dockId);

    // panels
    void openPanel(bool& shown, const char* name);
    void drawMenuBar();
    void drawScene();
    void drawHierarchy();
    void drawEntityNode(ecs::Entity entity, bool last, int depth);
    void drawInspector();
    void drawAddMenu();
    bool drawFields(const reflect::Type& type, void* instance);
    bool drawValue(const reflect::Field& field, void* instance);
    bool drawObjectType(const reflect::Field& field, void* instance, const reflect::Type& current);

    using ValueMap = std::unordered_map<std::string, reflect::Value>;
    void collectValues(const reflect::Type& type, const void* instance, ValueMap& out) const;
    void applyValues(const reflect::Type& type, void* instance, const ValueMap& values) const;

    // entities
    std::vector<ecs::Entity> getChildren(ecs::Entity entity) const;
    ecs::Entity spawnEntity(Preset preset);
    void createEntity(Preset preset);
    void destroyEntity(ecs::Entity entity);
    void collectSubtree(ecs::Entity entity, std::vector<ecs::Entity>& out) const;
    void destroySubtree(ecs::Entity entity);
    void addComponent(ecs::Entity entity, const reflect::Type& type);
    void removeComponent(ecs::Entity entity, std::type_index type);
    void applyCommands();
    void syncBody(ecs::Entity entity);
    void syncCollider(ecs::Entity entity);

    ecs::World& m_world;
    ecs::systems::PhysicsSystem& m_physics;
    ecs::Entity m_selection = ecs::INVALID_ENTITY;

    BulletRender::interface::TreeView m_tree;

    std::string m_scenePath = "scene.txt";
    std::shared_ptr<BulletRender::render::GraphicsShader> m_shader;

    // what is typed into an asset field before Load is pressed
    std::unordered_map<std::string, AssetPath> m_assetPaths;

    // queued while the world is being walked
    std::vector<Preset> m_pendingCreate;
    std::vector<ecs::Entity> m_pendingDestroy;

    std::vector<std::pair<ecs::Entity, const reflect::Type*>> m_pendingAdd;
    std::vector<std::pair<ecs::Entity, std::type_index>> m_pendingRemove;

    bool m_pendingLoad = false;
    bool m_pendingClear = false;

    // panel visibility, driven by the windows menu
    bool m_showScene = true;
    bool m_showHierarchy = true;
    bool m_showInspector = true;

    const char* m_focusPanel = nullptr;
    const reflect::Type* m_focusComponent = nullptr;

    bool m_themeApplied = false;
    bool m_layoutBuilt = false;

    glm::vec2 m_sceneSize{0.0f, 0.0f};
    bool m_sceneFocused = false;
    glm::vec2 m_scenePick{-1.0f, -1.0f};
};

} // namespace interface
} // namespace BulletEngine
