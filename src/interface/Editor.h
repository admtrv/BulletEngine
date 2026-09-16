/*
 * Editor.h
 */

#pragma once

#include "ecs/Ecs.h"
#include "ecs/systems/DebugDrawSystem.h"
#include "ecs/systems/PhysicsSystem.h"

#include "interface/elements/TreeView.h"
#include "interface/elements/Widgets.h"
#include "reflect/Value.h"
#include "scene/Archive.h"
#include "render/buffers/FrameBuffer.h"
#include "render/passes/RenderPass.h"
#include "render/Shader.h"
#include "scene/Camera.h"
#include "scene/Scene.h"

#include "imgui.h"

#include <glm/glm.hpp>

#include <cstdint>
#include <functional>
#include <string>
#include <typeindex>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace BulletEngine {

// fwd
namespace reflect { class Type; class Field; }
namespace project { struct Entry; }

namespace interface {

// panel titles, also the keys the dock layout is built with
constexpr const char* HIERARCHY_PANEL = "Hierarchy";
constexpr const char* INSPECTOR_PANEL = "Inspector";
constexpr const char* SCENE_PANEL = "Scene";
constexpr const char* GAME_PANEL = "Game";
constexpr const char* CONSOLE_PANEL = "Console";
constexpr const char* EXPLORER_PANEL = "Explorer";

// keys leaving the explorer, file to asset field or either kind to folder
constexpr const char* ASSET_DRAG_TYPE = "BE_ASSET";
constexpr const char* FOLDER_DRAG_TYPE = "BE_FOLDER";

constexpr const char* SCENE_EXTENSION = ".scene";
constexpr const char* SCENE_DEFAULT_NAME = "NewScene";

// what editor is doing with the world
enum class Mode {
    Edit,       // scene stands still, ready to arrange
    Play        // physics and scripts run, changes dropped on stop
};

// told after mode changed, world already restored on stop
using ModeListener = std::function<void(Mode)>;

// what a new entity comes with
enum class Preset {
    Empty,
    Box,
    Sphere
};

// menu label, also name of spawned entity
inline std::string toString(Preset preset)
{
    switch (preset)
    {
        case Preset::Box:    return "Box";
        case Preset::Sphere: return "Sphere";
        default:             return "Entity";
    }
}

// what an asset field types into, with last failure to report
struct AssetPath {
    char text[256] = "";
    std::string error;
};

// docked panels driving the world
class Editor {
public:
    Editor(ecs::World& world, ecs::systems::PhysicsSystem& physics, ecs::systems::DebugDrawSystem& debugDraw);

    void draw();

    // runs between frames, once imgui context exists
    void beforeFrame();

    // draws open panels, each into its own texture
    void renderViews(BulletRender::scene::Scene& scene);

    // what only scene panel shows, grid, axis and gizmos
    void addEditorPass(std::shared_ptr<BulletRender::render::IRenderPass> pass) { m_editorPasses.push_back(std::move(pass)); }

    // scene project opens with, first one it holds or a new one
    void openFirstScene();

    // world only advances while playing
    bool isPlaying() const { return m_mode == Mode::Play; }

    // called on every play and stop, systems hook own state to it
    void addModeListener(ModeListener listener) { m_modeListeners.push_back(std::move(listener)); }

    // what scene panel looks through, editor owns it so scenes never carry it
    BulletRender::scene::FlyCamera& getCamera() { return *m_camera; }

    bool isSceneFocused() const { return m_sceneFocused; }

    ecs::Entity getSelection() const { return m_selection; }
    void setSelection(ecs::Entity entity) { m_selection = entity; }

    // where the scene panel was last clicked, in panel coordinates
    bool hasScenePick() const { return m_scenePick.x >= 0.0f; }
    const glm::vec2& getScenePick() const { return m_scenePick; }
    void clearScenePick() { m_scenePick = {-1.0f, -1.0f}; }

    const glm::vec2& getSceneSize() const { return m_sceneSize; }

private:
    // layout
    void drawDockSpace();
    void buildLayout(unsigned dockId);

    // panels
    void openPanel(bool& shown, const char* name);
    std::string sceneName() const;
    void saveScene(const std::string& key);
    void drawSceneMenu();
    void drawDebugMenu();
    void setMode(Mode mode);
    void drawPlayBar();
    void drawMenuBar();
    void drawScene();
    void drawGame();
    void drawHierarchy();
    void drawEntityNode(ecs::Entity entity, bool last, int depth);
    void drawInspector();
    void drawAddMenu();
    void drawConsole();
    void drawExplorer();
    void applyEntryCommands();
    void acceptEntryDrop(const std::string& folder);
    void drawEntry(const project::Entry& entry, bool last);
    bool drawField(const reflect::Field& field, void* instance);
    bool drawAxes(const reflect::Field* const axes[3], void* instance);

    // walks fields given as values or as pointers, defined where it is used
    template<class F>
    bool drawRange(F fields, size_t count, void* instance);

    bool drawFields(const reflect::Type& type, void* instance, bool splitOwn = false);
    bool drawValue(const reflect::Field& field, void* instance);
    bool drawObjectType(const reflect::Field& field, void* instance, const reflect::Type& current);

    using ValueMap = std::unordered_map<std::string, reflect::Value>;
    void collectValues(const reflect::Type& type, const void* instance, ValueMap& out) const;
    void applyValues(const reflect::Type& type, void* instance, const ValueMap& values) const;

    // entities
    std::vector<ecs::Entity> getChildren(ecs::Entity entity) const;
    ecs::Entity spawnEntity(Preset preset);
    void createEntity(Preset preset);
    void fillNewScene();
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
    ecs::systems::DebugDrawSystem& m_debugDraw;
    ecs::Entity m_selection = ecs::INVALID_ENTITY;

    BulletRender::interface::TreeView m_tree;
    BulletRender::interface::TreeView m_explorerTree;

    Mode m_mode = Mode::Edit;
    std::vector<ModeListener> m_modeListeners;

    // world before play began, what stop restores
    scene::Node m_snapshot;

    // scene being edited, empty until saved somewhere
    std::string m_sceneKey;
    char m_sceneName[128] = "";

    // what is typed into an asset field before Load is pressed
    std::unordered_map<std::string, AssetPath> m_assetPaths;

    // queued while the world is being walked
    std::vector<Preset> m_pendingCreate;
    std::vector<ecs::Entity> m_pendingDestroy;

    std::vector<std::pair<ecs::Entity, const reflect::Type*>> m_pendingAdd;
    std::vector<std::pair<ecs::Entity, std::type_index>> m_pendingRemove;

    std::string m_pendingOpen;
    bool m_pendingClear = false;

    // panel visibility, driven by the windows menu
    bool m_showScene = true;
    bool m_showGame = true;
    bool m_showHierarchy = true;
    bool m_showInspector = true;
    bool m_showConsole = true;
    bool m_showExplorer = true;

    // folders user closed, everything else is open
    std::unordered_set<std::string> m_folded;
    std::string m_explorerSelection;

    // queued while tree is walked
    std::pair<std::string, std::string> m_pendingMove;
    std::string m_pendingDelete;

    // the journal as the field sees it, copied when it changes
    std::string m_consoleText;
    float m_consoleLines = 1.0f;
    uint32_t m_consoleRevision = 0;
    bool m_consoleTail = true;      // scrolls down once after new lines arrive

    const char* m_focusPanel = nullptr;
    const reflect::Type* m_focusComponent = nullptr;

    bool m_themeApplied = false;
    bool m_layoutBuilt = false;

    // passes the game view does without
    std::vector<std::shared_ptr<BulletRender::render::IRenderPass>> m_editorPasses;

    // what each panel draws into, sized to fill it
    std::unique_ptr<BulletRender::render::FrameBuffer> m_sceneView;
    std::unique_ptr<BulletRender::render::FrameBuffer> m_gameView;

    glm::vec2 m_sceneSize{0.0f, 0.0f};
    glm::vec2 m_gameSize{0.0f, 0.0f};

    bool m_sceneFocused = false;
    glm::vec2 m_scenePick{-1.0f, -1.0f};

    // what scene panel looks through, editor owns it so scenes never carry it
    std::unique_ptr<BulletRender::scene::FlyCamera> m_camera;

    // what game panel looks through, driven by camera entity
    std::shared_ptr<BulletRender::scene::StaticCamera> m_gameCamera;
};

} // namespace interface
} // namespace BulletEngine
