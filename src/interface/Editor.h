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
#include <initializer_list>
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

// panel titles, also keys dock layout is built with
constexpr const char* HIERARCHY_PANEL = "Hierarchy";
constexpr const char* INSPECTOR_PANEL = "Inspector";
constexpr const char* SCENE_PANEL = "Scene";
constexpr const char* GAME_PANEL = "Game";
constexpr const char* CONSOLE_PANEL = "Console";
constexpr const char* EXPLORER_PANEL = "Explorer";

// keys leaving explorer, file to asset field or either kind to folder
constexpr const char* ASSET_DRAG_TYPE = "BE_ASSET";
constexpr const char* FOLDER_DRAG_TYPE = "BE_FOLDER";

constexpr const char* SCENE_EXTENSION = ".scene";
constexpr const char* PREFAB_EXTENSION = ".prefab";
constexpr const char* SCENE_DEFAULT_NAME = "NewScene";

enum class Mode {
    Edit,       // scene stands still, ready to arrange
    Play        // physics and scripts run, changes dropped on stop
};

using ModeListener = std::function<void(Mode)>;     // told after mode changed, world already restored on stop

// what new entity comes with, flat ones lie in xy with depth left for ordering
enum class Preset {
    Cube,
    Sphere,
    Model,      // waits for asset, inspector picks file

    Square,
    Circle,
    Sprite,     // same, waiting for picture

    Empty
};

// menu label, also name of spawned entity
inline std::string toString(Preset preset)
{
    switch (preset)
    {
        case Preset::Cube:   return "Cube";
        case Preset::Sphere: return "Sphere";
        case Preset::Model:  return "Model";
        case Preset::Square: return "Square";
        case Preset::Circle: return "Circle";
        case Preset::Sprite: return "Sprite";
        default:             return "Empty";
    }
}

// editor camera
inline constexpr float EDITOR_CAMERA_FAR = 40.0f;

// ground overlays, shares of camera far plane so grid never leaves before objects on it
inline constexpr float GROUND_FADE_START = 0.7f;
inline constexpr float GROUND_FADE_END = 1.0f;

// docked panels driving the world
class Editor {
public:
    Editor(ecs::World& world, ecs::systems::PhysicsSystem& physics, ecs::systems::DebugDrawSystem& debugDraw);

    void draw();
    void beforeFrame();                                      // runs between frames, once imgui context exists
    void renderViews(BulletRender::scene::Scene& scene);     // draws open panels, each into its own texture

    // passes one view shows and other does without
    void addEditorPass(std::shared_ptr<BulletRender::render::IRenderPass> pass, const char* name) { m_editorPasses.push_back({std::move(pass), name, true}); }
    void addGamePass(std::shared_ptr<BulletRender::render::IRenderPass> pass) { m_gamePasses.push_back(std::move(pass)); }

    // scenes
    void openStartScene();
    void requestScene(std::string key) { m_pendingOpen = std::move(key); }   // world is swapped between frames, script may ask mid update

    // play mode
    bool isPlaying() const { return m_mode == Mode::Play; }
    void addModeListener(ModeListener listener) { m_modeListeners.push_back(std::move(listener)); }

    // what scene panel looks through, editor owns it so scenes never carry one
    BulletRender::scene::Camera& getCamera();
    bool isFlatView() const { return m_flatView; }
    bool isSceneFocused() const { return m_sceneFocused; }

    ecs::Entity getSelection() const { return m_selection; }
    void setSelection(ecs::Entity entity) { m_selection = entity; }

    // where scene panel was last clicked, in panel coordinates
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
    void savePrefab(ecs::Entity entity);        // writes it into project, named after entity
    void drawProjectMenu();
    void drawSceneMenu();
    void drawDebugMenu();
    void drawEditorMenu();
    void setMode(Mode mode);
    void drawPlayBar();
    void drawMenuBar();
    void drawProjectionToggle();
    void drawScene();
    void drawGame();
    void drawHierarchy();
    void drawPresetMenu(const char* label, std::initializer_list<Preset> presets);
    void drawEntityNode(ecs::Entity entity, bool last, int depth);
    void acceptPrefabDrop(ecs::Entity parent);
    void drawInspector();
    void drawAddMenu();
    void drawConsole();
    void drawExplorer();
    void applyEntryCommands();
    void acceptEntryDrop(const std::string& folder);
    void drawEntry(const project::Entry& entry, bool last);
    bool drawField(const reflect::Field& field, void* instance);
    bool drawAxes(const reflect::Field* const axes[3], void* instance);

    template<class F>
    bool drawRange(F fields, size_t count, void* instance);     // fields given as values or as pointers, defined where it is used

    bool drawFields(const reflect::Type& type, void* instance, bool splitOwn = false);
    bool drawValue(const reflect::Field& field, void* instance);
    bool drawOptional(const reflect::Field& field, void* instance);
    bool drawObjectType(const reflect::Field& field, void* instance, const reflect::Type* current);

    using ValueMap = std::unordered_map<std::string, reflect::Value>;
    void collectValues(const reflect::Type& type, const void* instance, ValueMap& out) const;
    void applyValues(const reflect::Type& type, void* instance, const ValueMap& values) const;

    // entities
    std::vector<ecs::Entity> getChildren(ecs::Entity entity) const;
    ecs::Entity spawnEntity(Preset preset);
    ecs::Entity spawnPrefab(const std::string& key, ecs::Entity parent);
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
    scene::Node m_snapshot;                     // world before play began, what stop restores

    std::string m_sceneKey;                     // scene being edited, empty until saved somewhere
    char m_sceneName[128] = "";
    char m_projectName[128] = "";

    std::unordered_map<std::string, BulletRender::interface::AssetFieldState> m_assetPaths;     // typed into asset field before Load is pressed

    // queued while world is being walked
    std::vector<Preset> m_pendingCreate;
    std::vector<std::pair<std::string, ecs::Entity>> m_pendingInstance;      // prefabs dropped on hierarchy, with parent to attach to
    std::vector<ecs::Entity> m_pendingDestroy;
    ecs::Entity m_pendingPrefab = ecs::INVALID_ENTITY;      // entity menu asked to save
    std::vector<std::pair<ecs::Entity, const reflect::Type*>> m_pendingAdd;
    std::vector<std::pair<ecs::Entity, std::type_index>> m_pendingRemove;
    std::string m_pendingOpen;
    bool m_pendingClear = false;

    // panel visibility, driven by windows menu
    bool m_showScene = true;
    bool m_showGame = true;
    bool m_showHierarchy = true;
    bool m_showInspector = true;
    bool m_showConsole = true;
    bool m_showExplorer = true;

    // explorer
    std::unordered_set<std::string> m_opened;   // folders user opened, rest stay shut
    std::string m_explorerSelection;
    std::pair<std::string, std::string> m_pendingMove;      // queued while tree is walked
    std::string m_pendingDelete;

    // journal as field sees it, copied when it changes
    std::string m_consoleText;
    uint32_t m_consoleRevision = 0;
    bool m_consoleTail = true;      // scrolls down once after new lines arrive

    const char* m_focusPanel = nullptr;
    const reflect::Type* m_focusComponent = nullptr;

    bool m_themeApplied = false;
    bool m_layoutBuilt = false;

    // passes one view shows and other does without
    struct EditorPass {
        std::shared_ptr<BulletRender::render::IRenderPass> pass;
        const char* name = nullptr;     // named ones reach settings menu
        bool shown = true;
    };

    std::vector<EditorPass> m_editorPasses;
    std::vector<std::shared_ptr<BulletRender::render::IRenderPass>> m_gamePasses;

    // what each panel draws into, sized to fill it
    std::unique_ptr<BulletRender::render::FrameBuffer> m_sceneView;
    std::unique_ptr<BulletRender::render::FrameBuffer> m_gameView;
    glm::vec2 m_sceneSize{0.0f, 0.0f};
    glm::vec2 m_gameSize{0.0f, 0.0f};

    bool m_sceneFocused = false;
    glm::vec2 m_scenePick{-1.0f, -1.0f};

    // cameras panels look through, editor owns them so scenes never carry one
    std::unique_ptr<BulletRender::scene::FlyCamera> m_camera;
    std::unique_ptr<BulletRender::scene::PanCamera> m_flatCamera;
    bool m_flatView = false;
    std::shared_ptr<BulletRender::scene::StaticCamera> m_gameCamera;     // driven by camera entity
};

} // namespace interface
} // namespace BulletEngine
