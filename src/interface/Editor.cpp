/*
 * Editor.cpp
 */

#include "Editor.h"

#include "interface/elements/Fonts.h"
#include "interface/elements/Theme.h"
#include "ecs/Components.h"
#include "project/Project.h"
#include "scene/Serializer.h"

#include "imgui.h"
#include "imgui_internal.h"

#include <cmath>
#include <cstdio>
#include <iostream>
#include <string_view>
#include <vector>

namespace BulletEngine {
namespace interface {

constexpr const char* DOCK_ID = "EngineDockSpace";
constexpr float SIDE_PANEL_FRACTION = 0.20f;
constexpr float CONSOLE_PANEL_FRACTION = 0.25f;
constexpr float VIEW_SPLIT_FRACTION = 0.5f;     // scene and game share middle
constexpr float NAME_FIELD_CHARS = 12.0f;       // save as field width, in font sizes

// where scene panel starts looking from
constexpr glm::vec3 EDITOR_CAMERA_POSITION{0.0f, 5.0f, 6.0f};
constexpr float EDITOR_CAMERA_YAW = -90.0f;
constexpr float EDITOR_CAMERA_PITCH = -35.0f;
constexpr float EDITOR_CAMERA_FOV = 60.0f;
constexpr float EDITOR_CAMERA_SPEED = 3.0f;
constexpr float EDITOR_CAMERA_NEAR = 0.1f;

Editor::Editor(ecs::World& world, ecs::systems::PhysicsSystem& physics, ecs::systems::DebugDrawSystem& debugDraw)
    : m_world(world), m_physics(physics), m_debugDraw(debugDraw),
      m_camera(std::make_unique<BulletRender::scene::FlyCamera>(EDITOR_CAMERA_POSITION, EDITOR_CAMERA_YAW, EDITOR_CAMERA_PITCH,
                                                               EDITOR_CAMERA_FOV, EDITOR_CAMERA_SPEED, EDITOR_CAMERA_NEAR, EDITOR_CAMERA_FAR)),
      m_flatCamera(std::make_unique<BulletRender::scene::PanCamera>()) {}

BulletRender::scene::Camera& Editor::getCamera()
{
    return m_flatView ? static_cast<BulletRender::scene::Camera&>(*m_flatCamera) : *m_camera;
}

void Editor::beforeFrame()
{
    if (!m_themeApplied)
    {
        BulletRender::interface::Theme::apply();
        m_themeApplied = true;
    }

    BulletRender::interface::Fonts::apply();
}

void Editor::draw()
{
    drawDockSpace();

    drawScene();
    drawGame();
    drawHierarchy();
    drawInspector();
    drawConsole();
    drawExplorer();

    if (m_focusPanel)
    {
        ImGui::SetWindowFocus(m_focusPanel);
        m_focusPanel = nullptr;
    }

    applyCommands();
}

void Editor::drawDockSpace()
{
    const ImGuiViewport* viewport = ImGui::GetMainViewport();

    ImGui::SetNextWindowPos(viewport->WorkPos);
    ImGui::SetNextWindowSize(viewport->WorkSize);
    ImGui::SetNextWindowViewport(viewport->ID);

    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));

    const ImGuiWindowFlags flags =
        ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse |
        ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoBringToFrontOnFocus |
        ImGuiWindowFlags_NoNavFocus | ImGuiWindowFlags_MenuBar;

    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.0f, 0.0f, 0.0f, 0.0f));
    ImGui::Begin("DockHost", nullptr, flags);
    ImGui::PopStyleColor();
    ImGui::PopStyleVar(3);

    const ImGuiID dockId = ImGui::GetID(DOCK_ID);
    // tabs carry their own close button
    const ImGuiDockNodeFlags dockFlags =
        ImGuiDockNodeFlags_PassthruCentralNode | static_cast<ImGuiDockNodeFlags>(ImGuiDockNodeFlags_NoCloseButton);

    ImGui::DockSpace(dockId, ImVec2(0.0f, 0.0f), dockFlags);

    if (!m_layoutBuilt)
    {
        m_layoutBuilt = true;
        buildLayout(dockId);
    }

    drawMenuBar();

    ImGui::End();
}

void Editor::buildLayout(unsigned dockId)
{
    ImGui::DockBuilderRemoveNode(dockId);
    ImGui::DockBuilderAddNode(dockId, static_cast<ImGuiDockNodeFlags>(ImGuiDockNodeFlags_DockSpace) | ImGuiDockNodeFlags_PassthruCentralNode);
    ImGui::DockBuilderSetNodeSize(dockId, ImGui::GetMainViewport()->WorkSize);

    ImGuiID center = dockId;
    const ImGuiID left = ImGui::DockBuilderSplitNode(center, ImGuiDir_Left, SIDE_PANEL_FRACTION, nullptr, &center);
    const ImGuiID right = ImGui::DockBuilderSplitNode(center, ImGuiDir_Right, SIDE_PANEL_FRACTION / (1.0f - SIDE_PANEL_FRACTION), nullptr, &center);
    const ImGuiID bottom = ImGui::DockBuilderSplitNode(center, ImGuiDir_Down, CONSOLE_PANEL_FRACTION, nullptr, &center);

    ImGui::DockBuilderDockWindow(HIERARCHY_PANEL, left);
    ImGui::DockBuilderDockWindow(INSPECTOR_PANEL, right);
    ImGui::DockBuilderDockWindow(CONSOLE_PANEL, bottom);
    ImGui::DockBuilderDockWindow(EXPLORER_PANEL, bottom);
    // two views split what is left, scene left and game right
    const ImGuiID game = ImGui::DockBuilderSplitNode(center, ImGuiDir_Right, VIEW_SPLIT_FRACTION, nullptr, &center);

    ImGui::DockBuilderDockWindow(SCENE_PANEL, center);
    ImGui::DockBuilderDockWindow(GAME_PANEL, game);

    ImGui::DockBuilderFinish(dockId);
}

void Editor::openPanel(bool& shown, const char* name)
{
    shown = true;

    // panel opened just now has no window yet, focus waits until it does
    m_focusPanel = name;
}

// key without extension, path is user's to choose
std::string Editor::sceneName() const
{
    if (m_sceneKey.empty())
    {
        return SCENE_DEFAULT_NAME;
    }

    return m_sceneKey.substr(0, m_sceneKey.size() - std::string_view(SCENE_EXTENSION).size());
}

void Editor::saveScene(const std::string& key)
{
    if (!scene::save(m_world, project::Project::instance().getPath(key)))
    {
        std::cerr << "scene save failed: " << key << '\n';
        return;
    }

    m_sceneKey = key;
    project::Project::instance().rescan();
}

void Editor::savePrefab(ecs::Entity entity)
{
    const auto* identity = m_world.get<ecs::IdentityComponent>(entity);
    const std::string key = (identity ? identity->name : std::string("Entity")) + PREFAB_EXTENSION;

    if (!scene::savePrefab(m_world, entity, project::Project::instance().getPath(key)))
    {
        std::cerr << "prefab save failed: " << key << '\n';
        return;
    }

    project::Project::instance().rescan();
}

void Editor::drawProjectMenu()
{
    project::Project& project = project::Project::instance();
    project::Settings settings = project.getSettings();

    if (ImGui::BeginMenu("Name"))
    {
        // field opens on current name, ready to edit or keep
        if (ImGui::IsWindowAppearing())
        {
            std::snprintf(m_projectName, sizeof(m_projectName), "%s", settings.name.c_str());
        }

        ImGui::SetNextItemWidth(ImGui::GetFontSize() * NAME_FIELD_CHARS);

        if (ImGui::InputText("##name", m_projectName, sizeof(m_projectName), ImGuiInputTextFlags_EnterReturnsTrue) && m_projectName[0])
        {
            settings.name = m_projectName;
            project.setSettings(settings);

            ImGui::CloseCurrentPopup();
        }

        ImGui::EndMenu();
    }

    if (ImGui::BeginMenu("Mode"))
    {
        static const project::Mode MODES[] = {project::Mode::Mode2D, project::Mode::Mode3D};

        for (project::Mode mode : MODES)
        {
            if (ImGui::MenuItem(project::toString(mode), nullptr, mode == settings.mode))
            {
                settings.mode = mode;
                project.setSettings(settings);
            }
        }

        ImGui::EndMenu();
    }

    // scene it opens with, and what build would start from
    if (ImGui::BeginMenu("Start Scene"))
    {
        const std::vector<std::string> scenes = project.getKeys(SCENE_EXTENSION);

        if (scenes.empty())
        {
            ImGui::TextDisabled("No scenes");
        }

        for (const std::string& key : scenes)
        {
            if (ImGui::MenuItem(key.c_str(), nullptr, key == settings.startScene))
            {
                settings.startScene = key;
                project.setSettings(settings);
            }
        }

        ImGui::EndMenu();
    }
}

void Editor::drawSceneMenu()
{
    if (ImGui::MenuItem("New"))
    {
        m_pendingClear = true;
    }

    if (ImGui::BeginMenu("Open"))
    {
        const std::vector<std::string> scenes = project::Project::instance().getKeys(SCENE_EXTENSION);

        if (scenes.empty())
        {
            ImGui::TextDisabled("No scenes");
        }

        for (const std::string& key : scenes)
        {
            if (ImGui::MenuItem(key.c_str()) && key != m_sceneKey)
            {
                m_pendingOpen = key;
            }
        }

        ImGui::EndMenu();
    }

    ImGui::Separator();

    // never saved scene has no key to save into
    if (ImGui::MenuItem("Save", nullptr, false, !m_sceneKey.empty()))
    {
        saveScene(m_sceneKey);
    }

    if (ImGui::BeginMenu("Save As"))
    {
        // field opens on current name, ready to edit or keep
        if (ImGui::IsWindowAppearing())
        {
            std::snprintf(m_sceneName, sizeof(m_sceneName), "%s", sceneName().c_str());
        }

        ImGui::SetNextItemWidth(ImGui::GetFontSize() * NAME_FIELD_CHARS);

        const bool entered = ImGui::InputText("##name", m_sceneName, sizeof(m_sceneName),
                                              ImGuiInputTextFlags_EnterReturnsTrue);

        ImGui::SameLine();

        if ((ImGui::Button("Save") || entered) && m_sceneName[0])
        {
            saveScene(m_sceneName + std::string(SCENE_EXTENSION));
            ImGui::CloseCurrentPopup();
        }

        ImGui::EndMenu();
    }
}

// nameless tool is not for menu to touch
void Editor::drawEditorMenu()
{
    for (EditorPass& entry : m_editorPasses)
    {
        if (entry.name && ImGui::MenuItem(entry.name, nullptr, &entry.shown))
        {
            entry.pass->setEnabled(entry.shown);
        }
    }
}

void Editor::drawDebugMenu()
{
    bool colliders = m_debugDraw.isShowColliders();

    if (ImGui::MenuItem("Visible Colliders", nullptr, &colliders))
    {
        m_debugDraw.setShowColliders(colliders);
    }

    bool physics = m_debugDraw.isShowPhysics();

    if (ImGui::MenuItem("Visible Physics", nullptr, &physics))
    {
        m_debugDraw.setShowPhysics(physics);
    }

    bool lights = m_debugDraw.isShowLights();

    if (ImGui::MenuItem("Visible Lights", nullptr, &lights))
    {
        m_debugDraw.setShowLights(lights);
    }

    bool cameras = m_debugDraw.isShowCameras();

    if (ImGui::MenuItem("Visible Cameras", nullptr, &cameras))
    {
        m_debugDraw.setShowCameras(cameras);
    }
}

// play takes snapshot, stop puts world back
void Editor::setMode(Mode mode)
{
    if (mode == m_mode)
    {
        return;
    }

    if (mode == Mode::Play)
    {
        m_snapshot = scene::toNode(m_world);
    }
    else
    {
        m_selection = ecs::INVALID_ENTITY;

        m_world.clear();
        m_world.flush();
        scene::fromNode(m_world, m_snapshot);
    }

    m_mode = mode;

    // after world settled, systems see final state
    for (const ModeListener& listener : m_modeListeners)
    {
        listener(m_mode);
    }
}

void Editor::drawPlayBar()
{
    const char* label = isPlaying() ? "Stop" : "Play";
    const float cursor = ImGui::GetCursorPosX();

    // button sits mid bar, menus after it carry on from where they were
    ImGui::SetCursorPosX((ImGui::GetWindowWidth() - ImGui::CalcTextSize(label).x) * 0.5f);

    if (ImGui::MenuItem(label))
    {
        setMode(isPlaying() ? Mode::Edit : Mode::Play);
    }

    ImGui::SetCursorPosX(cursor);
}

void Editor::drawMenuBar()
{
    if (ImGui::BeginMenuBar())
    {
        if (ImGui::BeginMenu("Project"))
        {
            drawProjectMenu();
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Scene"))
        {
            drawSceneMenu();
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Windows"))
        {
            if (ImGui::MenuItem(SCENE_PANEL))
            {
                openPanel(m_showScene, SCENE_PANEL);
            }

            if (ImGui::MenuItem(GAME_PANEL))
            {
                openPanel(m_showGame, GAME_PANEL);
            }

            if (ImGui::MenuItem(HIERARCHY_PANEL))
            {
                openPanel(m_showHierarchy, HIERARCHY_PANEL);
            }

            if (ImGui::MenuItem(INSPECTOR_PANEL))
            {
                openPanel(m_showInspector, INSPECTOR_PANEL);
            }

            if (ImGui::MenuItem(CONSOLE_PANEL))
            {
                openPanel(m_showConsole, CONSOLE_PANEL);
            }

            if (ImGui::MenuItem(EXPLORER_PANEL))
            {
                openPanel(m_showExplorer, EXPLORER_PANEL);
            }

            ImGui::Separator();

            if (ImGui::MenuItem("Reset"))
            {
                m_showScene = true;
                m_showGame = true;
                m_showHierarchy = true;
                m_showInspector = true;
                m_showConsole = true;
                m_showExplorer = true;
                m_layoutBuilt = false;
            }

            ImGui::EndMenu();
        }

        drawPlayBar();

        if (ImGui::BeginMenu("Settings"))
        {
            if (ImGui::BeginMenu("Editor"))
            {
                drawEditorMenu();
                ImGui::EndMenu();
            }

            if (ImGui::BeginMenu("Debug"))
            {
                drawDebugMenu();
                ImGui::EndMenu();
            }

            ImGui::EndMenu();
        }

        ImGui::EndMenuBar();
    }
}

} // namespace interface
} // namespace BulletEngine
