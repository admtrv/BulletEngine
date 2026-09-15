/*
 * Editor.cpp
 */

#include "Editor.h"

#include "interface/elements/Fonts.h"
#include "interface/elements/Theme.h"
#include "project/Project.h"
#include "scene/Serializer.h"

#include "imgui.h"
#include "imgui_internal.h"

#include <cstdio>
#include <iostream>
#include <string_view>
#include <vector>

namespace BulletEngine {
namespace interface {

constexpr const char* DOCK_ID = "EngineDockSpace";
constexpr float SIDE_PANEL_FRACTION = 0.20f;
constexpr float CONSOLE_PANEL_FRACTION = 0.25f;
constexpr float NAME_FIELD_CHARS = 12.0f;       // save as field width, in font sizes

Editor::Editor(ecs::World& world, ecs::systems::PhysicsSystem& physics, ecs::systems::DebugDrawSystem& debugDraw)
    : m_world(world), m_physics(physics), m_debugDraw(debugDraw) {}

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
    ImGui::DockBuilderDockWindow(SCENE_PANEL, center);

    ImGui::DockBuilderFinish(dockId);
}

void Editor::openPanel(bool& shown, const char* name)
{
    shown = true;

    // a panel opened just now has no window yet, focus waits until it does
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
            if (ImGui::MenuItem(key.c_str(), nullptr, key == m_sceneKey))
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
}

void Editor::drawMenuBar()
{
    if (ImGui::BeginMenuBar())
    {
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
                m_showHierarchy = true;
                m_showInspector = true;
                m_showConsole = true;
                m_showExplorer = true;
                m_layoutBuilt = false;
            }

            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Settings"))
        {
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
