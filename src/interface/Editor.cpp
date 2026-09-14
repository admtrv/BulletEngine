/*
 * Editor.cpp
 */

#include "Editor.h"

#include "interface/elements/Fonts.h"
#include "interface/elements/Theme.h"
#include "scene/Serializer.h"

#include "imgui.h"
#include "imgui_internal.h"

namespace BulletEngine {
namespace interface {

constexpr const char* DOCK_ID = "EngineDockSpace";
constexpr float SIDE_PANEL_FRACTION = 0.20f;
constexpr float CONSOLE_PANEL_FRACTION = 0.25f;

Editor::Editor(ecs::World& world, ecs::systems::PhysicsSystem& physics) : m_world(world), m_physics(physics) {}

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
    ImGui::DockBuilderDockWindow(SCENE_PANEL, center);

    ImGui::DockBuilderFinish(dockId);
}

void Editor::openPanel(bool& shown, const char* name)
{
    shown = true;

    // a panel opened just now has no window yet, focus waits until it does
    m_focusPanel = name;
}

void Editor::drawMenuBar()
{
    if (ImGui::BeginMenuBar())
    {
        if (ImGui::BeginMenu("File"))
        {
            if (ImGui::MenuItem("Save"))
            {
                scene::save(m_world, m_scenePath);
            }

            if (ImGui::MenuItem("Load"))
            {
                m_pendingLoad = true;
            }

            ImGui::Separator();

            if (ImGui::MenuItem("Clear"))
            {
                m_pendingClear = true;
            }

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

            ImGui::Separator();

            if (ImGui::MenuItem("Reset"))
            {
                m_showScene = true;
                m_showHierarchy = true;
                m_showInspector = true;
                m_showConsole = true;
                m_layoutBuilt = false;
            }

            ImGui::EndMenu();
        }

        ImGui::EndMenuBar();
    }
}

} // namespace interface
} // namespace BulletEngine
