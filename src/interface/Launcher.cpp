/*
 * Launcher.cpp
 */

#include "Launcher.h"

#include "project/Project.h"
#include "project/Recent.h"

#include "interface/elements/Widgets.h"

#include "imgui.h"

#include <cstdio>
#include <cstdlib>
#include <filesystem>

namespace BulletEngine {
namespace interface {

namespace fs = std::filesystem;

constexpr const char* LAUNCHER_TITLE = "Projects";
constexpr const char* CREATE_TITLE = "New Project";
constexpr const char* FIRST_SCENE = "Scene.scene";      // beside project file, sorting assets is up to user

constexpr float CARD_HEIGHT = 28.0f;
constexpr float FORM_WIDTH = 420.0f;

bool Launcher::open(const std::string& path)
{
    if (!project::Project::instance().open(path))
    {
        m_error = "cannot open " + path;
        return false;
    }

    project::Recent::instance().add(path);
    return true;
}

void Launcher::drawBar()
{
    if (ImGui::BeginMenuBar())
    {
        if (ImGui::MenuItem("New"))
        {
            m_creating = true;
            m_error.clear();
        }

        if (ImGui::MenuItem("Open"))
        {
            m_browser.open = true;
            m_error.clear();
        }

        ImGui::EndMenuBar();
    }

    if (m_browser.open)
    {
        std::string picked;

        if (BulletRender::interface::fileBrowser(m_browser.title, m_browser, picked))
        {
            m_browser.open = false;

            if (!picked.empty())
            {
                open(picked);
            }
        }
    }
}

// name then path, forgotten by button at far end
void Launcher::drawCard(const std::string& path)
{
    const float buttonWidth = ImGui::CalcTextSize("Delete").x + ImGui::GetStyle().FramePadding.x * 2.0f;
    const float spacing = ImGui::GetStyle().ItemInnerSpacing.x;
    const float padding = ImGui::GetStyle().FramePadding.x;

    const float cardWidth = ImGui::GetContentRegionAvail().x;
    const ImVec2 corner = ImGui::GetCursorScreenPos();

    // folder may be gone since it was last opened, saying so beats failing on click
    const bool there = fs::is_directory(path);

    ImGui::BeginDisabled(!there);

    if (ImGui::Selectable("##card", false, ImGuiSelectableFlags_None, {cardWidth, CARD_HEIGHT}))
    {
        open(path);
    }

    // drawn back over row selectable took
    ImGui::SetCursorScreenPos({corner.x + padding, corner.y + padding});
    ImGui::TextUnformatted(fs::path(path).filename().string().c_str());

    ImGui::SameLine();
    ImGui::TextDisabled("%s", path.c_str());

    ImGui::EndDisabled();

    ImGui::SetCursorScreenPos({corner.x + cardWidth - buttonWidth - spacing, corner.y});

    if (ImGui::Button("Delete", {buttonWidth, CARD_HEIGHT}))
    {
        m_pendingDelete = path;
    }
}

void Launcher::drawRecent()
{
    // card may open project, which reorders very list being walked
    const std::vector<std::string> paths = project::Recent::instance().getPaths();

    for (const std::string& path : paths)
    {
        ImGui::PushID(path.c_str());
        drawCard(path);
        ImGui::PopID();
    }
}

void Launcher::drawCreate()
{
    ImGui::SetNextWindowSize({FORM_WIDTH, 0.0f});

    if (!ImGui::IsPopupOpen(CREATE_TITLE))
    {
        ImGui::OpenPopup(CREATE_TITLE);
    }

    if (!ImGui::BeginPopupModal(CREATE_TITLE, nullptr, ImGuiWindowFlags_NoResize))
    {
        return;
    }

    // starts somewhere sensible, so field is edited rather than filled
    if (m_folderSlot.path[0] == '\0')
    {
        const char* home = std::getenv("HOME");
        std::snprintf(m_folderSlot.path, sizeof(m_folderSlot.path), "%s", home ? home : ".");
    }

    BulletRender::interface::inputTextField("Name", m_name, sizeof(m_name));

    if (BulletRender::interface::assetField("Folder", m_folderSlot.path, m_folderSlot.path[0] != '\0', m_folderSlot) == BulletRender::interface::AssetAction::Clear)
    {
        m_folderSlot.path[0] = '\0';
    }

    int mode = m_mode == project::Mode::Mode2D ? 0 : 1;
    static const char* const MODES[] = {"2D", "3D"};

    if (BulletRender::interface::comboField("Mode", mode, MODES, 2))
    {
        m_mode = mode == 0 ? project::Mode::Mode2D : project::Mode::Mode3D;
    }

    BulletRender::interface::errorText(m_error);

    if (ImGui::Button("Cancel"))
    {
        m_creating = false;
        m_error.clear();

        ImGui::CloseCurrentPopup();
    }

    ImGui::SameLine();

    if (ImGui::Button("Create") && m_name[0])
    {
        project::Settings settings;
        settings.name = m_name;
        settings.startScene = FIRST_SCENE;
        settings.mode = m_mode;

        // project lives in folder of its own, named after itself
        const std::string folder = (fs::path(m_folderSlot.path) / settings.name).generic_string();

        if (project::Project::create(folder, settings) && open(folder))
        {
            m_creating = false;
            ImGui::CloseCurrentPopup();
        }
        else if (m_error.empty())
        {
            m_error = "cannot create " + folder;
        }
    }

    ImGui::EndPopup();
}

void Launcher::draw()
{
    const ImGuiViewport* viewport = ImGui::GetMainViewport();

    ImGui::SetNextWindowPos(viewport->WorkPos);
    ImGui::SetNextWindowSize(viewport->WorkSize);
    ImGui::SetNextWindowViewport(viewport->ID);

    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);

    const ImGuiWindowFlags flags =
        ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse |
        ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoBringToFrontOnFocus |
        ImGuiWindowFlags_NoNavFocus | ImGuiWindowFlags_MenuBar;

    ImGui::Begin(LAUNCHER_TITLE, nullptr, flags);
    ImGui::PopStyleVar(2);

    drawBar();
    drawRecent();
    BulletRender::interface::errorText(m_error);

    ImGui::End();

    if (m_creating)
    {
        drawCreate();
    }

    // list is walked while drawing, so what it loses waits for end of frame
    if (!m_pendingDelete.empty())
    {
        project::Recent::instance().remove(m_pendingDelete);
        m_pendingDelete.clear();
    }
}

} // namespace interface
} // namespace BulletEngine
