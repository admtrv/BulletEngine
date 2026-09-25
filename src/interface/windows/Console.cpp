/*
 * Console.cpp
 */

#include "interface/Editor.h"

#include "io/Log.h"

#include "imgui.h"


namespace BulletEngine {
namespace interface {

void Editor::drawConsole()
{
    if (!m_showConsole)
    {
        return;
    }

    ImGui::Begin(CONSOLE_PANEL, &m_showConsole);

    // journal is copied only when it moved, field reads it every frame
    const uint32_t revision = io::Log::instance().getRevision();

    if (m_consoleRevision != revision)
    {
        m_consoleRevision = revision;
        m_consoleText = io::Log::instance().getText();

        m_consoleTail = true;
    }

    ImGui::BeginChild("lines", {0.0f, 0.0f}, ImGuiChildFlags_None, ImGuiWindowFlags_HorizontalScrollbar);

    ImGui::TextUnformatted(m_consoleText.c_str());

    if (m_consoleTail)
    {
        ImGui::SetScrollHereY(1.0f);
        m_consoleTail = false;
    }

    ImGui::EndChild();
    ImGui::End();
}

} // namespace interface
} // namespace BulletEngine
