/*
 * Console.cpp
 */

#include "interface/Editor.h"

#include "io/Log.h"

#include "imgui.h"
#include "imgui_stdlib.h"

#include <algorithm>
#include <cfloat>

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
        m_consoleLines = float(std::count(m_consoleText.begin(), m_consoleText.end(), '\n') + 1);

        m_consoleTail = true;
    }

    ImGui::BeginChild("lines", {0.0f, 0.0f}, ImGuiChildFlags_None, ImGuiWindowFlags_HorizontalScrollbar);

    // field spans whole text, child window around it is what scrolls
    ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0.0f, 0.0f, 0.0f, 0.0f));
    ImGui::InputTextMultiline("##log", &m_consoleText, {-FLT_MIN, ImGui::GetTextLineHeight() * m_consoleLines},
                              ImGuiInputTextFlags_ReadOnly);
    ImGui::PopStyleColor();

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
