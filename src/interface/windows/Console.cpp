/*
 * Console.cpp
 */

#include "interface/Editor.h"

#include "io/Log.h"

#include "imgui.h"

#include <algorithm>


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

    // field holds whole text, so child scrolls instead of it and text stays selectable
    const ImVec2 text = ImGui::CalcTextSize(m_consoleText.c_str());
    const ImVec2 padding = ImGui::GetStyle().FramePadding;

    const ImVec2 size{std::max(text.x + padding.x * 2.0f, ImGui::GetContentRegionAvail().x),
                      std::max(text.y + padding.y * 2.0f, ImGui::GetContentRegionAvail().y)};

    ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0.0f, 0.0f, 0.0f, 0.0f));

    ImGui::InputTextMultiline("##log", m_consoleText.data(), m_consoleText.size() + 1, size,
                              ImGuiInputTextFlags_ReadOnly | ImGuiInputTextFlags_NoHorizontalScroll);

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
