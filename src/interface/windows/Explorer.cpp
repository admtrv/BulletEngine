/*
 * Explorer.cpp
 */

#include "interface/Editor.h"

#include "project/Project.h"

#include "interface/elements/Widgets.h"
#include "imgui.h"

namespace BulletEngine {
namespace interface {

void Editor::drawExplorer()
{
    if (!m_showExplorer)
    {
        return;
    }

    ImGui::Begin(EXPLORER_PANEL, &m_showExplorer);

    const project::Entry& root = project::Project::instance().getTree();

    // own tree, guides start fresh
    m_explorerTree.reset();
    m_explorerTree.setRootless(true);

    // dropping on panel moves back to root
    acceptEntryDrop({});

    for (size_t i = 0; i < root.children.size(); i++)
    {
        drawEntry(root.children[i], i + 1 == root.children.size());
    }

    applyEntryCommands();

    ImGui::End();
}

// either rebuilds the tree, so both wait until it is walked
void Editor::applyEntryCommands()
{
    project::Project& project = project::Project::instance();

    if (!m_pendingMove.first.empty())
    {
        project.move(m_pendingMove.first, m_pendingMove.second);
        m_pendingMove = {};
    }

    if (!m_pendingDelete.empty())
    {
        if (m_pendingDelete == m_sceneKey)
        {
            m_sceneKey.clear();
        }

        project.remove(m_pendingDelete);
        m_pendingDelete.clear();
    }
}

// dragged entry lands in named folder, empty means root
void Editor::acceptEntryDrop(const std::string& folder)
{
    if (!ImGui::BeginDragDropTarget())
    {
        return;
    }

    for (const char* type : {ASSET_DRAG_TYPE, FOLDER_DRAG_TYPE})
    {
        if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload(type))
        {
            m_pendingMove = {static_cast<const char*>(payload->Data), folder};
        }
    }

    ImGui::EndDragDropTarget();
}

void Editor::drawEntry(const project::Entry& entry, bool last)
{
    const bool folded = m_folded.count(entry.key) != 0;

    if (m_explorerTree.row(&entry, entry.name.c_str(), last, m_explorerSelection == entry.key, entry.directory, folded))
    {
        m_explorerSelection = entry.key;
    }

    if (m_explorerTree.toggled())
    {
        if (folded)
        {
            m_folded.erase(entry.key);
        }
        else
        {
            m_folded.insert(entry.key);
        }
    }

    // menus and payloads share the row, id keeps them apart per entry
    ImGui::PushID(entry.key.c_str());

    BulletRender::interface::contextMenu("entry", [&]() {
        if (ImGui::MenuItem("Delete"))
        {
            m_pendingDelete = entry.key;
        }
    });

    // file carries key to asset field, both kinds carry it to folder
    if (ImGui::BeginDragDropSource())
    {
        ImGui::SetDragDropPayload(entry.directory ? FOLDER_DRAG_TYPE : ASSET_DRAG_TYPE,
                                  entry.key.c_str(), entry.key.size() + 1);

        ImGui::TextUnformatted(entry.name.c_str());
        ImGui::EndDragDropSource();
    }

    if (entry.directory)
    {
        acceptEntryDrop(entry.key);
    }

    ImGui::PopID();

    if (!entry.directory || folded || entry.children.empty())
    {
        return;
    }

    m_explorerTree.push(last);

    for (size_t i = 0; i < entry.children.size(); i++)
    {
        drawEntry(entry.children[i], i + 1 == entry.children.size());
    }

    m_explorerTree.pop();
}

} // namespace interface
} // namespace BulletEngine
