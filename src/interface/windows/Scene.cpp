/*
 * Scene.cpp
 */

#include "interface/Editor.h"

#include "render/Renderer.h"

#include "imgui.h"

#include <algorithm>

namespace BulletEngine {
namespace interface {

void Editor::applySceneSize()
{
    BulletRender::render::Renderer::setOffscreenSize(static_cast<int>(m_sceneSize.x), static_cast<int>(m_sceneSize.y));
}

void Editor::drawScene()
{
    if (!m_showScene)
    {
        m_sceneSize = {0.0f, 0.0f};
        m_sceneFocused = false;
        return;
    }

    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
    ImGui::Begin(SCENE_PANEL, &m_showScene, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);

    // the camera listens over the scene, and keeps listening until a look ends
    const bool looking = ImGui::IsMouseDown(ImGuiMouseButton_Right);

    m_sceneFocused = looking ? m_sceneFocused : (ImGui::IsWindowFocused() || ImGui::IsWindowHovered());

    const ImVec2 size = ImGui::GetContentRegionAvail();
    m_sceneSize = {std::max(size.x, 0.0f), std::max(size.y, 0.0f)};

    const ImVec2 origin = ImGui::GetCursorScreenPos();

    if (auto* fbo = BulletRender::render::Renderer::getOffscreenFrameBuffer())
    {
        // texture origin sits at the bottom, uv flipped to match
        ImGui::Image(static_cast<ImTextureID>(fbo->getColorTexture()), size, ImVec2(0.0f, 1.0f), ImVec2(1.0f, 0.0f));
    }

    // a click inside the image asks for whatever entity sits under it
    if (ImGui::IsItemClicked())
    {
        const ImVec2 cursor = ImGui::GetMousePos();
        m_scenePick = {cursor.x - origin.x, cursor.y - origin.y};
    }

    ImGui::End();
    ImGui::PopStyleVar();
}

} // namespace interface
} // namespace BulletEngine
