/*
 * Scene.cpp
 */

#include "interface/Editor.h"

#include "ecs/Components.h"

#include "render/Renderer.h"

#include "imgui.h"

#include <algorithm>

namespace BulletEngine {
namespace interface {

// keeps view sized to its panel, empty panel drops it
static void resizeView(std::unique_ptr<BulletRender::render::FrameBuffer>& view, const glm::vec2& size)
{
    const int width = static_cast<int>(size.x);
    const int height = static_cast<int>(size.y);

    if (width <= 0 || height <= 0)
    {
        view.reset();
        return;
    }

    if (!view)
    {
        view = std::make_unique<BulletRender::render::FrameBuffer>(width, height);
    }
    else if (view->getWidth() != width || view->getHeight() != height)
    {
        view->resize(width, height);
    }
}

// entity the world plays through, marked main or first there is
static ecs::Entity findGameCamera(ecs::World& world)
{
    ecs::Entity found = ecs::INVALID_ENTITY;

    for (ecs::Entity entity : world.getEntities())
    {
        const auto* camera = world.get<ecs::CameraComponent>(entity);

        if (!camera || !world.get<ecs::TransformComponent>(entity))
        {
            continue;
        }

        if (camera->main)
        {
            return entity;
        }

        if (found == ecs::INVALID_ENTITY)
        {
            found = entity;
        }
    }

    return found;
}

void Editor::renderViews(BulletRender::scene::Scene& scene)
{
    resizeView(m_sceneView, m_sceneSize);
    resizeView(m_gameView, m_gameSize);

    if (m_sceneView)
    {
        // scene is the world being arranged, player interface has no place over it
        for (auto& pass : m_gamePasses)
        {
            pass->setEnabled(false);
        }

        scene.setActiveCamera(m_camera.get());
        BulletRender::render::Renderer::renderTo(scene, *m_sceneView);

        for (auto& pass : m_gamePasses)
        {
            pass->setEnabled(true);
        }
    }

    if (!m_gameView)
    {
        return;
    }

    const ecs::Entity entity = findGameCamera(m_world);

    // nothing to look through, panel stays empty instead of holding last frame
    if (entity == ecs::INVALID_ENTITY)
    {
        m_gameView->bind();
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        m_gameView->unbind();

        return;
    }

    // game sees world as player would, without tools laid over it
    for (auto& pass : m_editorPasses)
    {
        pass->setEnabled(false);
    }

    if (!m_gameCamera)
    {
        m_gameCamera = std::make_shared<BulletRender::scene::StaticCamera>(glm::vec3(0.0f), glm::vec3(0.0f, 0.0f, -1.0f));
    }

    const auto& component = *m_world.get<ecs::CameraComponent>(entity);
    const auto& transform = m_world.get<ecs::TransformComponent>(entity)->transform;

    // static camera looks at a point, entity says which way
    const glm::vec3 position = transform.getPosition();

    m_gameCamera->setPosition(position);
    m_gameCamera->setTarget(position + transform.getForward());
    m_gameCamera->setProjection(component.projection);
    m_gameCamera->setFov(component.fov);
    m_gameCamera->setHeight(component.height);
    m_gameCamera->setClipPlanes(component.nearPlane, component.farPlane);

    scene.setActiveCamera(m_gameCamera.get());
    BulletRender::render::Renderer::renderTo(scene, *m_gameView);

    for (auto& pass : m_editorPasses)
    {
        pass->setEnabled(true);
    }
}

// panel showing one view, tells where its image was clicked
static bool drawView(const char* title, bool& shown, const BulletRender::render::FrameBuffer* view,
                     glm::vec2& size, glm::vec2& clicked)
{
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
    ImGui::Begin(title, &shown, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);

    const ImVec2 available = ImGui::GetContentRegionAvail();
    size = {std::max(available.x, 0.0f), std::max(available.y, 0.0f)};

    if (!view)
    {
        return false;
    }

    const ImVec2 origin = ImGui::GetCursorScreenPos();

    // texture origin sits at the bottom, uv flipped to match
    ImGui::Image(static_cast<ImTextureID>(view->getColorTexture()), available, ImVec2(0.0f, 1.0f), ImVec2(1.0f, 0.0f));

    if (ImGui::IsItemClicked())
    {
        const ImVec2 cursor = ImGui::GetMousePos();
        clicked = {cursor.x - origin.x, cursor.y - origin.y};

        return true;
    }

    return false;
}

void Editor::drawScene()
{
    if (!m_showScene)
    {
        m_sceneSize = {0.0f, 0.0f};
        m_sceneFocused = false;
        return;
    }

    // click asks for whatever entity sits under it
    drawView(SCENE_PANEL, m_showScene, m_sceneView.get(), m_sceneSize, m_scenePick);

    // the camera listens over the scene, and keeps listening until a look ends
    const bool looking = ImGui::IsMouseDown(ImGuiMouseButton_Right);

    m_sceneFocused = looking ? m_sceneFocused : (ImGui::IsWindowFocused() || ImGui::IsWindowHovered());

    ImGui::End();
    ImGui::PopStyleVar();
}

void Editor::drawGame()
{
    if (!m_showGame)
    {
        m_gameSize = {0.0f, 0.0f};
        return;
    }

    glm::vec2 ignored;
    drawView(GAME_PANEL, m_showGame, m_gameView.get(), m_gameSize, ignored);

    ImGui::End();
    ImGui::PopStyleVar();
}

} // namespace interface
} // namespace BulletEngine
