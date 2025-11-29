/*
 * ImGuiSystem.cpp
 */

#include "ImGuiSystem.h"

namespace BulletEngine {
namespace ecs {
namespace systems {

ImGuiSystem::ImGuiSystem(BulletPhysic::dynamics::PhysicsWorld& physicsWorld)
    : m_physicsWorld(physicsWorld)
{}

void ImGuiSystem::render()
{
    ImGui::Begin("Forces");

    ImGui::BeginTable("ForcesTable", 3, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg);

    ImGui::TableSetupColumn("Name");
    ImGui::TableSetupColumn("Symbol");
    ImGui::TableSetupColumn("Magnitude (N)");
    ImGui::TableHeadersRow();

    for (const auto& force : m_physicsWorld.getForces())
    {
        if (force && force->isActive())
        {
            float magnitude = force->getForce().length();

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::Text("%s", force->getName().c_str());

            ImGui::TableSetColumnIndex(1);
            ImGui::Text("%s", force->getSymbol().c_str());

            ImGui::TableSetColumnIndex(2);
            ImGui::Text("%.8f", magnitude);
        }
    }

    ImGui::EndTable();
    ImGui::End();
}

} // namespace systems
} // namespace ecs
} // namespace BulletEngine
