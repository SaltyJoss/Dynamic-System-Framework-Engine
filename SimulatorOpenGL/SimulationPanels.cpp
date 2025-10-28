#include "SimulationPanels.h"
#include <iostream>
#include <string>

void SimulationPanels::Render(ImVec2 winSize, ImVec2 padding, float debugHeight, float ctrlPanelWidth)
{
    ImGuiWindowFlags simPanelFlags =
        ImGuiWindowFlags_NoTitleBar
        | ImGuiWindowFlags_NoResize
        | ImGuiWindowFlags_NoScrollbar
        | ImGuiWindowFlags_NoScrollWithMouse;

    ImGui::SetCursorPos(ImVec2(ctrlPanelWidth + padding.x, padding.y));
    ImGui::BeginChild("Simulation", ImVec2(winSize.x - ctrlPanelWidth - padding.x * 2, winSize.y - debugHeight - padding.y * 2), true, simPanelFlags);
    if (ImGui::BeginTabBar("Simulation")) {
        if (ImGui::BeginTabItem("Main Simulation")) {
            MainPanel();
            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("4-Fixed Simulation Angles")) {
            SecondaryPanels();
            ImGui::EndTabItem();
        }

        ImGui::EndTabBar();
    }
    ImGui::EndChild();
}

void SimulationPanels::MainPanel() {
    ImVec2 avail = ImGui::GetContentRegionAvail();
    float w = std::max(avail.x, 1.0f);
    float h = std::max(avail.y, 1.0f);

    ImVec2 pos = ImGui::GetCursorScreenPos();
    ImDrawList* draw = ImGui::GetWindowDrawList();
    draw->AddRectFilled(pos, ImVec2(pos.x + w, pos.y + h), IM_COL32(100,150,250,255));
    draw->AddRect(pos, ImVec2(pos.x + w, pos.y + h), IM_COL32(255,255,255,255));
    draw->AddText(ImVec2(pos.x + 5, pos.y + 5), IM_COL32(255,255,255,255), "Main Simulation");
}

void SimulationPanels::SecondaryPanels() {
    ImVec2 avail = ImGui::GetContentRegionAvail();
    float spacing = 25.0f;
    float panelW = std::max((avail.x - spacing) / 2.0f, 1.0f);
    float panelH = std::max((avail.y - spacing) / 2.0f, 1.0f);

    ImDrawList* draw = ImGui::GetWindowDrawList();

    // Lambda to draw a colored mock simulator
    auto DrawMock = [&](const char* label, ImVec2 pos) {
        ImVec2 screenPos = ImGui::GetCursorScreenPos();
        draw->AddRectFilled(ImVec2(screenPos.x + pos.x, screenPos.y + pos.y),
            ImVec2(screenPos.x + pos.x + panelW, screenPos.y + pos.y + panelH),
            IM_COL32(100, 150, 250, 255));
        draw->AddRect(ImVec2(screenPos.x + pos.x, screenPos.y + pos.y),
            ImVec2(screenPos.x + pos.x + panelW, screenPos.y + pos.y + panelH),
            IM_COL32(255, 255, 255, 255));
        draw->AddText(ImVec2(screenPos.x + pos.x + 5, screenPos.y + pos.y + 5), IM_COL32(255, 255, 255, 255), label);
        };

    // Draw all 4 panels in a 2x2 grid
    DrawMock("Angle-1", ImVec2(0, 0));
    DrawMock("Angle-2", ImVec2(panelW + spacing, 0));
    DrawMock("Angle-3", ImVec2(0, panelH + spacing));
    DrawMock("Angle-4", ImVec2(panelW + spacing, panelH + spacing));

    // Optional: leave dummy children to reserve layout space
    ImGui::Dummy(ImVec2(avail.x, avail.y));
}