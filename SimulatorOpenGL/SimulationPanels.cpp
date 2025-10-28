#include "SimulationPanels.h"
#include <iostream>
#include <string>

void SimulationPanels::Render() {
    ImGuiWindowFlags panelFlags = ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize;
    Container(panelFlags);
    MainPanel(panelFlags);

}

void SimulationPanels::Container(ImGuiWindowFlags panelFlags) {
    ImGui::SetNextWindowPos(ImVec2(400, 0));
    ImGui::SetNextWindowSize(ImVec2(ImGui::GetIO().DisplaySize.x, ImGui::GetIO().DisplaySize.y));
    ImGui::Begin("Simulation Container", nullptr, panelFlags);
    ImGui::End();
}

void SimulationPanels::MainPanel(ImGuiWindowFlags panelFlags) {
    ImGui::SetNextWindowPos(ImVec2(415, 15));
    ImGui::SetNextWindowSize(ImVec2((ImGui::GetIO().DisplaySize.x)-15, (ImGui::GetIO().DisplaySize.y)-15));
    ImGui::Begin("Simulation", nullptr, panelFlags);
    ImGui::Text("3D render goes here");
    ImGui::End();
}

void SimulationPanels::Panel1(ImGuiWindowFlags panelFlags) {
    /* STUBBED */
}

void SimulationPanels::Panel2(ImGuiWindowFlags panelFlags) {
    /* STUBBED */
}

void SimulationPanels::Panel3(ImGuiWindowFlags panelFlags) {
    /* STUBBED */
}

void SimulationPanels::Panel4(ImGuiWindowFlags panelFlags) {
    /* STUBBED */
}