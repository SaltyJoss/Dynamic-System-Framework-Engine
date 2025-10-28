#include "ControlPanel.h"
#include <iostream>
#include <string>

void ControlPanel::Render(ImVec2 winSize, ImVec2 padding, float debugHeight, float ctrlPanelWidth)
{
    ImGui::SetCursorPos(ImVec2(padding.x, padding.y));
    ImGui::BeginChild("ControlPanel", ImVec2(ctrlPanelWidth - padding.x * 2, winSize.y - debugHeight - padding.y * 2), true);

    static char text[128] = "";
    ImGui::InputText("Enter text", text, IM_ARRAYSIZE(text));

    ImGui::Text("Controls");
    static bool enabled = true;
    ImGui::Checkbox("Enable Feature", &enabled);

    RenderSimulationControls();
    RenderCameraControls();
    RenderDisplaySettings();
    RenderStats();

    ImGui::EndChild();
}

void ControlPanel::HandleInput()
{

}

void ControlPanel::RenderSimulationControls()
{

}

void ControlPanel::RenderCameraControls()
{

}

void ControlPanel::RenderObjectControls()
{
    ImGui::SeparatorText("Object Controls");

    static char velocityBuf[64] = "";
    static char torqueBuf[64] = "";

    ImGui::InputText("Velocity", velocityBuf, IM_ARRAYSIZE(velocityBuf));
    ImGui::InputText("Torque", torqueBuf, IM_ARRAYSIZE(torqueBuf));

    // Convert text to float when needed
    velocity = std::stof(velocityBuf);
    torque = std::stof(torqueBuf);

}

void ControlPanel::RenderLinkControls()
{
    ImGui::SeparatorText("Link Settings");

    static char linkLengthBuf[64] = "";
    static char dampingBuf[64] = "";

    ImGui::InputText("Link Length", linkLengthBuf, IM_ARRAYSIZE(linkLengthBuf));
    ImGui::InputText("Damping", dampingBuf, IM_ARRAYSIZE(dampingBuf));

    linkLength = std::atof(linkLengthBuf);
    damping = std::atof(dampingBuf);
}

void ControlPanel::RenderDisplaySettings()
{

}

void ControlPanel::RenderStats()
{

}

void ControlPanel::SimulationStats(const char* label, int* idx, float* velocity, float* torque, float* damping, float* position)
{
    ImGui::PlotLines("Sin", [](void* velocity, int idx) { return sinf(idx * 0.2f); }, NULL, 100);
    ImGui::PlotLines("Cos", [](void* velocity, int idx) { return cosf(idx * 0.2f); }, NULL, 100);
}