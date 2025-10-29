#include "ControlPanel.h"
#include <iostream>
#include <string>

void ControlPanel::Render(ImVec2 winSize, ImVec2 padding, float debugHeight, float ctrlPanelWidth)
{
    ImGui::SetCursorPos(ImVec2(padding.x, padding.y));
    ImGui::BeginChild("ControlPanel", ImVec2(ctrlPanelWidth - padding.x * 2, winSize.y - debugHeight - padding.y * 2), false);
    ImGui::Text("CONTROL PANEL");
    ImGui::Separator();

    RenderSimulationControls();
    RenderObjectControls();
    RenderLinkControls();
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
    ImGui::SeparatorText("Simulation Controls");
    static bool physicsEnabled = true;
    static bool gravityEnabled = true;

    ImGui::Checkbox("Enable Physcics", &physicsEnabled);
    ImGui::Checkbox("Enable Gravity", &gravityEnabled);
}

void ControlPanel::RenderCameraControls()
{
    ImGui::SeparatorText("Camera Controls");
    static char text[64] = "";
    ImGui::InputText("Enter text", text, IM_ARRAYSIZE(text));

    ImGui::Text("Controls");
    static bool enabled = true;
    ImGui::Checkbox("Enable Feature", &enabled);
}

void ControlPanel::RenderObjectControls()
{
    ImGui::SeparatorText("Object Controls");

    static char velocityBuf[64] = "";
    static char torqueBuf[64] = "";

    ImGui::InputText("Velocity", velocityBuf, IM_ARRAYSIZE(velocityBuf));
    ImGui::InputText("Torque", torqueBuf, IM_ARRAYSIZE(torqueBuf));
}

void ControlPanel::RenderLinkControls()
{
    ImGui::SeparatorText("Link Controls");

    static char linkLengthBuf[64] = "";
    static char dampingBuf[64] = "";

    ImGui::InputText("Link Length", linkLengthBuf, IM_ARRAYSIZE(linkLengthBuf));
    ImGui::InputText("Damping", dampingBuf, IM_ARRAYSIZE(dampingBuf));
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