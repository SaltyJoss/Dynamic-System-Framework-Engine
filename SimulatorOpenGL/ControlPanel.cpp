#include "ControlPanel.h"
#include <iostream>
#include <string>

void ControlPanel::Render()
{
    // Position and size for the control panel
    ImGui::SetNextWindowPos(ImVec2(0, 0));
    ImGui::SetNextWindowSize(ImVec2(300, ImGui::GetIO().DisplaySize.y)); // dynamic height
    ImGuiWindowFlags panelFlags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove;
    ImGui::Begin("Control Panel", nullptr, panelFlags);

    // Example widget
    static char text[128] = "";
    ImGui::InputText("Enter text", text, IM_ARRAYSIZE(text));

    RenderSimulationControls();
    RenderCameraControls();
    RenderDisplaySettings();
    RenderStats();

    ImGui::End();

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