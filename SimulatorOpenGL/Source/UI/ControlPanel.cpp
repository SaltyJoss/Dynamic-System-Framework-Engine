#include "ch.h"

#include <imgui.h>

#include "ControlPanel.h"

void gui::ControlPanel::render(gui::SceneView* sceneView) {
    _mesh = sceneView->getMesh();

    ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(300, 400), ImGuiCond_FirstUseEver);
    ImGui::Begin("Control Panel", nullptr, ImGuiWindowFlags_NoCollapse);

    if (ImGui::Button("Open")) { _fileDialog.Open(); }
    ImGui::SameLine(0, 5.0f);
    ImGui::Text(_currentFile.c_str());

    if (ImGui::CollapsingHeader("Simulation Settings")) { renderSimulationProperties(); renderObjectProperties(); renderLinkProperties(); renderStats(); }
    if (ImGui::CollapsingHeader("Camera Settings")) { renderCameraProperties(); }
    if (ImGui::CollapsingHeader("Display Settings")) { renderDisplaySettings(); }

    ImGui::End();

    _fileDialog.Display();
    if (_fileDialog.HasSelected()) {
        auto file_path = _fileDialog.GetSelected().string();
        _currentFile = file_path.substr(file_path.find_last_of("/\\") + 1);
        _meshLoadCallback(file_path);

        _fileDialog.ClearSelected();
    }
}

void gui::ControlPanel::renderSimulationProperties() {
    ImGui::SeparatorText("Simulation Controls");
    static bool physicsEnabled = true;
    static bool gravityEnabled = true;
    ImGui::Checkbox("Enable Physcics", &physicsEnabled);
    ImGui::Checkbox("Enable Gravity", &gravityEnabled);

    ImGui::SeparatorText("Light Controls");
    ImGui::Text("Position");
}

void gui::ControlPanel::renderObjectProperties() {
    ImGui::SeparatorText("Object Controls");
    static char velocityBuf[64] = "";
    static char torqueBuf[64] = "";
    ImGui::InputText("Velocity", velocityBuf, IM_ARRAYSIZE(velocityBuf));
    ImGui::InputText("Torque", torqueBuf, IM_ARRAYSIZE(torqueBuf));

    ImGui::SeparatorText("Object Appearance");
    if (!_mesh) {
        ImGui::Text("No mesh loaded!");
    }
    else {
        ImGui::ColorPicker3("Color", glm::value_ptr(_mesh->_colour), ImGuiColorEditFlags_PickerHueWheel | ImGuiColorEditFlags_DisplayRGB);
        ImGui::SliderFloat("Metallic", &_mesh->_metallic, 0.0f, 1.0f);
    }
}

void gui::ControlPanel::renderLinkProperties() {
    ImGui::SeparatorText("Link Controls");
    static char linkLengthBuf[64] = "";
    static char dampingBuf[64] = "";
    ImGui::InputText("Link Length", linkLengthBuf, IM_ARRAYSIZE(linkLengthBuf));
    ImGui::InputText("Damping", dampingBuf, IM_ARRAYSIZE(dampingBuf));
}

void gui::ControlPanel::renderStats() {
    ImGui::SeparatorText("Simulation Statistics");
    static float data[100];
    for (int i = 0; i < 100; i++) data[i] = sinf(i * 0.1f);
    ImGui::PlotLines("Velocity", data, 100);
}

void gui::ControlPanel::renderCameraProperties() {
    ImGui::SeparatorText("Camera Controls");
    static char text[64] = "";
    ImGui::InputText("Camera Label", text, IM_ARRAYSIZE(text));
    static bool enabled = true;
    ImGui::Checkbox("Enable Camera", &enabled);
}

void gui::ControlPanel::renderDisplaySettings() {
    ImGui::SeparatorText("Display Settings");
}

/*
OLD LOGIC -- DELETE SOON, KEEP WHILE STILL TUNING NEW LOGIC

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

*/
