#include <GLFW/glfw3.h>
#include "ControlPanel.h"
#include "GUIManager.h"

// Begin Frame Method
void GUIManager::BeginFrame() 
{
	ImGui_ImplOpenGL3_NewFrame();
	ImGui_ImplGlfw_NewFrame();
	ImGui::NewFrame();
}

// End Framew Method
void GUIManager::EndFrame()
{
	ImGui::Render();
	ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

// Draw Panel method 
void GUIManager::DrawPanel() 
{
    // Window flags: fixed, no collapse, no resize, no title bar
    ImGuiWindowFlags panelFlags = ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize;



    // Simulation Canvas
    ImGui::SetNextWindowPos(ImVec2(300, 0));
    ImGui::SetNextWindowSize(ImVec2(980, 720));
    ImGui::Begin("Simulation Canvas", nullptr, panelFlags);
    ImGui::Text("3D render goes here");
    ImGui::End();
}