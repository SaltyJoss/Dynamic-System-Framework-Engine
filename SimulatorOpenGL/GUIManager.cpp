#include <GLFW/glfw3.h>
#include "ControlPanel.h"
#include "SimulationPanels.h"
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
void GUIManager::DrawPanel() {
	ImGuiWindowFlags panelFlags = ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize;
	ImGui::SetNextWindowPos(ImVec2(0, 0));
	ImGui::SetNextWindowSize(ImVec2(ImGui::GetIO().DisplaySize.x, ImGui::GetIO().DisplaySize.y));
	ImGui::Begin("Robotic-Arm Simualtor V0.0", nullptr, panelFlags);

	ContainerPanel();

	ImGui::End();
}

void GUIManager::ContainerPanel() {
	ControlPanel ctrlPanel;
	SimulationPanels simPanel;

	ctrlPanel.Render();
	ImGui::SameLine();
	simPanel.Render();
	

}

