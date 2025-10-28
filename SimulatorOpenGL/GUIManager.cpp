#include <GLFW/glfw3.h>
#include "ControlPanel.h"
#include "SimulationPanels.h"
#include "DebugPanel.h"
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
	ImGuiWindowFlags panelFlags  =	ImGuiWindowFlags_NoTitleBar |
									ImGuiWindowFlags_NoResize	|
									ImGuiWindowFlags_NoCollapse;
	ImGui::SetNextWindowPos(ImVec2(0, 0));
	ImGui::SetNextWindowSize(ImVec2(ImGui::GetIO().DisplaySize.x, ImGui::GetIO().DisplaySize.y));
	ImGui::Begin("Robotic-Arm Simualtor V0.0", nullptr, panelFlags);

	ImGui::BeginChild("TitleBar", ImVec2(ImGui::GetWindowWidth(), 30), false);
	ImGui::Text("Simulator");

	// Right-aligned buttons
	ImGui::SameLine(ImGui::GetWindowWidth() - 90);
	if (ImGui::Button("_")) { glfwIconifyWindow(window); }  // Minimize
	ImGui::SameLine();
	if (ImGui::Button("[ ]")) {  // Toggle fullscreen
		static bool fullscreen = false;
		fullscreen = !fullscreen;
		if (fullscreen) {
			const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
			glfwSetWindowMonitor(window, glfwGetPrimaryMonitor(), 0, 0, mode->width, mode->height, mode->refreshRate);
		}
		else {
			glfwSetWindowMonitor(window, nullptr, 100, 100, 1280, 720, 0);
		}
	}
	ImGui::SameLine();
	if (ImGui::Button("X")) { glfwSetWindowShouldClose(window, GLFW_TRUE); } // Close

	// Optional: make top bar draggable
	ImGui::InvisibleButton("##drag", ImVec2(ImGui::GetWindowWidth(), 30));
	if (ImGui::IsItemActive()) {
		ImVec2 delta = ImGui::GetIO().MouseDelta;
		ImVec2 pos = ImGui::GetWindowPos();
		ImGui::SetWindowPos(ImVec2(pos.x + delta.x, pos.y + delta.y));
	}

	ImGui::EndChild();


	ContainerPanel();

	ImGui::End();
}

void GUIManager::ContainerPanel() {
	ControlPanel ctrlPanel;
	SimulationPanels simPanel;
	DebugPanel debug;

	ctrlPanel.Render();
	ImGui::SameLine();
	simPanel.Render();
	debug.Render();
	

}

