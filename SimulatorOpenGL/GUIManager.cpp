#include "GUIManager.h"


GUIManager::GUIManager(WindowManager* manager) : windowManager(manager) {}

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
	ImGuiWindowFlags panelFlags  =	
		  ImGuiWindowFlags_NoTitleBar
		| ImGuiWindowFlags_NoResize
		| ImGuiWindowFlags_NoCollapse
		| ImGuiWindowFlags_NoScrollbar
		| ImGuiWindowFlags_NoBringToFrontOnFocus;

	ImGui::SetNextWindowPos(ImVec2(0, 0));
	ImGui::SetNextWindowSize(ImVec2(ImGui::GetIO().DisplaySize.x, ImGui::GetIO().DisplaySize.y));
	ImGui::Begin("Robotic-Arm Simualtor V0.0", nullptr, panelFlags);

	ContainerPanel();

	ImGui::End();
}

void GUIManager::ContainerPanel() {
	titleBar = std::make_unique<TitleBarPanel>(windowManager->GetWindow());

	titleBar->Render(20.0f);

	ctrlPanel.Render();
	ImGui::SameLine();
	simPanel.Render();
	debug.Render();
}

void GUIManager::InitResources() {
	ResourceManager::LoadTexture("close", "assets/close.png");
	ResourceManager::LoadTexture("minimise", "assets/minimise.png");
	ResourceManager::LoadTexture("maximise", "assets/maximise.png");
}

