#ifndef SIMULATION_PANELS_H
#define SIMULATION_PANELS_H

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

class SimulationPanels 
{
public:
	void Render(ImVec2 winSize, ImVec2 padding, float debugHeight, float ctrlPanelWidth);

private:
	void MainPanel();
	void SecondaryPanels();
};

#endif 