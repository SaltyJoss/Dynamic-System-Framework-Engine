#ifndef SIMULATION_PANELS_H
#define SIMULATION_PANELS_H

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

class SimulationPanels 
{
public:
	void Render();

private:
	void MainPanel();
	void SecondaryPanels();
};

#endif 