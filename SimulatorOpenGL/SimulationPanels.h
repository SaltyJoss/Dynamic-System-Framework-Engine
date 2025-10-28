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
	void Container(ImGuiWindowFlags panelFlags);
	void MainPanel(ImGuiWindowFlags panelFlags);
	void Panel1(ImGuiWindowFlags panelFlags);
	void Panel2(ImGuiWindowFlags panelFlags);
	void Panel3(ImGuiWindowFlags panelFlags);
	void Panel4(ImGuiWindowFlags panelFlags);
};


#endif 