/*#ifndef GUI_MANAGER_H
#define GUI_MANAGER_H

#include "ch.h"
#include "window.h"

#include "UIx/ControlPanel.h"
#include "UIx/SimulationPanels.h"
#include "UIx/DebugPanel.h"
#include "UIx/Styles.h"



#include "Rendering/SimulationManager.h"

using namespace window;

class GUIManager
{
public:
	explicit GUIManager(GLWindow* _window);

	void InitResources();
	void BeginFrame();
	void EndFrame();
	void DrawPanel();

private:
	ImVec2 winSize;
	ImVec2 padding;
	float debugHeight = 0.0f;
	float ctrlPanelWidth = 0.0f;

	void ContainerPanel();
	window::GLWindow* _window;

	ControlPanel ctrlPanel;
	SimulationPanels simPanel;
	DebugPanel debug;
	StyleModes styles;
	std::unique_ptr<SimulationManager> simulation;
};

#endif*/