#ifndef GUI_MANAGER_H
#define GUI_MANAGER_H

#include "../ch.h"
#include "../UI/ControlPanel.h"
#include "../UI/SimulationPanels.h"
#include "../UI/DebugPanel.h"
#include "../Elements/ResourceManager.h"
#include "../UI/Styles.h"
#include "../Render/SimulationManager.h"
#include "WindowManager.h"

class GUIManager
{
public:
	explicit GUIManager(WindowManager* windowManager);

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
	WindowManager* windowManager;

	ControlPanel ctrlPanel;
	SimulationPanels simPanel;
	DebugPanel debug;
	StyleModes styles;
	std::unique_ptr<SimulationManager> simulation;
};

#endif