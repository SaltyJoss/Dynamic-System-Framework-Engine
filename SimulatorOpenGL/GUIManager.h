#ifndef GUI_MANAGER_H
#define GUI_MANAGER_H

#include "CoreIncludes.h"
#include "ControlPanel.h"
#include "SimulationPanels.h"
#include "DebugPanel.h"
#include "WindowManager.h"
#include "ResourceManager.h"
//#include "TitleBarPanel.h"

class GUIManager
{
public:
	explicit GUIManager(WindowManager* windowManager);

	void InitResources();
	void BeginFrame();
	void EndFrame();
	void DrawPanel();

private:
	void ContainerPanel();
	WindowManager* windowManager;

	//std::unique_ptr<TitleBarPanel> titleBar;
	ControlPanel ctrlPanel;
	SimulationPanels simPanel;
	DebugPanel debug;
};

#endif