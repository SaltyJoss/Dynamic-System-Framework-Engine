#ifndef GUI_MANAGER_H
#define GUI_MANAGER_H

#include <imgui.h>

class GUIManager
{
public:
	void BeginFrame();
	void EndFrame();
	void DrawPanel();
};

#endif