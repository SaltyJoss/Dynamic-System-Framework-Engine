#ifndef GUI_MANAGER_H
#define GUI_MANAGER_H

#include <imgui.h>

class GUIManager
{
public:
	GUIManager(GLFWwindow* win) : window(win) {}

	void BeginFrame();
	void EndFrame();
	void DrawPanel();

private:
	void ContainerPanel();
	GLFWwindow* window;
};

#endif