#ifndef APPLICATION_H
#define APPLICATION_H

#include <GLFW/glfw3.h>
#include "GUIManager.h"

class Application
{
public:
	bool Initialise();
	void Run();
	void Shutdown();

private:
	GLFWwindow* window = nullptr;
	GUIManager gui;
};

#endif