#ifndef APPLICATION_H
#define APPLICATION_H

#include "CoreIncludes.h"
#include "GUIManager.h"
#include "WindowManager.h"
#include "SimulationManager.h"

class Application
{
public:
	bool Initialise();
	void Run();
	void Shutdown();

private:
	std::unique_ptr<GUIManager> gui;
	WindowManager windowManager;
	GLFWwindow* window = nullptr;
	std::unique_ptr<SimulationManager> simulation;
};

#endif