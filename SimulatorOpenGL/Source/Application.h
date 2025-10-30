#ifndef APPLICATION_H
#define APPLICATION_H

#include "ch.h"
#include "Window/GUIManager.h"
#include "Window/WindowManager.h"
#include "Render/SimulationManager.h"

class Application
{
public:
	Application(const std::string& name);

	static Application& Instance() { return *sInstance; }

	bool Init();
	void Run();
	void Shutdown();

private:
	static Application* sInstance;

	std::unique_ptr<GUIManager> gui;
	WindowManager windowManager;
	GLFWwindow* window = nullptr;
	std::unique_ptr<SimulationManager> simulation;
};

#endif