#ifndef APPLICATION_H
#define APPLICATION_H

#include "CoreIncludes.h"
#include "GUIManager.h"
#include "WindowManager.h"

class Application
{
public:
	Application() = default;
	~Application() = default;

	bool Initialise();
	void Run();
	void Shutdown();

private:
	std::unique_ptr<GUIManager> gui;
	WindowManager windowManager;
	GLFWwindow* window = nullptr;
};

#endif