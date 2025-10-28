#ifndef APPLICATION_H
#define APPLICATION_H

#include <memory>
#include <GLFW/glfw3.h>
#include "GUIManager.h"

class Application
{
public:
	Application() = default;
	~Application() = default;

	bool Initialise();
	void Run();
	void Shutdown();

private:
	GLFWwindow* window = nullptr;
	std::unique_ptr<GUIManager> gui; // persistent
};

#endif