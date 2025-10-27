#include <iostream>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include "imgui.h"
#include "GUIManager.h"
#include "Application.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

// Main method fpr 
int main() {
	Application app;

	if (!app.Initialise())
		return -1;

	app.Run();
	app.Shutdown();
	
	return 0;
}