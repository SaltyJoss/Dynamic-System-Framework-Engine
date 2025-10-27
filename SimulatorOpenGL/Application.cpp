#include "Application.h"
#include <glad/glad.h>
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <iostream>

// Resize callback
void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
	glViewport(0, 0, width, height);
}

bool Application::Initialise()
{
	if (!glfwInit()) { std::cerr << "Failed to initialize GLFW" << std::endl; return false; }

	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);
	glfwWindowHint(GLFW_DEPTH_BITS, 24);

	window = glfwCreateWindow(1920, 1080, "Simulator V0.1", nullptr, nullptr);	// Create window (windowed mode) and its OpenGL context
	if (!window) { std::cerr << "Failed to create GLFW window" << std::endl; return false; }

	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);
	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) { std::cerr << "Failed to initialize GLAD" << std::endl; return false; }

	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO();
	(void)io;

	ImGui::StyleColorsDark();
	ImGui_ImplGlfw_InitForOpenGL(window, true);
	ImGui_ImplOpenGL3_Init("#version 330");

	return true;
}

void Application::Run()
{
	while (!glfwWindowShouldClose(window)) {
		glfwPollEvents();

		int display_w, display_h;
		glfwGetFramebufferSize(window, &display_w, &display_h);
		glViewport(0, 0, display_w, display_h);
		glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		gui.BeginFrame();
		gui.DrawPanel();
		gui.EndFrame();

		glfwSwapBuffers(window);	// Swap front and back buffers
	}
}

void Application::Shutdown()
{
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();
	glfwDestroyWindow(window);
	glfwTerminate();

}