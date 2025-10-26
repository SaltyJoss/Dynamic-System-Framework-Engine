#include <iostream>
#include <glad/glad.h>
#include <GLFW/glfw3.h>

int main() {
	// Initialize GLFW
	if (!glfwInit()) {
		std::cerr << "Failed to initialize GLFW" << std::endl;
		return -1;
	}

	GLFWwindow* window = glfwCreateWindow(1920, 1080, "Simulator", nullptr, nullptr);	// Create window (windowed mode) and its OpenGL context

	if (!window) {
		std::cerr << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
		return -1;
	}

	glfwMakeContextCurrent(window);	// Make the window's context current
	gladLoadGL();					// Load OpenGL functions

	glViewport(0, 0, 1920, 1080);			// Set the viewport size
	glClearColor(0.1f, 0.1f, 0.1f, 1.0f);	// Set clear color
	glClear(GL_COLOR_BUFFER_BIT);			// Clear the color buffer
	glfwSwapBuffers(window);				// Swap front and back buffers

	// Load OpenGL functions using GLAD
	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
		std::cerr << "Failed to initialize GLAD" << std::endl;
		return -1;
	}

	// Main loop
	while (!glfwWindowShouldClose(window)) {
		
		glClear(GL_COLOR_BUFFER_BIT);	// Render here
		
		glfwSwapBuffers(window);	// Swap front and back buffers
		glfwPollEvents();			// Poll for and process events
	}

	// Clean up and exit
	glfwDestroyWindow(window);
	glfwTerminate();
	return 0;
}