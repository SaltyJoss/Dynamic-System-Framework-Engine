#include <iostream>
#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "shaderClass.h"
#include "VAO.h"
#include "VBO.h"
#include "EBO.h"

// Vertices of a triangle
GLfloat vertices[] = 
{ //				COORDINATES					  /		COLOURS			//
	-0.5f, -0.5f * float(sqrt(3)) / 3,		0.0f,	0.8f, 0.3f,  0.02f,	// Bottom left Corner {0}
	 0.5f, -0.5f * float(sqrt(3)) / 3,		0.0f,	0.8f, 0.3f,  0.02f,	// Bottom right Corner {1}
	 0.0f,  0.5f * float(sqrt(3)) * 2 / 3,	0.0f,	1.0f, 0.6f,  0.32f,	// Top Corner {2}
	 -0.5f / 2, 0.5f * float(sqrt(3)) / 6,	0.0f,	0.9f, 0.45f, 0.17f,	// Inner left {3}
	 0.5f / 2, 0.5f * float(sqrt(3)) / 6,	0.0f,	0.9f, 0.45f, 0.17f,	// Inner right {4}
	 0.0f, -0.5f * float(sqrt(3)) / 3,		0.0f,	0.8f, 0.3f,  0.02f	// Inner bottom {5}

};

GLuint indices[] =
{
	5, 3, 0,	// Lower left triangle	{5,3,0}
	4, 2, 3,	// Upper triangle		{4,2,3}
	1, 4, 5		// Lower right triangle	{1,4,5}
};

int main() {
	// Initialize GLFW
	if (!glfwInit()) {
		std::cerr << "Failed to initialize GLFW" << std::endl;
		return -1;
	}

	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);	// Set OpenGL version to 3.4
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);	// Set OpenGL version to 3.4
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);	// Use core profile

	GLFWwindow* window = glfwCreateWindow(1080, 1080, "Simulator V0.1", nullptr, nullptr);	// Create window (windowed mode) and its OpenGL context

	if (!window) {
		std::cerr << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
		return -1;
	}

	glfwMakeContextCurrent(window);	// Make the window's context current
	gladLoadGL();					// Load OpenGL functions

	glViewport(0, 0, 1080, 1080);			// Set the viewport size

	Shader shaderProgram("default.vert", "default.frag"); // Create shader program from shaders	
	
	VAO VAO1;		// Create Vertex Array Object
	VAO1.Bind();	// Bind the VAO

	VBO VBO1(vertices, sizeof(vertices));	// Create Vertex Buffer Object and link it to vertices
	EBO EBO1(indices, sizeof(indices));		// Create Element Buffer Object and link it to indices

	VAO1.LinkAttrib(VBO1, 0, 3, GL_FLOAT, 6 * sizeof(float), (void*)0);	// Links VAO to VBO and the attributes
	VAO1.LinkAttrib(VBO1, 1, 3, GL_FLOAT, 6 * sizeof(float), (void*)(3 * sizeof(float)));

	VAO1.Unbind();			// Unbind the VAO
	VBO1.Unbind();			// Unbind the VBO
	EBO1.Unbind();			// Unbind the EBO

	GLuint uniID = glGetUniformLocation(shaderProgram.ID, "scale"); 


	glClearColor(0.075f, 0.125f, 0.15f, 1.0f);	// Set clear color
	glClear(GL_COLOR_BUFFER_BIT);			// Clear the color buffer
	glfwSwapBuffers(window);				// Swap front and back buffers

	// Load OpenGL functions using GLAD
	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
		std::cerr << "Failed to initialize GLAD" << std::endl;
		return -1;
	}

	// Main loop
	while (!glfwWindowShouldClose(window)) {
		
		glClearColor(0.075f, 0.125f, 0.15f, 1.0f);	// Set clear color
		glClear(GL_COLOR_BUFFER_BIT);			// Clear the color buffer

		shaderProgram.Activate();		// Activate the shader program
		glUniform1f(uniID, 0.4f);		// Scale of triangles 
		VAO1.Bind();					// Bind the VAO

		glDrawElements(GL_TRIANGLES, 9, GL_UNSIGNED_INT, 0); // Draw the triangle using the EBO

		glfwSwapBuffers(window);	// Swap front and back buffers
		glfwPollEvents();			// Poll for and process events
	}

	// Delete allocated resources
	VAO1.Delete();
	VBO1.Delete();
	EBO1.Delete();
	shaderProgram.Delete();

	// Clean up and exit
	glfwDestroyWindow(window);
	glfwTerminate();
	return 0;
}