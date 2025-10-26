#include <iostream>
#include <glad/glad.h>
#include <GLFW/glfw3.h>

const char* vertexShaderSource = "#version 330 core\n"
"layout (location = 0) in vec3 aPos;\n"
"void main()\n"
"{\n"
"   gl_Position = vec4(aPos.x, aPos.y, aPos.z, 1.0);\n"
"}\0";

const char* fragmentShaderSource = "#version 330 core\n"
"out vec4 FragColor;\n"
"void main()\n"
"{\n"
"   FragColor = vec4(1.0f, 0.5f, 0.2f, 1.0f);\n"
"}\n\0";

int main() {
	// Initialize GLFW
	if (!glfwInit()) {
		std::cerr << "Failed to initialize GLFW" << std::endl;
		return -1;
	}

	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);	// Set OpenGL version to 3.4
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);	// Set OpenGL version to 3.4
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);	// Use core profile

	GLfloat vertices[] = {	// Define vertices for a triangle
		-0.5f, -0.5f * float(sqrt(3)) / 3, 0.0f,		// Bottom left
		 0.5f, -0.5f * float(sqrt(3)) / 3, 0.0f,		// Bottom right
		 0.0f,  0.5f * float(sqrt(3)) * 2 / 3, 0.0f		// Top
	};

	GLFWwindow* window = glfwCreateWindow(1920, 1080, "Simulator", nullptr, nullptr);	// Create window (windowed mode) and its OpenGL context

	if (!window) {
		std::cerr << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
		return -1;
	}

	glfwMakeContextCurrent(window);	// Make the window's context current
	gladLoadGL();					// Load OpenGL functions

	glViewport(0, 0, 1920, 1080);			// Set the viewport size

	GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);			// Create vertex shader
	glShaderSource(vertexShader, 1, &vertexShaderSource, nullptr);	// Attach shader source code
	glCompileShader(vertexShader);									// Compile vertex shader

	GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);			// Create fragment shader
	glShaderSource(fragmentShader, 1, &fragmentShaderSource, nullptr);	// Attach shader source code
	glCompileShader(fragmentShader);									// Compile fragment shader

	GLuint shaderProgram = glCreateProgram();		// Create shader program
	glAttachShader(shaderProgram, vertexShader);	// Attach vertex shader
	glAttachShader(shaderProgram, fragmentShader);	// Attach fragment shader

	glLinkProgram(shaderProgram);				// Link shader program

	// Delete shaders as they are linked into the program now and no longer necessary
	glDeleteShader(vertexShader);	// Delete vertex shader
	glDeleteShader(fragmentShader);	// Delete fragment shader

	GLuint VAO, VBO; // Vertex Buffer Object

	glGenVertexArrays(1, &VAO);	// Generate VAO
	glGenBuffers(1, &VBO);		// Generate VBO

	glBindVertexArray(VAO);		// Bind VAO

	glBindBuffer(GL_ARRAY_BUFFER, VBO);	// Bind VBO
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);		// Copy vertex data to VBO

	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);	// Define vertex attribute layout
	glEnableVertexAttribArray(0);	// Enable vertex attribute

	// Unbind VBO and VAO (Order is important)
	glBindBuffer(GL_ARRAY_BUFFER, 0);	// Unbind VBO
	glBindVertexArray(0);				// Unbind VAO


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
		
		glClearColor(0.1f, 0.1f, 0.1f, 1.0f);	// Set clear color
		glClear(GL_COLOR_BUFFER_BIT);			// Clear the color buffer

		glUseProgram(shaderProgram);		// Use the shader program
		glBindVertexArray(VAO);				// Bind the VAO
		glDrawArrays(GL_TRIANGLES, 0, 3);	// Draw the triangle

		glfwSwapBuffers(window);	// Swap front and back buffers
		glfwPollEvents();			// Poll for and process events
	}

	glDeleteVertexArrays(1, &VAO);	// Delete VAO
	glDeleteBuffers(1, &VBO);		// Delete VBO
	glDeleteProgram(shaderProgram);	// Delete shader program

	// Clean up and exit
	glfwDestroyWindow(window);
	glfwTerminate();
	return 0;
}