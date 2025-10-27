#include <iostream>
#include <glad/glad.h>
#include <GLFW/glfw3.h>

int main() {
	// Initialize GLFW
	if (!glfwInit()) {
		std::cerr << "Failed to initialize GLFW" << std::endl;
		return -1;
	}

	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);	// Set OpenGL version to 3.4
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);	// Set OpenGL version to 3.4
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);	// Use core profile

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

	// Vertices of a triangle
	GLfloat vertices[] = {
		-0.5f, -0.5f * float(sqrt(3)) / 3, 0.0f,		// Bottom left Corner {0}
		 0.5f, -0.5f * float(sqrt(3)) / 3, 0.0f,		// Bottom right Corner {1}
		 0.0f,  0.5f * float(sqrt(3)) * 2 / 3, 0.0f,	// Top Corner {2}
		 -0.5f / 2, 0.5f * float(sqrt(3)) / 6, 0.0f,	// Inner left {3}
		 0.5f / 2, 0.5f * float(sqrt(3)) / 6, 0.0f,		// Inner right {4}
		 0.0f, -0.5f * float(sqrt(3)) / 3, 0.0f			// Inner bottom {5}

	};

	GLuint indices[] =
	{
		5, 3, 0,	// Lower left triangle	{5,3,0}
		4, 2, 3,	// Upper triangle		{4,2,3}
		1, 4, 5		// Lower right triangle	{1,4,5}
	};

	GLuint VAO, VBO, EBO; // Create reference containers for the Vartex Array Object, Vertex Buffer Object and Element Buffer Object

	// Generate the VAO, VBO and EBO with only 1 object each
	glGenVertexArrays(1, &VAO);
	glGenBuffers(1, &VBO);
	glGenBuffers(1, &EBO);

	glBindVertexArray(VAO);	// Bind VAO, making it the current Vertex Array Object

	glBindBuffer(GL_ARRAY_BUFFER, VBO);	// Bind VBO, specifying its type is a GL_ARRAY_BUFFER
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW); // Introduce the vertices into the VBO]

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);	// Bind EBO, specifying its type is a GL_ELEMENT_ARRAY_BUFFER
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW); // Introduce the indices into the EBO

	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);	// Define vertex attribute layout
	glEnableVertexAttribArray(0);	// Enable vertex attribute

	// Unbind VBO and VAO (Order is important)
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0); // Note: Do NOT unbind the EBO while a VAO is active as the bound EBO is stored in the VAO


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
		
		glClearColor(0.075f, 0.125f, 0.15f, 1.0f);	// Set clear color
		glClear(GL_COLOR_BUFFER_BIT);			// Clear the color buffer

		glUseProgram(shaderProgram);		// Use the shader program
		glBindVertexArray(VAO);				// Bind the VAO

		glDrawElements(GL_TRIANGLES, 9, GL_UNSIGNED_INT, 0); // Draw the triangle using the EBO

		glfwSwapBuffers(window);	// Swap front and back buffers
		glfwPollEvents();			// Poll for and process events
	}

	// Delete allocated resources
	glDeleteVertexArrays(1, &VAO);	
	glDeleteBuffers(1, &VBO);		
	glDeleteBuffers(1, &EBO);		
	glDeleteProgram(shaderProgram);

	// Clean up and exit
	glfwDestroyWindow(window);
	glfwTerminate();
	return 0;
}