#include "shaderClass.h"

// Function to read the contents of a file and return it as a string
std::string get_file_content(const char* filename)
{
	std::ifstream file(filename, std::ios::binary);
	if (file) 
	{
		std::string contents;						// Create a string to hold the file contents
		file.seekg(0, std::ios::end);				// Move the file pointer to the end of the file
		contents.resize(file.tellg());				// Resize the string to fit the file contents
		file.seekg(0, std::ios::beg);				// Move the file pointer back to the beginning of the file
		file.read(&contents[0], contents.size());	// Read the file contents into the string
		file.close();								// Close the file
		return(contents);							// Return the file contents
	}
	throw(errno);
}

// Constructor that builds the shader program from 2 different shaders
Shader::Shader(const char* vertexPath, const char* fragmentPath)
{
	std::string vertexCode = get_file_content(vertexPath);		// Read vertex shader code from file
	std::string fragmentCode = get_file_content(fragmentPath);	// Read fragment shader code from file

	const char* vShaderCode = vertexCode.c_str();	// Convert vertex shader code to C-style string
	const char* fShaderCode = fragmentCode.c_str();	// Convert fragment shader code to C-style string

	GLuint vertex, fragment; // Create shader object references

	vertex = glCreateShader(GL_VERTEX_SHADER);		// Create vertex shader object
	glShaderSource(vertex, 1, &vShaderCode, NULL);	// Attach vertex shader source code
	glCompileShader(vertex);

	fragment = glCreateShader(GL_FRAGMENT_SHADER);		// Create fragment shader object
	glShaderSource(fragment, 1, &fShaderCode, NULL);	// Attach fragment shader source code
	glCompileShader(fragment);

	ID = glCreateProgram();	// Create shader program

	glAttachShader(ID, vertex);		// Attach vertex shader	
	glAttachShader(ID, fragment);	// Attach fragment shader
	glLinkProgram(ID);

	glDeleteShader(vertex);		// Delete vertex shader
	glDeleteShader(fragment);	// Delete fragment shader
}

// Activate the shader program
void Shader::Activate()
{
	glUseProgram(ID);
}

// Delete the shader program
void Shader::Delete()
{
	glDeleteProgram(ID);
}