// DSFE_GUI ShaderUtil.cpp
#include <glad/glad.h>
#include "Rendering/ShaderUtil.h"

#include "EngineLib/LogMacros.h"

namespace shaders {
	// Helper function to compile a shader and check for errors
	unsigned int Shader::getCompiledShader(unsigned int shaderType, const std::string& shaderSource) {
		unsigned int shaderID = glCreateShader(shaderType);
		const char* cSource = shaderSource.c_str();

		// Compile shader
		glShaderSource(shaderID, 1, &cSource, nullptr);
		glCompileShader(shaderID);

		GLint result;
		glGetShaderiv(shaderID, GL_COMPILE_STATUS, &result);

		// Check for compilation errors
		if (result == GL_FALSE) {
			int length;
			glGetShaderiv(shaderID, GL_INFO_LOG_LENGTH, &length);

			GLchar* infoLog = new GLchar[length + 1];
			glGetShaderInfoLog(shaderID, length, &length, infoLog);

			LOG_ERROR("Shader compilation failed: %s", infoLog);
			delete[] infoLog;
		} else { /*LOG_INFO("Shader compiled successfully");*/ }

		return shaderID;
	}

	// Load and compile vertex and fragment shaders, then link them into a shader program
	bool Shader::load(const std::string& vertexShaderFile, const std::string& fragmentShaderFile) {
		_vertexFile = vertexShaderFile;
		_fragmentFile = fragmentShaderFile;

		// Open shader source files
		std::ifstream isVS(vertexShaderFile);
		std::ifstream isFS(fragmentShaderFile);

		// Check if files were opened successfully
		if (!isVS.is_open() || !isFS.is_open()) {
			LOG_ERROR("Failed to open shader files: VS=%s FS=%s", vertexShaderFile.c_str(), fragmentShaderFile.c_str());
			return false;
		}

		// Read shader source code into strings
		const std::string fVS((std::istreambuf_iterator<char>(isVS)), std::istreambuf_iterator<char>());
		const std::string fFS((std::istreambuf_iterator<char>(isFS)), std::istreambuf_iterator<char>());

		_programID = glCreateProgram();

		// Compile shaders
		unsigned int vs = getCompiledShader(GL_VERTEX_SHADER, fVS);
		unsigned int fs = getCompiledShader(GL_FRAGMENT_SHADER, fFS);

		// Attach shaders to the program and link
		glAttachShader(_programID, vs);
		glAttachShader(_programID, fs);

		glLinkProgram(_programID);

		// Check for linking errors
		GLint linkStatus;
		glGetProgramiv(_programID, GL_LINK_STATUS, &linkStatus);
		if (linkStatus == GL_FALSE) {
			int length;
			glGetProgramiv(_programID, GL_INFO_LOG_LENGTH, &length);
			GLchar* infoLog = new GLchar[length + 1];
			glGetProgramInfoLog(_programID, length, &length, infoLog);
			LOG_ERROR("Shader program linking failed: %s", infoLog);
			delete[] infoLog;
		}
		else { /*LOG_INFO("Shader program linked successfully");*/ }

		// Validate the shader program
		glValidateProgram(_programID);

		glDeleteShader(vs);
		glDeleteShader(fs);

		return true;
	}

	// Reload the shader program by re-reading the source files and recompiling/linking
	bool Shader::reload() {
		if (_vertexFile.empty() || _fragmentFile.empty()) {
			LOG_ERROR("Shader reload failed: no shader source paths stored.");
			return false;
		}
		glDeleteProgram(_programID);
		return load(_vertexFile, _fragmentFile);
	}

	// Use the shader program for rendering
 	void Shader::use() { glUseProgram(_programID); LOG_INFO_ONCE("Shader program bound"); }
	// Unload the shader program and free resources
	void Shader::unload() { glDeleteProgram(_programID); LOG_INFO_ONCE("Shader program deleted"); }

	// Set a 4x4 matrix uniform in the shader program
	void Shader::setMat4(const glm::mat4& mat4, const std::string& name) {
		GLint matLoc = glGetUniformLocation(getProgramID(), name.c_str());
		glUniformMatrix4fv(matLoc, 1, GL_FALSE, glm::value_ptr(mat4));
	}
	// Set an integer uniform in the shader program
	void Shader::setInt1(int a, const std::string& name) {
		GLint matLoc = glGetUniformLocation(getProgramID(), name.c_str());
		glUniform1i(matLoc, a);
	}
	// Set a float uniform in the shader program
	void Shader::setFlt1(float a, const std::string& name) {
		GLint matLoc = glGetUniformLocation(getProgramID(), name.c_str());
		glUniform1f(matLoc, a);
	}
	// Set a vec2 uniform in the shader program
	void Shader::setFlt2(float a, float b, const std::string& name) {
		GLint matLoc = glGetUniformLocation(getProgramID(), name.c_str());
		glUniform2f(matLoc, a, b);
	}
	// Set a vec3 uniform in the shader program
	void Shader::setFlt3(float a, float b, float c, const std::string& name) {
		GLint matLoc = glGetUniformLocation(getProgramID(), name.c_str());
		glUniform3f(matLoc, a, b, c);
	}
	void Shader::setFltArray2(float a, float b, const std::string& name) {
		GLint loc = glGetUniformLocation(getProgramID(), name.c_str());
		float vals[2] = { a, b };
		glUniform1fv(loc, 2, vals);
	}
	// Set a vec4 uniform in the shader program
	void Shader::setVec2(const glm::vec2& vec2, const std::string& name) {
		GLint matLoc = glGetUniformLocation(getProgramID(), name.c_str());
		glProgramUniform2fv(getProgramID(), matLoc, 1, glm::value_ptr(vec2));
	}
	// Set a vec3 uniform in the shader program
	void Shader::setVec3(const glm::vec3& vec3, const std::string& name) {
		GLint matLoc = glGetUniformLocation(getProgramID(), name.c_str());
		glProgramUniform3fv(getProgramID(), matLoc, 1, glm::value_ptr(vec3));
	}
	// Set a vec4 uniform in the shader program
	void Shader::setVec4(const glm::vec4& vec4, const std::string& name) {
		GLint matLoc = glGetUniformLocation(getProgramID(), name.c_str());
		glProgramUniform4fv(getProgramID(), matLoc, 1, glm::value_ptr(vec4));
	}
	// Set a boolean uniform in the shader program (as an integer)
	void Shader::setBool(bool value, const std::string& name) {
		GLint matLoc = glGetUniformLocation(getProgramID(), name.c_str());
		glUniform1i(matLoc, value ? 1 : 0);  // use glUniform1i for bool
	}
}