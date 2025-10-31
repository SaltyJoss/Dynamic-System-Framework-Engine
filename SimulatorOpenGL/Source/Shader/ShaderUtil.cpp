#include "ch.h"
#include "ShaderUtil.h"

namespace shaders {
	unsigned int Shader::getCompiledShader(unsigned int shaderType, const std::string& shaderSource) {
		unsigned int shaderID = glCreateShader(shaderType);
		const char* cSource = shaderSource.c_str();

		glShaderSource(shaderID, 1, &cSource, nullptr);
		glCompileShader(shaderID);

		GLint result;
		glGetShaderiv(shaderID, GL_COMPILE_STATUS, &result);

		if (result == GL_FALSE) {
			int length;
			glGetShaderiv(shaderID, GL_INFO_LOG_LENGTH, &length);

			GLchar* infoLog = new GLchar[length + 1];
			glGetShaderInfoLog(shaderID, length, &length, infoLog);

			fprintf(stderr, "[ERROR / OpenGL] ShaderCompile: %s\n", infoLog);
			delete[] infoLog;
		}

		return shaderID;
	}

	bool Shader::load(const std::string& vertexShaderFile, const std::string& fragmentShaderFile) {
		std::ifstream isVS(vertexShaderFile);
		std::ifstream isFS(fragmentShaderFile);

		const std::string fVS((std::istreambuf_iterator<char>(isVS)), std::istreambuf_iterator<char>());
		const std::string fFS((std::istreambuf_iterator<char>(isFS)), std::istreambuf_iterator<char>());

		_programID = glCreateProgram();

		unsigned int vs = getCompiledShader(GL_VERTEX_SHADER, fVS);
		unsigned int fs = getCompiledShader(GL_FRAGMENT_SHADER, fFS);

		glAttachShader(_programID, vs);
		glAttachShader(_programID, fs);

		glLinkProgram(_programID);
		glValidateProgram(_programID);

		glDeleteShader(vs);
		glDeleteShader(fs);

		return true;
	}

	void Shader::use() { glUseProgram(_programID); }
	void Shader::unload() { glDeleteProgram(_programID); }

	void Shader::setMat4(const glm::mat4& mat4, const std::string& name) {
		GLint matLoc = glGetUniformLocation(getProgramID(), name.c_str());
		glUniformMatrix4fv(matLoc, 1, GL_FALSE, glm::value_ptr(mat4));
	}
	void Shader::setInt1(int v, const std::string& name) {
		GLint matLoc = glGetUniformLocation(getProgramID(), name.c_str());
		glUniform1i(matLoc, v);
	}
	void Shader::setFlt1(float v, const std::string& name) {
		GLint matLoc = glGetUniformLocation(getProgramID(), name.c_str());
		glUniform1f(matLoc, v);
	}
	void Shader::setFlt3(float a, float b, float c, const std::string& name) {
		GLint matLoc = glGetUniformLocation(getProgramID(), name.c_str());
		glUniform3f(matLoc, a, b, c);
	}
	void Shader::setVec3(const glm::vec3& vec3, const std::string& name) {
		GLint matLoc = glGetUniformLocation(getProgramID(), name.c_str());
		glProgramUniform3fv(getProgramID(), matLoc, 1, glm::value_ptr(vec3));
	}
	void Shader::setVec4(const glm::vec4& vec4, const std::string& name) {
		GLint matLoc = glGetUniformLocation(getProgramID(), name.c_str());
		glProgramUniform4fv(getProgramID(), matLoc, 1, glm::value_ptr(vec4));
	}
}