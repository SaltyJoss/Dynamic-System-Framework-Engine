#pragma once

// =============================================
//            File: ShaderUtil.h
// =============================================
// Class for loading, compiling, and managing OpenGL shaders.
//
// Summary:
// =============================================
// 
// public:
// ---------------------------------------------
// Shader()
//      -> Default constructor for the Shader class.
// bool load(const std::string& vertexShaderFile, const std::string& fragmentShaderFile)
//      -> Loads and compiles the vertex and fragment shaders from the specified files.
// bool reload()
//      -> Reloads and recompiles the shaders from the previously specified files.
// void use()
//      -> Activates the shader program for rendering.
// void unload()
//      -> Unloads the shader program and frees resources.
// unsigned int getProgramID()
//      -> Returns the OpenGL program ID of the shader.
// void setMat4(const glm::mat4& mat4, const std::string& name)
//      -> Sets a 4x4 matrix uniform in the shader.
// void setInt1(int a, const std::string& name)
//      -> Sets an integer uniform in the shader.
// void setFlt1(float a, const std::string& name)
//      -> Sets a float uniform in the shader.
// void setFlt2(float a, float b, const std::string& name)
//      -> Sets a vec2 uniform in the shader.
// void setFlt3(float a, float b, float c, const std::string& name)
//      -> Sets a vec3 uniform in the shader.
// void setVec3(const glm::vec3& vec3, const std::string& name)
//      -> Sets a vec3 uniform in the shader.
// void setVec4(const glm::vec4& vec4, const std::string& name)
//      -> Sets a vec4 uniform in the shader.
// void setBool(bool value, const std::string& name)
//      -> Sets a boolean uniform in the shader.
// --------------------------------------------
//
// private:
// --------------------------------------------
// unsigned int _programID
//      -> OpenGL program ID for the shader.
// unsigned int getCompiledShader(unsigned int shaderType, const std::string& shaderSource)
//      -> Compiles a shader of the specified type from the given source code.
// std::string _vertexFile
//      -> File path of the vertex shader source.
// std::string _fragmentFile
//      -> File path of the fragment shader source.
// --------------------------------------------
//
// ============================================
//			  GitHub: SaltyJoss
// ============================================

#include "EngineCore.h"
#include "Platform/Logger.h"

extern ENGINE_API Debug gLog;

namespace shaders {
	class ENGINE_API Shader {
	public:
		Shader() = default;

		bool load(const std::string& vertexShaderFile, const std::string& fragmentShaderFile);
		bool reload();

		void use();
		void unload();

		unsigned int getProgramID() { return _programID; }

		void setMat4(const glm::mat4& mat4, const std::string& name);
		void setInt1(int a, const std::string& name);
		void setFlt1(float a, const std::string& name);
		void setFlt2(float a, float b, const std::string& name);
		void setFlt3(float a, float b, float c, const std::string& name);
		void setVec2(const glm::vec2& vec2, const std::string& name);
		void setVec3(const glm::vec3& vec3, const std::string& name);
		void setVec4(const glm::vec4& vec4, const std::string& name);
		void setBool(bool value, const std::string& name);

	private:
		unsigned int _programID = 0;
		unsigned int getCompiledShader(unsigned int shaderType, const std::string& shaderSource);

		std::string _vertexFile;
		std::string _fragmentFile;
	};
}