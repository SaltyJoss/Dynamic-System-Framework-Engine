#pragma once
// File:   ShaderUtil.h
// GitHub: SaltyJoss
#include "EngineCore.h"
#include "Platform/Logger.h"

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
} // namespace shaders