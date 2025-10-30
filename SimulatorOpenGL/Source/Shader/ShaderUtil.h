#pragma once

namespace shaders {
	class Shader {
	private:
		unsigned int mProgramId;
		unsigned int getCompiledShader(unsigned int shader_type, const std::string& shader_source);

	public:
		Shader() = default;

		bool load(const std::string& vertexshaderFile, const std::string& fragmentshaderFile);
		void use();
		void unload();

		unsigned int get_program_id() { return mProgramId; }

		void setMat4(const glm::mat4& mat4, const std::string& name);
		void setI1(int v, const std::string& name);
		void setF1(float v, const std::string& name);
		void setF3(float a, float b, float c, const std::string& name);
		void setVec3(const glm::vec3& vec3, const std::string& name);
		void setVec4(const glm::vec4& vec4, const std::string& name);
	};
}