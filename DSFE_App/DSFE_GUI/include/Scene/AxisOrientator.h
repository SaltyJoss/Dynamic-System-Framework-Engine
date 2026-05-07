// DSFE_GUI AxisOrientator.h
#pragma once

#include <glad/glad.h>
#include <glm/glm.hpp>

namespace gui {
	class AxisOrientator
	{
	public:
		void render(const glm::mat4& viewMatrix);

	private:
		void init();

		static GLuint g_VAO;		// lines
		static GLuint g_VBO;		// lines
		static GLuint g_VAOCones;   // cones
		static GLuint g_VBOCones;   // cones
		static GLuint g_Shader;

		static bool g_Initialised;
		static int g_ConeVertexCount;
	};
} // namespace gui