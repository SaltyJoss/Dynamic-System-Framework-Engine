#pragma once
// File:   AxisOrientator.h
// GitHub: SaltyJoss
#include "EngineCore.h"
#include <glad/glad.h>

namespace gui {
	class ENGINE_API AxisOrientator
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