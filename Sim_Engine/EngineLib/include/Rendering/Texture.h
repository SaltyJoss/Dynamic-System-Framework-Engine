#pragma once
#include "EngineCore.h"
#include <glad/glad.h>

#include <string>
#include "Platform/Logger.h"

extern ENGINE_API Debug gLog;

namespace render {
	class ENGINE_API Texture {
	public:
		GLuint ID = 0;
		
		Texture() = default;
		~Texture();

		void bind(GLuint unit = 0) const;
		static GLuint load(const std::string& path, bool sRGB = true);
	};
}