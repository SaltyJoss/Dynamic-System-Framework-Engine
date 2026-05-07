// DSFE_GUI Texture.h
#pragma once

#include <glad/glad.h>

#include <string>
#include "Platform/Logger.h"

namespace render {
	class Texture {
	public:
		GLuint ID = 0;
		
		Texture() = default;
		~Texture();

		void bind(GLuint unit = 0) const;
		static GLuint load(const std::string& path, bool sRGB = true);
	};
} // namespace render