#pragma once

// =============================================
//            File: Texture.h
// =============================================
// Class for loading and managing 2D textures.
//
// Summary:
// =============================================
//
// public:
// --------------------------------------------
// GLuint ID
//      -> OpenGL texture ID for the texture.
// Texture()
//      -> Default constructor for the Texture class.
// ~Texture()
//      -> Destructor that cleans up the texture.
// void bind(GLuint unit = 0) const
//      -> Binds the texture to the specified texture unit.
// static GLuint load(const std::string& path, bool sRGB = true)
//      -> Static method to load a texture from the specified file path, with an option for sRGB format.
// --------------------------------------------
//
// ============================================
//			  GitHub: SaltyJoss
// ============================================

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