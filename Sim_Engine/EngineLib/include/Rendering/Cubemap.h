#pragma once

// =============================================
//            File: Cubemap.h
// =============================================
// Class for loading and managing a cubemap texture.
//
// Summary:
// =============================================
//
// public:
// --------------------------------------------
// Cubemap(const std::array<std::string, 6>& faces)
//      -> Constructor that loads a cubemap texture from the specified face file paths.
// ~Cubemap()
//      -> Destructor that cleans up the cubemap texture.
// void bind(unsigned int unit = 0) const
//      -> Binds the cubemap texture to the specified texture unit.
// unsigned int ID() const
//      -> Returns the OpenGL texture ID of the cubemap.
// --------------------------------------------
//
// private:
// --------------------------------------------
// unsigned int _cubemapTexture
//      -> OpenGL texture ID for the cubemap.
// bool _isLoaded
//      -> Indicates whether the cubemap texture was successfully loaded.
// --------------------------------------------
//
// ============================================
//			  GitHub: SaltyJoss
// ============================================

#include "EngineCore.h"
#include "RenderBase.h"
#include "Scene/VertexHolder.h"
#include "Scene/Element.h"

#include "Scene/Face.h"
#include "Platform/Logger.h"

extern ENGINE_API Debug gLog;

namespace render {
	class ENGINE_API Cubemap {
	public:
		Cubemap(const std::array<std::string, 6>& faces);
		~Cubemap();

		void bind(unsigned int unit = 0) const;
		unsigned int ID() const;

	private:
		unsigned int _cubemapTexture;
		bool _isLoaded;
	};
}