#pragma once

// =============================================
//            File: IBL.h
// =============================================
// Class for Image-Based Lighting (IBL) setup and management.
//
// Summary:
// =============================================
// 
// public:
// --------------------------------------------
// IBL()
//      -> Constructor that initializes the IBL system.
// ~IBL()
//      -> Destructor that cleans up IBL resources.
// void init(const std::string& hdrPath)
//      -> Initializes the IBL system with the specified HDR environment map.
// GLuint getEnvCubemap() const
//      -> Returns the OpenGL texture ID of the environment cubemap.
// GLuint getIrradianceMap() const
//      -> Returns the OpenGL texture ID of the irradiance map.
// GLuint getPrefilterMap() const
//      -> Returns the OpenGL texture ID of the prefilter map.
// GLuint getBRDFLUT() const
//      -> Returns the OpenGL texture ID of the BRDF lookup texture.
// --------------------------------------------
//
// private:
// --------------------------------------------
// GLuint _hdrTexture
//      -> OpenGL texture ID for the loaded HDR texture.
// GLuint _envCubemap
//      -> OpenGL texture ID for the environment cubemap.
// GLuint _irradianceMap
//      -> OpenGL texture ID for the irradiance map.
// GLuint _prefilterMap
//      -> OpenGL texture ID for the prefilter map.
// GLuint _brdfLUT
//      -> OpenGL texture ID for the BRDF lookup texture.
// void loadHDR(const std::string& hdrPath)
//      -> Loads the HDR texture from the specified file path.
// void generateCubemap()
//      -> Generates the environment cubemap from the HDR texture.
// void generateIrradianceMap()
//      -> Generates the irradiance map from the environment cubemap.
// void generatePrefilterMap()
//      -> Generates the prefilter map from the environment cubemap.
// void generateBRDFLUT()
//      -> Generates the BRDF lookup texture.
// --------------------------------------------
//
// ============================================
//			  GitHub: SaltyJoss
// ============================================

#include "EngineCore.h"
#include "Platform/Logger.h"

extern ENGINE_API Debug gLog;

namespace render {
	class ENGINE_API IBL {
	public:
		IBL();
		~IBL();
		
		void init(const std::string& hdrPath);

		GLuint getEnvCubemap() const { return _envCubemap; }

		GLuint getIrradianceMap() const;
		GLuint getPrefilterMap() const;
		GLuint getBRDFLUT() const;
		
	private:
		GLuint _hdrTexture = 0;
		GLuint _envCubemap = 0;
		GLuint _irradianceMap = 0;
		GLuint _prefilterMap = 0;
		GLuint _brdfLUT = 0;

		void loadHDR(const std::string& hdrPath);
		void generateCubemap();
		void generateIrradianceMap();
		void generatePrefilterMap();
		void generateBRDFLUT();
	};
}