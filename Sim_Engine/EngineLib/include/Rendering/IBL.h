#pragma once
#include "EngineCore.h"

#include "Platform/Logger.h"

extern ENGINE_API Debug gLog;

using GLuint = std::uint32_t;

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