#pragma once
// File:   IBL.h
// GitHub: SaltyJoss
#include "EngineCore.h"
#include "Platform/Logger.h"

namespace render {
	class DSFE_API IBL {
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
} // namespace render