#pragma once
#include "EngineCore.h"

#include "Platform/Logger.h"

extern ENGINE_API Debug gLog;

using GLuint = std::uint32_t;

namespace render {
	class ENGINE_API IBL {
	public:
		IBL() = default;
		~IBL() = default;
		
		void init(const std::string& hdrPath);

		virtual GLuint getIrradianceMap() const = 0;
		virtual GLuint getPrefilterMap() const = 0;
		virtual GLuint getBRDFLUT() const = 0;
		
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