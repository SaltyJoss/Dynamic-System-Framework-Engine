#include "pch.h"
#include "Scene/RenderPreset.h"

namespace render {
	RenderSettings MakeSettings(ResolutionPreset res, QualityPreset quality) {
		RenderSettings s;
		// Set quality-based s
		switch (quality) {
		case QualityPreset::Low:
			s.shadowMapRes = 1024;
			s.shadowCascades = 1;
			s.pcfKernel = 3;
			s.ssao = false;
			s.bloom = false;
			s.envPreFilterRes = 128;
			s.envIrradianceRes = 16;
			s.msaaSamples = 1;
			s.fxaa = true;
			break;
		case QualityPreset::Medium:
			s.shadowMapRes = 4096;
			s.shadowCascades = 4;
			s.pcfKernel = 5;
			s.ssao = true;
			s.ssaoResDiv = 2;
			s.ssaoSamples = 32;
			s.ssaoStrength = 0.75f;
			s.bloom = true;
			s.bloomThreshold = 0.5f;
			s.envPreFilterRes = 256;
			s.envIrradianceRes = 32;
			s.msaaSamples = 4;
			s.fxaa = false;
			break;
		case QualityPreset::High:
			s.shadowMapRes = 8192;
			s.shadowCascades = 8;
			s.pcfKernel = 5;
			s.ssao = true;
			s.ssaoResDiv = 2;
			s.ssaoSamples = 64;
			s.ssaoStrength = 1.0f;
			s.bloom = true;
			s.bloomThreshold = 0.75f;
			s.envPreFilterRes = 512;
			s.envIrradianceRes = 64;
			s.msaaSamples = 8;
			s.fxaa = false;
			break;
		case QualityPreset::Ultra:
			s.shadowMapRes = 8192;
			s.shadowCascades = 16;
			s.pcfKernel = 9; // 7 gives better softness at high res
			s.ssao = true;
			s.ssaoResDiv = 1;
			s.ssaoSamples = 64;
			s.ssaoStrength = 1.0f;
			s.bloom = true;
			s.bloomThreshold = 1.0f;
			s.envPreFilterRes = 512;
			s.envIrradianceRes = 128;
			s.msaaSamples = 16;
			s.fxaa = false;
			break;
		}

		switch (res) {
			case ResolutionPreset::R_720p: s.renderScale = 1.0f; break;
			case ResolutionPreset::R_1080p: s.renderScale = 1.0f; break;
			case ResolutionPreset::R_1440p: s.renderScale = 1.0f; break;
			case ResolutionPreset::R_4K: s.renderScale = 1.0f; break;
		}

		s.exposure = 1.0f;
		s.whitePoint = 1.675f;
		s.grid = true;
		s.axisOrientator = true;

		return s;
	}


}