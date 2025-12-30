#include "pch.h"
#include "Scene/RenderPreset.h"

namespace render {
	RenderSettings MakeSettings(LookPreset look, QualityPreset quality) {
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
			s.renderScale = 0.5f;
			break;
		case QualityPreset::Medium:
			s.shadowMapRes = 2048;
			s.shadowCascades = 2;
			s.pcfKernel = 5;
			s.ssao = true;
			s.ssaoResDiv = 3;
			s.ssaoSamples = 16;
			s.ssaoStrength = 0.5f;
			s.bloom = true;
			s.bloomThreshold = 0.5f;
			s.envPreFilterRes = 256;
			s.envIrradianceRes = 32;
			s.msaaSamples = 2;
			s.fxaa = false;
			s.renderScale = 0.75f;
			break;
		case QualityPreset::High:
			s.shadowMapRes = 4096;
			s.shadowCascades = 4;
			s.pcfKernel = 5;
			s.ssao = true;
			s.ssaoResDiv = 2;
			s.ssaoSamples = 32;
			s.ssaoStrength = 0.75f;
			s.bloom = true;
			s.bloomThreshold = 0.8f;
			s.envPreFilterRes = 512;
			s.envIrradianceRes = 64;
			s.msaaSamples = 4;
			s.fxaa = false;
			s.renderScale = 1.0f;
			break;
		case QualityPreset::Ultra:
			s.shadowMapRes = 8192;
			s.shadowCascades = 4;
			s.pcfKernel = 7;
			s.ssao = true;
			s.ssaoResDiv = 1;
			s.ssaoSamples = 64;
			s.ssaoStrength = 1.0f;
			s.bloom = true;
			s.exposure = 1.0f;
			s.bloomThreshold = 1.0f;
			s.envPreFilterRes = 1024;
			s.envIrradianceRes = 128;
			s.msaaSamples = 8;
			s.fxaa = false;
			s.renderScale = 1.0f;
			break;
		}

		// Look Asxis s
		switch (look) {
		case LookPreset::Studio:
			s.exposure = 1.15f;
			s.grid = true;
			s.axisOrientator = true;
			break;
		case LookPreset::Cinematic:
			s.exposure = 0.95f;
			s.grid = true;
			s.axisOrientator = false;
			break;
		}

		return s;
	}
}