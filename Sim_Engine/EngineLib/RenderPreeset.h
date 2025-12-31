#pragma once

//=============================================
//            File: RenderPreset.h
//=============================================
// Enumeration for different rendering presets.
//
// Summary:
// ============================================
//
// =============================================

namespace render {
	enum class LookPreset{ Studio, Cinematic };
	enum class QualityPreset { Low, Medium, High};

	struct RenderSettings {
		// Pipeline
		bool hdr = true;
		bool tonemap = true;
		float exposure = 1.0f;

		// Lighting
		bool ibl = true;
		int envPreFilterRes = 256;
		int envIrradianceRes = 32;

		// Shadows
		bool shadows = true;
		int shadowMapRes = 2048;
		int shadowCascades = 2; // 1 or 2 for single, 3-4 for larger scenes
		int pcfKernel = 3;		// Percentage-closer filtering kernel size, 3,5,7 for PCF

		// Ambient Occlusion
		bool ssao = true;
		int ssaoResDiv = 2; // Resolution division factor for SSAO buffer
		int ssaoSamples = 8;
		float ssaoStrength = 1.0f;

		// Post Processing
		bool bloom = true;
		float bloomThreshold = 0.5f;

		// Anti-Aliasing
		bool fxaa = false;
		bool taa = false;
		int msaaSamples = 4; // 0 = off, 2,4,8

		// Editor Extras
		bool grid = true;
		bool axisOrientator = true;
	};

	RenderSettings MakeSettings(render::LookPreset look, render::QualityPreset quality);
}