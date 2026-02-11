#pragma once
// File:    RenderPreset.h
// GitHub:  SaltyJoss

namespace render {
	enum class ResolutionPreset { R_720p, R_1080p, R_1440p, R_4K };
	enum class QualityPreset { Low, Medium, High, Ultra};

	struct RenderSettings {
		// Pipeline
		bool hdr = true;
		float exposure = 1.0f;
		float whitePoint = 1.0f;

		// Lighting
		bool ibl = true;
		int envPreFilterRes = 256;	// Resolution for environment pre-filtered map, default 256 
		int envIrradianceRes = 32;	// Resolution for environment irradiance map, default 32

		// Shadows
		bool shadows = true;
		int shadowMapRes = 2048;
		int shadowCascades = 2; // 1 or 2 for single, 3-4 for larger scenes
		int pcfKernel = 3;		// Percentage-closer filtering kernel size, 3,5,7 for PCF and 9 for VSM (Viariance Shadow Maps)

		// Ambient Occlusion
		bool ssao = true;
		int ssaoResDiv = 2;  // Resolution division factor for SSAO buffer, 1 = full res, 2 = half res, 4 = quarter res
		int ssaoSamples = 16; // Number of samples for SSAO, higher = better quality
		float ssaoStrength = 0.3f;

		// Post Processing
		bool bloom = true;
		float bloomThreshold = 0.5f;

		// Anti-Aliasing
		bool fxaa = false;	// Fast Approximate Anti-Aliasing -> if msaa > 1, fxaa is ignored
		bool taa = false;	// Temporal Anti-Aliasing, if msaa > 1, taa is ignored
		int msaaSamples = 2; // 1 = off, 2,4,8

		// Editor Extras
		bool grid = true;
		bool axisOrientator = true;

		// Render-Specific
		float renderScale = 1.0f;
		float ambientStrength = 0.5f; // IBL ambient multiplier (0.0 = no ambient, 1.0 = full)
	};

	RenderSettings MakeSettings(ResolutionPreset res, QualityPreset qual);
} // namespace render