#version 460 core

// ------------------------ OUTPUTS & INPUTS ------------------------

out vec4 FragColour;

in vec3 WorldPos;
in vec3 Normal;
in vec2 TexCoords;

// ------------------------ UNIFORMS ------------------------

// material parameters
uniform vec3 albedo;
uniform float metallic;
uniform float roughness;
uniform float ao;

uniform bool useTexture;
uniform sampler2D albedoTex;

// Textures
uniform sampler2D dayTex;
uniform sampler2D nightTex;
uniform sampler2D cloudTex;
uniform sampler2D normalTex;
uniform sampler2D specularTex;

// CheckerBoard Plane parameters
uniform float checkSize;
uniform vec3 colour1;
uniform vec3 colour2;
uniform bool isFloor; // true for the checker floor, false for objects

// lights
uniform vec3 lightPosition;
uniform vec3 lightDirection;
uniform vec3 lightColour;
uniform float lightSize;
uniform float lightIntensity;

// Shadow map (Cascaded)
uniform sampler2DShadow cascadeShadowMap[2];
uniform mat4 lightSpaceMatrix[2];
uniform float cascadeSplits[2];
uniform vec3 camPos;

// IBL Maps
uniform samplerCube irradianceMap; // diffuse
uniform samplerCube prefilterMap;  // specular
uniform sampler2D brdfLUT;         // BRDF lookup texture

// ------------------------ CONSTANTS ------------------------

const float PI = 3.14159265359;

// ------------------------ PBR FUNCTIONS ------------------------

// Cook-Torrance BRDF components
float DistributionGGX(vec3 N, vec3 H, float roughness)
{
  float a = roughness * roughness;
  float a2 = a * a;
  float NdotH = max(dot(N, H), 0.0);
  float NdotH2 = NdotH * NdotH;

  float nom = a2;
  float denom = (NdotH2 * (a2 - 1.0) + 1.0);
  denom = PI * denom * denom;

  return nom / max(denom, 0.0000001);
}

// Schlick-GGX Geometry function
float GeometrySchlickGGX(float NdotV, float roughness)
{
  float r = (roughness + 1.0);
  float k = (r * r) / 8.0;

  float nom = NdotV;
  float denom = NdotV * (1.0 - k) + k;

  return nom / denom;
}

// Smith's method for combined geometry term
float GeometrySmith(vec3 N, vec3 V, vec3 L, float roughness)
{
  float NdotV = max(dot(N, V), 0.0);
  float NdotL = max(dot(N, L), 0.0);
  float ggx2 = GeometrySchlickGGX(NdotV, roughness);
  float ggx1 = GeometrySchlickGGX(NdotL, roughness);

  return ggx1 * ggx2;
}

vec3 fresnelSchlick(float cosTheta, vec3 F0)
{
  return F0 + (1.0 - F0) * pow(max(1.0 - cosTheta, 0.0), 5.0);
}

// ------------------------ SHADOW FUNCTIONS ------------------------

float computeShadowCSM(vec3 worldPos, vec3 N, vec3 L)
{
	// 1. Compute distance from camera to fragment
    float currentDepth = length(worldPos - camPos);

	// 2. Determine which cascade the fragment is in
	// Choose cascade
	int cascadeIndex = (currentDepth > cascadeSplits[0]) ? 1 : 0;

	// Fragment beyond last cascade -> no shadow
	if (currentDepth > cascadeSplits[1]) { return 0.0; }

	// 3. Transforming Cascade into light space
	vec4 lightSpacePos = lightSpaceMatrix[cascadeIndex] * vec4(worldPos, 1.0);
	vec3 projCoords = lightSpacePos.xyz / lightSpacePos.w;
	projCoords = projCoords * 0.5 + 0.5;

	// 4. Check if fragment is outside shadow map bounds
    if (projCoords.z > 1.0) return 0.0;
    if (projCoords.x < 0.0 || projCoords.x > 1.0 ||
        projCoords.y < 0.0 || projCoords.y > 1.0) return 0.0;

	// 5. Bias to prevent shadow acne
	float bias = max(0.005 * (1.0 - dot(N, L)), 0.001);

	// 6. PCF Sampling
	vec2 texelSize = 1.0 / textureSize(cascadeShadowMap[cascadeIndex], 0);
	bias += texelSize.x * 2.0;

	float sum = 0.0;
    int samples = 0;

    for (int x = -2;  x<= 2; ++x) {
        for (int y = -2; y <= 2; ++y) {
            vec3 coord = vec3(projCoords.xy + vec2(x, y) * texelSize, projCoords.z - bias);

            // Sample shadow map
            float lit = texture(cascadeShadowMap[cascadeIndex], coord);
            sum += lit;
            samples++;
        }
	}
    
	// 7. Average the results
    float litFactor = sum / float(samples);   // 1 = fully lit, 0 = fully shadowed
	float shadowAmount = 1.0 - litFactor;     // 0 = lit, 1 = shadow

    return shadowAmount;
}

// ------------------------ DIRECT PBR LIGHT EVAL ------------------------

vec3 evaluateDirectionalLightPBR(
	vec3 N, vec3 V,
	vec3 albedo, float roughness, float metallic,
	vec3 lightDir, vec3 lightCol, float intensity,
	float sizeScale,
	float shadow
) {
	// Light direction: from surface to light
	vec3 L = normalize(lightDir);
	vec3 H = normalize(V + L);

	float NdotL = max(dot(N, L), 0.0);
	if (NdotL <= 0.0)
		return vec3(0.0);

	vec3 F0 = mix(vec3(0.04), albedo, metallic);

	// Optionally scale roughness by light size for softer highlights
	float effectiveRoughness = clamp(roughness * sizeScale, 0.001, 1.0);

	float NDF = DistributionGGX(N, H, effectiveRoughness);
	float G = GeometrySmith(N, V, L, effectiveRoughness);
	vec3  F = fresnelSchlick(max(dot(H, V), 0.0), F0);

	float NdotV = max(dot(N, V), 0.0);

	vec3 kS = F;
	vec3 kD = (vec3(1.0) - kS) * (1.0 - metallic);

	vec3 nominator = NDF * G * F;
	float denominator = max(4.0 * NdotV * NdotL, 0.001);
	vec3 specular = nominator / denominator;

	vec3 radiance = lightCol * intensity;

	vec3 Lo = (kD * albedo / PI + specular) * radiance * NdotL;

	// Apply shadows here
	Lo *= (1.0 - shadow);

	return Lo;
}

// ------------------------ MAIN FUNCTION ------------------------

void main()
{
	vec3 L1 = normalize(lightPosition - WorldPos);
	FragColour = vec4(normalize(L1) * 0.5 + 0.5, 1.0);
	return;
}

