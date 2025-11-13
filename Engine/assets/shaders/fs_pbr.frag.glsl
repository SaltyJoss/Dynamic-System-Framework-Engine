#version 460 core

// ------------------------ OUTPUTS & INPUTS ------------------------

out vec4 FragColour;
in vec3 WorldPos;
in vec3 Normal;

// ------------------------ UNIFORMS ------------------------

// material parameters
uniform vec3 albedo;
uniform float metallic;
uniform float roughness;
uniform float ao;

// CheckerBoard Plane parameters
uniform float checkSize;
uniform vec3 colour1;
uniform vec3 colour2;
uniform bool isFloor; // true for the checker floor, false for objects

// lights
uniform vec3 lightPosition;
uniform vec3 lightColour;
uniform float lightSize;
uniform vec3 lightDirection;

// Shadow map (Cascaded)
uniform sampler2DShadow cascadeShadowMap[2];
uniform mat4 lightSpaceMatrixCascade[2];
uniform float cascadeSplits[2];
uniform vec3 camPos;

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
	vec4 lightSpacePos = lightSpaceMatrixCascade[cascadeIndex] * vec4(worldPos, 1.0);
	vec3 projCoords = lightSpacePos.xyz / lightSpacePos.w;
	projCoords = projCoords * 0.5 + 0.5;

	// 4. Check if fragment is outside shadow map bounds
    if (projCoords.z > 1.0) return 0.0;
    if (projCoords.x < 0.0 || projCoords.x > 1.0 ||
        projCoords.y < 0.0 || projCoords.y > 1.0) return 0.0;

	// 5. Bias to prevent shadow acne
    float bias = max(0.002 * (1.0 - dot(N, L)), 0.0007);

	// 6. PCF Sampling
	vec2 texelSize = 1.0 / textureSize(cascadeShadowMap[cascadeIndex], 0);
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

// ------------------------ MAIN FUNCTION ------------------------

void main()
{
	// normalise inputs
    vec3 N = normalize(Normal);
    vec3 V = normalize(camPos - WorldPos);

    // determine albedo colour
    vec3 finalAlbedo = albedo;
    if (isFloor)
    {
        float pattern = mod(floor(WorldPos.x * checkSize) + floor(WorldPos.z * checkSize), 2.0);
        finalAlbedo = mix(colour1, colour2, pattern);
    }

    // base reflection (Cook-Torrance PBR)
    vec3 F0 = vec3(0.04);
    F0 = mix(F0, finalAlbedo, metallic);

    vec3 Lo = vec3(0.0);

	// Single directional light
    vec3 L = normalize(-lightDirection);
    vec3 H = normalize(V + L);

    vec3 radiance = lightColour;

	// Compute shadow using CSM
    float shadow = computeShadowCSM(WorldPos, N, L);

    // Cook-Torrance BRDF
    float NDF = DistributionGGX(N, H, roughness * lightSize);
    float G = GeometrySmith(N, V, L, roughness);
    vec3 F = fresnelSchlick(clamp(dot(H, V), 0.0, 1.0), F0);

    vec3 nominator = NDF * G * F;
    float denominator = 4 * max(dot(N, V), 0.0) * max(dot(N, L), 0.0);
    vec3 specular = nominator / max(denominator, 0.001); // prevent divide by zero for NdotV=0.0 or NdotL=0.0

	// Energy conservation
    vec3 kS = F;
    vec3 kD = vec3(1.0) - kS;
    kD *= 1.0 - metallic;
     
    float NdotL = max(dot(N, L), 0.0);

    Lo += (kD * finalAlbedo / PI + specular) * radiance * NdotL;

    vec3 ambient = vec3(0.03) * finalAlbedo * ao;
    vec3 colour = ambient + (1.0 - shadow) * Lo;

    // HDR tonemapping
    colour = colour / (colour + vec3(1.0));
    colour = pow(colour, vec3(1.0 / 2.2));

    FragColour = vec4(colour, 1.0);
}
