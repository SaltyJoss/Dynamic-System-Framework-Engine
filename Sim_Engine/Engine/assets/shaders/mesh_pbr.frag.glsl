#version 460 core

// ------------------------------------------------------------
// Outputs & Inputs
// ------------------------------------------------------------
out vec4 FragColour;

in vec3 WorldPos;
in vec3 Normal;
in vec2 TexCoords;
in vec4 Colour;

// ------------------------------------------------------------
// Material parameters
// ------------------------------------------------------------
uniform vec3  albedo;      // base colour
uniform float metallic;
uniform float roughness;
uniform float ao;
uniform float ambientStrength = 0.3; // IBL ambient multiplier

uniform bool        useTexture;
uniform sampler2D   albedoTex;

// ------------------------------------------------------------
// Lighting (single directional / distant light)
// ------------------------------------------------------------
uniform vec3 lightDirection;
uniform vec3 lightColour;
uniform float lightIntensity;

// Camera
uniform vec3 camPos;

// ------------------------------------------------------------
// Cascaded Shadow Mapping
// ------------------------------------------------------------
uniform sampler2DShadow cascadeShadowMap[2];
uniform mat4            lightSpaceMatrix[2];
uniform float           cascadeSplits[2];

// ------------------------------------------------------------
// Image-Based Lighting (IBL)
// ------------------------------------------------------------
uniform samplerCube irradianceMap;  // diffuse IBL
uniform samplerCube prefilterMap;   // specular IBL
uniform sampler2D   brdfLUT;        // precomputed BRDF integration

// ------------------------------------------------------------
// Constants
// ------------------------------------------------------------
const float PI = 3.14159265359;

// ------------------------------------------------------------
// PBR Helper Functions (Cook-Torrance GGX)
// ------------------------------------------------------------

float DistributionGGX(vec3 N, vec3 H, float roughness) {
    float a = roughness * roughness;
    float a2 = a * a;
    float NdotH = max(dot(N, H), 0.0);
    float NdotH2 = NdotH * NdotH;

    float nom = a2;
    float denom = (NdotH2 * (a2 - 1.0) + 1.0);
    denom = PI * denom * denom;

    return nom / max(denom, 0.0000001);
}

float GeometrySchlickGGX(float NdotV, float roughness) {
    float r = roughness + 1.0;
    float k = (r * r) / 8.0;

    float nom = NdotV;
    float denom = NdotV * (1.0 - k) + k;

    return nom / max(denom, 0.0000001);
}

float GeometrySmith(vec3 N, vec3 V, vec3 L, float roughness) {
    float NdotV = max(dot(N, V), 0.0);
    float NdotL = max(dot(N, L), 0.0);

    float ggx2 = GeometrySchlickGGX(NdotV, roughness);
    float ggx1 = GeometrySchlickGGX(NdotL, roughness);

    return ggx1 * ggx2;
}

vec3 fresnelSchlick(float cosTheta, vec3 F0) {
    return F0 + (1.0 - F0) * pow(max(1.0 - cosTheta, 0.0), 5.0);
}

// Fresnel with roughness for IBL (reduces overly strong reflections at grazing angles)
vec3 fresnelSchlickRoughness(float cosTheta, vec3 F0, float roughness) {
    return F0 + (max(vec3(1.0 - roughness), F0) - F0) *
        pow(max(1.0 - cosTheta, 0.0), 5.0);
}

// ------------------------------------------------------------
// Cascaded Shadow Mapping
// ------------------------------------------------------------

float shadowSingleCascade(int cascadeIndex, vec3 worldPos, vec3 N, vec3 L, float dist) {
    vec4 lightSpacePos = lightSpaceMatrix[cascadeIndex] * vec4(worldPos, 1.0);
    vec3 projCoords = lightSpacePos.xyz / lightSpacePos.w;
    projCoords = projCoords * 0.5 + 0.5;

    if (projCoords.z > 1.0 ||
        projCoords.x < 0.0 || projCoords.x > 1.0 ||
        projCoords.y < 0.0 || projCoords.y > 1.0)
        return 0.0;

    float ndotl = max(dot(N, L), 0.0);

    // --- PCF radius in texels ---
    float baseRadius = (cascadeIndex == 0) ? 2.0 : 5.0;
    
    // near cascade = tighter, far cascade = wider
    float radiusTexels = mix(baseRadius, baseRadius * 2.5, dist);
    radiusTexels *= mix(1.0, 1.25, projCoords.z);

    float slopeBias    = 0.0030 * (1.0 - ndotl);
    float receiverBias = 0.0010 + 0.0015 * radiusTexels;   // tune this pair if needed
    float bias         = slopeBias + receiverBias;

    // PCF
    vec2 texelSize = 1.0 / vec2(textureSize(cascadeShadowMap[cascadeIndex], 0));

    float sum = 0.0;
    int samples = 0;

    for (int x = -2; x <= 2; ++x)
    for (int y = -2; y <= 2; ++y) {
        vec2 offset = vec2(x, y) * texelSize * radiusTexels;
        float lit = texture(cascadeShadowMap[cascadeIndex], vec3(projCoords.xy + offset, projCoords.z - bias));
        sum += lit;
        samples++;
    }

    float litFactor = sum / float(samples);
    return 1.0 - litFactor; // 0 lit, 1 shadow
}

float computeShadowCSM(vec3 worldPos, vec3 N, vec3 L) {
    float d = length(worldPos - camPos);

    // normalize distance into [0..1] over the whole shadow range
    float dist01 = clamp(d / max(cascadeSplits[1], 0.0001), 0.0, 1.0);

    if (d > cascadeSplits[1]) return 0.0;

    // Blend region around split 0
    float blendWidth = max(1.0, 0.15 * cascadeSplits[0]);
    float t = smoothstep(cascadeSplits[0] - blendWidth, cascadeSplits[0] + blendWidth, d);

    float s0 = shadowSingleCascade(0, worldPos, N, L, dist01);
    float s1 = shadowSingleCascade(1, worldPos, N, L, dist01);

    return mix(s0, s1, t);
}

// ------------------------------------------------------------
// Procedural studio reflection for metallic surfaces.
// Simulates a soft overhead/key-light environment so metals
// have convincing reflections even with a plain white HDR.
// Only blends in when the actual environment map is weak.
// ------------------------------------------------------------
vec3 studioReflection(vec3 R, vec3 L, vec3 lightCol) {
    // Vertical gradient: brighter above (overhead softbox), darker below (floor)
    float up = smoothstep(-0.2, 1.0, R.y);
    vec3 env = mix(vec3(0.18), vec3(0.65), up);

    // Key light reflection — tight highlight along the sun direction
    float keyDot = max(dot(R, -L), 0.0);
    env += lightCol * pow(keyDot, 48.0) * 2.5;

    // Softer secondary lobe (broader sheen around the key)
    env += lightCol * pow(keyDot, 8.0) * 0.3;

    // Subtle fill from the opposite side
    float fillDot = max(dot(R, L), 0.0);
    env += lightCol * pow(fillDot, 12.0) * 0.2;

    return env;
}

// ------------------------------------------------------------
// Main
// ------------------------------------------------------------

void main() {
    vec3 N = normalize(Normal);
    vec3 V = normalize(camPos - WorldPos);
    vec3 L = normalize(-lightDirection);

    // Use raw NdotL for specular (physically correct compared to previous)
    float NdotL_raw = max(dot(N, L), 0.0);

    // Subtle wrap for diffuse only (softens terminator without ruining specular)
    float wrap = 0.15;
    float NdotL_wrap = clamp((dot(N, L) + wrap) / (1.0 + wrap), 0.0, 1.0);

    float NdotV = max(dot(N, V), 0.0);

    if (NdotV <= 0.0) {
        FragColour = vec4(0.0, 0.0, 0.0, 1.0);
        return;
    }

    vec3 baseColour = albedo;

    // Base reflectance -> dielectrics ~0.04, metals use albedo
    vec3 F0 = mix(vec3(0.04), baseColour, metallic);

    vec3 H = normalize(V + L);

    float NDF = DistributionGGX(N, H, roughness);
    float G   = GeometrySmith(N, V, L, roughness);
    vec3  F   = fresnelSchlick(max(dot(H, V), 0.0), F0);

    // --- Energy-conserving split ---
    // Fresnel tells us how much light is reflected (specular)
    // The rest is refracted (diffuse) —> metals have no diffuse
    vec3 kS = F;
    vec3 kD = (vec3(1.0) - kS) * (1.0 - metallic);

    // Cook-Torrance specular BRDF
    float denom = max(4.0 * NdotV * NdotL_raw, 0.001);
    vec3  specular = (NDF * G * F) / denom;

    vec3 radiance = lightColour * lightIntensity;

    // Combine: diffuse uses wrapped NdotL, specular uses raw NdotL
    vec3 Lo = (kD * baseColour / PI * NdotL_wrap + specular * NdotL_raw) * radiance;

    float shadow = computeShadowCSM(WorldPos, N, L);
    Lo *= (1.0 - shadow);

    // --- Diffuse IBL ---
    vec3 F_ibl = fresnelSchlickRoughness(NdotV, F0, roughness);
    vec3 kD_ibl = (vec3(1.0) - F_ibl) * (1.0 - metallic);

    vec3 irradiance = texture(irradianceMap, N).rgb;
    vec3 diffuseIBL = irradiance * baseColour;

    // --- Specular IBL ---
    vec3 R = reflect(-V, N);
    float maxMip = float(textureQueryLevels(prefilterMap) - 1);
    float r = max(roughness, 0.08);
    vec3 prefiltered = textureLod(prefilterMap, R, r * maxMip).rgb;

    // Blend with procedural studio reflection for metallic surfaces.
    // When the environment map is weak (e.g. the plain white HDR I use), metals have nothing to reflect and look dark.
    // The studio function provides a gradient + key-light highlight so metals read as shiny.
    // Blends out automatically when a rich HDRI is loaded.
    vec3 studioRef = studioReflection(R, L, lightColour);
    float envLum   = dot(prefiltered, vec3(0.2126, 0.7152, 0.0722));
    float studioBlend = metallic * smoothstep(0.5, 0.0, envLum);
    prefiltered = mix(prefiltered, studioRef, studioBlend);

    vec2 brdf = texture(brdfLUT, vec2(NdotV, roughness)).rg;
    vec3 specularIBL = prefiltered * (F_ibl * brdf.x + brdf.y);

    // Ambient: shadow contact darkening on diffuse only
    float contactShadow = smoothstep(0.0, 0.02, shadow);
    vec3 ambientDiffuse  = kD_ibl * diffuseIBL * ao * (1.0 - 0.5 * contactShadow);
    vec3 ambientSpecular = specularIBL * ao;
    vec3 ambient = ambientDiffuse * ambientStrength + ambientSpecular;

    vec3 colour = ambient + Lo;

    FragColour = vec4(colour, 1.0);
}