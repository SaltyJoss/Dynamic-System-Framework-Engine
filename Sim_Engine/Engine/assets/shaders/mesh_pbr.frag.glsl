#version 460 core

// ------------------------------------------------------------
// Outputs & Inputs
// ------------------------------------------------------------
out vec4 FragColour;

in vec3 WorldPos;
in vec3 Normal;
in vec2 TexCoords;

// ------------------------------------------------------------
// Material parameters
// ------------------------------------------------------------
uniform vec3  albedo;      // base colour
uniform float metallic;
uniform float roughness;
uniform float ao;

uniform bool        useTexture;
uniform sampler2D   albedoTex;

// ------------------------------------------------------------
// Lighting (single directional / distant light)
// ------------------------------------------------------------
uniform vec3 lightPosition;   // treated as position for now
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

float GeometrySchlickGGX(float NdotV, float roughness)
{
    float r = roughness + 1.0;
    float k = (r * r) / 8.0;

    float nom = NdotV;
    float denom = NdotV * (1.0 - k) + k;

    return nom / max(denom, 0.0000001);
}

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

// Fresnel with roughness for IBL (reduces overly strong reflections at grazing angles)
vec3 fresnelSchlickRoughness(float cosTheta, vec3 F0, float roughness)
{
    return F0 + (max(vec3(1.0 - roughness), F0) - F0) *
        pow(max(1.0 - cosTheta, 0.0), 5.0);
}

// ------------------------------------------------------------
// Cascaded Shadow Mapping
// ------------------------------------------------------------

float computeShadowCSM(vec3 worldPos, vec3 N, vec3 L)
{
    // Distance from camera to fragment
    float currentDepth = length(worldPos - camPos);

    // Choose cascade index based on distance
    int cascadeIndex = (currentDepth > cascadeSplits[0]) ? 1 : 0;

    // If beyond last cascade, no shadow
    if (currentDepth > cascadeSplits[1])
        return 0.0;

    // Transform fragment position to light space for chosen cascade
    vec4 lightSpacePos = lightSpaceMatrix[cascadeIndex] * vec4(worldPos, 1.0);
    vec3 projCoords = lightSpacePos.xyz / lightSpacePos.w;
    projCoords = projCoords * 0.5 + 0.5; // NDC [-1,1] -> [0,1]

    // Outside shadow map bounds -> no shadow
    if (projCoords.z > 1.0 ||
        projCoords.x < 0.0 || projCoords.x > 1.0 ||
        projCoords.y < 0.0 || projCoords.y > 1.0)
    {
        return 0.0;
    }

    // Bias to reduce shadow acne
    float bias = max(0.005 * (1.0 - dot(N, L)), 0.001);

    // PCF (5x5 kernel)
    vec2 texelSize = 1.0 / textureSize(cascadeShadowMap[cascadeIndex], 0);
    float sum = 0.0;
    int samples = 0;

    for (int x = -2; x <= 2; ++x)
    {
        for (int y = -2; y <= 2; ++y)
        {
            vec3 coord = vec3(
                projCoords.xy + vec2(x, y) * texelSize,
                projCoords.z - bias
            );

            float lit = texture(cascadeShadowMap[cascadeIndex], coord);
            sum += lit;
            samples++;
        }
    }

    float litFactor = sum / float(samples); // 1 = lit, 0 = shadowed
    float shadowAmount = 1.0 - litFactor;      // 0 = lit, 1 = shadow

    return shadowAmount;
}

// ------------------------------------------------------------
// Main
// ------------------------------------------------------------

void main()
{
    // --- Basic vectors ---
    vec3 N = normalize(Normal);
    vec3 V = normalize(camPos - WorldPos);

    // Light treated as positional; if you want directional, normalise lightDirection instead
    vec3 L = normalize(lightPosition - WorldPos);
    vec3 H = normalize(V + L);

    float NdotL = max(dot(N, L), 0.0);
    float NdotV = max(dot(N, V), 0.0);

    // If light is behind the surface, we can early out on direct lighting
    if (NdotL <= 0.0 || NdotV <= 0.0)
    {
        // Still allow IBL to contribute some ambient
        vec3 baseColour = albedo;
        if (useTexture)
            baseColour *= texture(albedoTex, TexCoords).rgb;

        vec3 F0 = mix(vec3(0.04), baseColour, metallic);

        // IBL diffuse
        vec3 irradiance = texture(irradianceMap, N).rgb;
        vec3 diffuseIBL = irradiance * baseColour * (1.0 - metallic);

        // IBL specular
        vec3 R = reflect(-V, N);
        vec3 prefiltered = textureLod(prefilterMap, R, roughness * 4.0).rgb;
        vec2 brdf = texture(brdfLUT, vec2(NdotV, roughness)).rg;
        vec3 F_ibl = fresnelSchlickRoughness(NdotV, F0, roughness);
        vec3 specularIBL = prefiltered * (F_ibl * brdf.x + brdf.y);

        vec3 colour = diffuseIBL + specularIBL;
        colour *= ao;

        FragColour = vec4(colour, 1.0);
        return;
    }

    // --- Material base colour, with optional albedo texture ---
    vec3 baseColour = albedo;
    if (useTexture)
        baseColour *= texture(albedoTex, TexCoords).rgb;

    // --- Base reflectance F0 ---
    vec3 F0 = mix(vec3(0.04), baseColour, metallic);

    // --- Cook-Torrance BRDF for direct lighting ---
    float NDF = DistributionGGX(N, H, roughness);
    float G = GeometrySmith(N, V, L, roughness);
    vec3  F = fresnelSchlick(max(dot(H, V), 0.0), F0);

    vec3 kS = F;
    vec3 kD = (vec3(1.0) - kS) * (1.0 - metallic);

    float denom = max(4.0 * NdotV * NdotL, 0.0001);
    vec3  specular = (NDF * G * F) / denom;

    vec3 radiance = lightColour * lightIntensity;

    vec3 Lo = (kD * baseColour / PI + specular) * radiance * NdotL;

    // --- Shadowing ---
    float shadow = computeShadowCSM(WorldPos, N, L);
    Lo *= (1.0 - shadow);

    // --------------------------------------------------------
    // Image-Based Lighting (IBL)
    // --------------------------------------------------------
    vec3 irradiance = texture(irradianceMap, N).rgb;
    vec3 diffuseIBL = irradiance * baseColour * (1.0 - metallic);

    vec3 R = reflect(-V, N);
    vec3 prefiltered = textureLod(prefilterMap, R, roughness * 4.0).rgb;
    vec2 brdf = texture(brdfLUT, vec2(NdotV, roughness)).rg;
    vec3 F_ibl = fresnelSchlickRoughness(NdotV, F0, roughness);
    vec3 specularIBL = prefiltered * (F_ibl * brdf.x + brdf.y);

    vec3 ambient = (diffuseIBL + specularIBL) * ao;

    // --------------------------------------------------------
    // Final colour
    // --------------------------------------------------------
    vec3 colour = ambient + Lo;

    // You can add tone mapping / gamma correction here if desired:
    // colour = colour / (colour + vec3(1.0));
    // colour = pow(colour, vec3(1.0/2.2));

    FragColour = vec4(colour, 1.0);
}
