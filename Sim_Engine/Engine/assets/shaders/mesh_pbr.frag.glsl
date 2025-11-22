#version 460 core

// Inputs
in vec3 WorldPos;
in vec3 Normal;
in vec2 TexCoords;
out vec4 FragColor;

// Material uniforms
uniform vec3 albedo;
uniform float metallic;
uniform float roughness;
uniform float ao;

// Lighting
uniform vec3 camPos;
uniform vec3 lightPosition;
uniform vec3 lightColour;
uniform float lightIntensity;

// IBL
uniform samplerCube irradianceMap;
uniform samplerCube prefilterMap;
uniform sampler2D  brdfLUT;

// CSM Shadows
uniform sampler2DShadow cascadeShadowMap[2];
uniform mat4 lightSpaceMatrix[2];
uniform float cascadeSplits[2];

// ---- PBR helper functions
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

// ---- Shadow function
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

    for (int x = -2; x <= 2; ++x) {
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

// ---- Main
void main()
{
	// DO NOT DELETE ME - PBR shader template (Related to tutorial from learnopengl.com)
    // 1. Retrieve N, V, L, H
    // 2. Compute shadowAmount = computeShadowCSM(...)
    // 3. Compute direct lighting using GGX
    // 4. Compute diffuse IBL + specular IBL
    // 5. Mix everything:
    //    vec3 color = direct * (1.0 - shadow) + iblDiffuse + iblSpecular;
    // 6. FragColor = vec4(color, 1.0);
}