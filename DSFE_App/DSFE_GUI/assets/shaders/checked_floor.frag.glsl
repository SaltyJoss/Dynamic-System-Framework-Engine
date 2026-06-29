#version 460 core
out vec4 FragColour;

in vec3 WorldPos;
in float vViewZ;
in vec4 vScreenPos;

// Framework Uniform Alignment
uniform vec3 gCameraWorldPos;
uniform vec3 lightDirection;  
uniform vec3 lightColour;
uniform float lightIntensity;

// Cascaded Shadow Mapping Alignment
uniform sampler2DShadow cascadeShadowMap[2];
uniform mat4            lightSpaceMatrix[2];
uniform float           cascadeSplits[2];

// Image-Based Lighting Alignment
uniform samplerCube irradianceMap;
uniform samplerCube prefilterMap;
uniform sampler2D   brdfLUT;

// Real-time dynamic mirror buffer bound to texture slot 3
uniform sampler2D   planarReflectionMap; 

// Local Scene Buffer Inputs (Optional, can be omitted for standard depth-testing)
uniform sampler2D sceneDepthTexture; 

vec2 poissonDisk[12] = vec2[](
    vec2(-0.326212, -0.405805), vec2(-0.840144, -0.07358),
    vec2(-0.695914,  0.457137), vec2(-0.203345,  0.620716),
    vec2( 0.96234,  -0.194983), vec2( 0.473434, -0.480026),
    vec2( 0.519456,  0.767022), vec2( 0.185461, -0.893124),
    vec2( 0.507431,  0.064425), vec2( 0.89642,   0.412458),
    vec2(-0.32194,  -0.932615), vec2(-0.791559, -0.597705)
);

float shadowSingleCascade(int cascadeIndex, vec3 worldPos, vec3 N, vec3 L, float dist) {
    vec3 biasedPos = worldPos + N * 0.002;
    vec4 lightSpacePos = lightSpaceMatrix[cascadeIndex] * vec4(biasedPos, 1.0);
    vec3 projCoords = lightSpacePos.xyz / lightSpacePos.w;
    projCoords = projCoords * 0.5 + 0.5;

    if (projCoords.z > 1.0) { return 0.0; }

    float ndotl = max(dot(N, L), 0.0);
    float bias = max(0.005 * (1.0 - ndotl), 0.0005);
    vec2 texelSize = 1.0 / vec2(textureSize(cascadeShadowMap[cascadeIndex], 0));
    float spreadRadius = mix(1.5, 3.5, dist); 
    float currentDepth = projCoords.z - bias;
    float shadowSum = 0.0;

    for (int i = 0; i < 12; i++) {
        vec2 offset = poissonDisk[i] * texelSize * spreadRadius;
        float depthSample = texture(cascadeShadowMap[cascadeIndex], vec3(projCoords.xy + offset, currentDepth));
        shadowSum += depthSample;
    }

    return 1.0 - (shadowSum / 12.0);
}

float computeShadowCSM(vec3 worldPos, vec3 N, vec3 L) {
    float d = length(worldPos - gCameraWorldPos);
    float dist01 = clamp(d / max(cascadeSplits[1], 0.0001), 0.0, 1.0);

    if (d > cascadeSplits[1]) return 0.0;

    float blendWidth = max(1.0, 0.15 * cascadeSplits[0]);
    float t = smoothstep(cascadeSplits[0] - blendWidth, cascadeSplits[0] + blendWidth, d);

    float s0 = shadowSingleCascade(0, worldPos, N, L, dist01);
    float s1 = shadowSingleCascade(1, worldPos, N, L, dist01);

    return mix(s0, s1, t);
}

vec3 calculateCheckered(vec2 uv, vec3 colorA, vec3 colorB) {
    vec2 width = fwidth(uv) * 0.7071; 
    vec2 pulse = sin(uv * 3.14159265) / max(width, 0.0001);
    pulse = clamp(pulse, -1.0, 1.0);
    return mix(colorA, colorB, (pulse.x * pulse.y) * 0.5 + 0.5);
}

void main() {
    float hold = 2500.0;  
    float falloff = 1000.0;
    float t = max(vViewZ - hold, 0.0);
    float fadeFactor = exp(-t / falloff);

    if (fadeFactor < 0.001) { discard; }

    vec3 N = vec3(0.0, 1.0, 0.0);
    vec3 V = normalize(gCameraWorldPos - WorldPos);
    vec3 L = normalize(-lightDirection);

    // Render Checkerboard Pattern
    // WorldPos.xz is perfectly locked to the world axes. 1.0 = 1 meter squares.
    vec2 GroundUV = WorldPos.xz * 16.0; 
    vec3 c1 = vec3(0.07, 0.08, 0.40); 
    vec3 c2      = vec3(0.16, 0.18, 0.20); 
    vec3 albedo = calculateCheckered(GroundUV, c1, c2);

    float metallic  = 0.15; 
    float roughness = 0.04; 

    float NdotL = max(dot(N, L), 0.0);
    float NdotV = max(dot(N, V), 0.0);
    vec3 F0 = mix(vec3(0.04), albedo, metallic);
    
    // Light and CSM shadow computations
    vec3 diffuseDirect = (albedo / 3.14159265) * NdotL * lightColour * lightIntensity;
    float shadow = computeShadowCSM(WorldPos, N, L);
    diffuseDirect *= (1.0 - shadow);

    // Specular IBL Environment reflections
    vec3 R = reflect(-V, N);
    vec3 staticEnvColour = textureLod(prefilterMap, R, roughness * 4.0).rgb;

    vec2 screenUV = vScreenPos.xy / vScreenPos.w;
    screenUV = screenUV * 0.5 + 0.5; // Map from [-1, 1] device space to [0, 1] texture coordinates

    vec3 dynamicMeshColour = texture(planarReflectionMap, screenUV).rgb; // Reads dynamic high-quality RGBA16F reflection buffer
    vec3 reflectionColour = mix(staticEnvColour, dynamicMeshColour, 0.75); // Blends real-time meshes over static background box reflections

    vec2 brdf = texture(brdfLUT, vec2(NdotV, roughness)).rg;
    vec3 specularIBL = reflectionColour * (F0 * brdf.x + brdf.y);
    specularIBL *= mix(1.0, 1.0 - shadow, 0.5); 

    vec3 finalColour = diffuseDirect + specularIBL;
    finalColour = finalColour / (finalColour + vec3(1.0)); 
    
    FragColour = vec4(finalColour, fadeFactor);
}