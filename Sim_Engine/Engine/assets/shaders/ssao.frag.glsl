#version 460 core

in vec2 uv;
out float FragOcclusion;

uniform sampler2D gDepth;       // scene depth (GL_DEPTH24_STENCIL8)
uniform sampler2D gNoise;       // 4x4 random rotation texture

uniform vec3  samples[64];      // hemisphere kernel in tangent space
uniform int   kernelSize;       // number of samples (16, 32, 64)
uniform float radius;           // sampling radius in world units
uniform float bias;             // depth bias to prevent self-occlusion
uniform float strength;         // occlusion strength multiplier

uniform mat4 projection;        // camera projection matrix
uniform mat4 invProjection;     // inverse projection

uniform vec2 noiseScale;        // screen size / noise texture size

// Reconstruct view-space position from depth buffer
vec3 viewPosFromDepth(vec2 texCoord) {
    float depth = texture(gDepth, texCoord).r;
    // NDC: [-1, 1]
    vec4 ndc = vec4(texCoord * 2.0 - 1.0, depth * 2.0 - 1.0, 1.0);
    vec4 viewPos = invProjection * ndc;
    return viewPos.xyz / viewPos.w;
}

// Approximate view-space normal from depth via cross product of partial derivatives
vec3 viewNormalFromDepth(vec3 viewPos) {
    vec3 dpdx = dFdx(viewPos);
    vec3 dpdy = dFdy(viewPos);
    return normalize(cross(dpdy, dpdx));
}

void main() {
    vec3 fragPos = viewPosFromDepth(uv);
    vec3 normal  = viewNormalFromDepth(fragPos);

    // Random rotation vector from noise texture
    vec3 randomVec = normalize(texture(gNoise, uv * noiseScale).xyz * 2.0 - 1.0);

    // Gram-Schmidt to build TBN from normal + random vector
    vec3 tangent   = normalize(randomVec - normal * dot(randomVec, normal));
    vec3 bitangent = cross(normal, tangent);
    mat3 TBN       = mat3(tangent, bitangent, normal);

    float occlusion = 0.0;

    for (int i = 0; i < kernelSize; ++i) {
        // Transform sample from tangent space to view space
        vec3 samplePos = TBN * samples[i];
        samplePos = fragPos + samplePos * radius;

        // Project sample to screen space
        vec4 offset = projection * vec4(samplePos, 1.0);
        offset.xyz /= offset.w;
        offset.xyz  = offset.xyz * 0.5 + 0.5;

        // Sample depth at projected position
        float sampleDepth = viewPosFromDepth(offset.xy).z;

        // Range check: only occlude if within radius
        float rangeCheck = smoothstep(0.0, 1.0, radius / max(abs(fragPos.z - sampleDepth), 1e-5));
        occlusion += (sampleDepth >= samplePos.z + bias ? 1.0 : 0.0) * rangeCheck;
    }

    occlusion = 1.0 - (occlusion / float(kernelSize)) * strength;
    FragOcclusion = clamp(occlusion, 0.0, 1.0);
}
