#version 450

layout(location = 0) in vec3 v_world_pos;
layout(location = 1) in vec3 v_normal;

layout(set = 0, binding = 0) uniform Camera {
    mat4 view;
    mat4 proj;
    vec4 cam_pos;
} cam;

layout(push_constant) uniform Push {
    mat4 mvp;
    mat4 model;
    vec4 albedo;      // rgb = base colour
    vec4 material;    // x=metallic, y=roughness, z=ao
} pc;

layout(location = 0) out vec4 o_colour;

const float PI = 3.14159265359;

// Hardcoded key light — moves into a light UBO with the shadow pass (Step C)
const vec3  LIGHT_DIR       = normalize(vec3(-0.4, -1.0, -0.3));
const vec3  LIGHT_COL       = vec3(1.0);
const float LIGHT_INTENSITY = 3.0;
const float AMBIENT         = 0.03;

float DistributionGGX(vec3 N, vec3 H, float roughness) {
    float a  = roughness * roughness;
    float a2 = a * a;
    float NdotH  = max(dot(N, H), 0.0);
    float NdotH2 = NdotH * NdotH;
    float denom = (NdotH2 * (a2 - 1.0) + 1.0);
    denom = PI * denom * denom;
    return a2 / max(denom, 0.0000001);
}

float GeometrySchlickGGX(float NdotV, float roughness) {
    float r = roughness + 1.0;
    float k = (r * r) / 8.0;
    return NdotV / max(NdotV * (1.0 - k) + k, 0.0000001);
}

float GeometrySmith(vec3 N, vec3 V, vec3 L, float roughness) {
    return GeometrySchlickGGX(max(dot(N, V), 0.0), roughness)
         * GeometrySchlickGGX(max(dot(N, L), 0.0), roughness);
}

vec3 fresnelSchlick(float cosTheta, vec3 F0) {
    return F0 + (1.0 - F0) * pow(max(1.0 - cosTheta, 0.0), 5.0);
}

vec3 fresnelSchlickRoughness(float cosTheta, vec3 F0, float roughness) {
    return F0 + (max(vec3(1.0 - roughness), F0) - F0) * pow(max(1.0 - cosTheta, 0.0), 5.0);
}

// Your analytic studio environment — reflections with zero IBL precompute
vec3 studioReflection(vec3 R, vec3 L, vec3 lightCol) {
    float up = smoothstep(-0.2, 1.0, R.y);
    vec3 env = mix(vec3(0.18), vec3(0.65), up);
    float keyDot = max(dot(R, -L), 0.0);
    env += lightCol * pow(keyDot, 48.0) * 2.5;
    env += lightCol * pow(keyDot, 8.0)  * 0.3;
    float fillDot = max(dot(R, L), 0.0);
    env += lightCol * pow(fillDot, 12.0) * 0.2;
    return env;
}

vec3 ACESFilm(vec3 x) {
    float a = 2.51, b = 0.03, c = 2.43, d = 0.59, e = 0.14;
    return clamp((x*(a*x + b)) / (x*(c*x + d) + e), 0.0, 1.0);
}

void main() {
    vec3 N = normalize(v_normal);
    vec3 V = normalize(cam.cam_pos.xyz - v_world_pos);
    vec3 L = -LIGHT_DIR;

    float metallic  = pc.material.x;
    float roughness = clamp(pc.material.y, 0.04, 1.0);
    float ao        = pc.material.z;
    vec3  baseColour = pc.albedo.rgb;

    float NdotL_raw = max(dot(N, L), 0.0);
    float wrap = 0.2;
    float NdotL_wrap = clamp((dot(N, L) + wrap) / (1.0 + wrap), 0.0, 1.0);
    float NdotV = max(dot(N, V), 0.001);

    vec3 F0 = mix(vec3(0.04), baseColour, metallic);

    vec3 H = normalize(V + L);
    float NDF = DistributionGGX(N, H, roughness);
    float G   = GeometrySmith(N, V, L, roughness);
    vec3  F   = fresnelSchlick(max(dot(H, V), 0.0), F0);

    vec3 kD = (vec3(1.0) - F) * (1.0 - metallic);
    vec3 specular = (NDF * G * F) / (4.0 * NdotV * NdotL_raw + 0.001);

    vec3 radiance = LIGHT_COL * LIGHT_INTENSITY;
    vec3 Lo = (kD * baseColour / PI * NdotL_wrap + specular * NdotL_raw) * radiance;

    // Ambient: flat floor for dielectrics + studio env reflection for metals
    vec3 R = reflect(-V, N);
    vec3 F_ibl = fresnelSchlickRoughness(NdotV, F0, roughness);
    vec3 envSpec = studioReflection(R, L, LIGHT_COL) * F_ibl * (1.0 - roughness) * 0.35;
    vec3 ambient = baseColour * AMBIENT * ao + envSpec * ao;

    vec3 colour = ACESFilm(ambient + Lo);
    o_colour = vec4(colour, 1.0);
}