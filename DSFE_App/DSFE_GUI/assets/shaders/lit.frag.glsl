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
    vec4 albedo;
    vec4 material;   // x=metallic, y=roughness, z=ao
} pc;

layout(location = 0) out vec4 o_colour;

void main() {
    vec3 N = normalize(v_normal);
    vec3 V = normalize(cam.cam_pos.xyz - v_world_pos);

    // Single directional key light (hardcoded for now; light UBO later).
    vec3 L = normalize(vec3(-0.4, -1.0, -0.3));
    vec3 light_dir = -L;
    vec3 light_col = vec3(1.0);

    float ndotl = max(dot(N, light_dir), 0.0);

    // Diffuse + a cheap Blinn-Phong spec so metals read as shiny.
    vec3 H = normalize(light_dir + V);
    float ndoth = max(dot(N, H), 0.0);
    float spec = pow(ndoth, mix(8.0, 128.0, 1.0 - pc.material.y));  // roughness -> shininess

    vec3 albedo = pc.albedo.rgb;
    vec3 diffuse = albedo * ndotl;
    vec3 specular = light_col * spec * mix(0.04, 1.0, pc.material.x);  // metallic -> spec strength
    vec3 ambient = albedo * 0.15;   // flat ambient floor

    vec3 colour = ambient + (diffuse + specular) * light_col;
    o_colour = vec4(colour, 1.0);
}