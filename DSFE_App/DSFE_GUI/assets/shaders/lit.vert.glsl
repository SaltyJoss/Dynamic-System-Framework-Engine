#version 450

layout(location = 0) in vec3 in_pos;
layout(location = 1) in vec3 in_normal;
layout(location = 2) in vec2 in_uv;

layout(set = 0, binding = 0) uniform Camera {
    mat4 view;
    mat4 proj;
    vec4 cam_pos;
} cam;

layout(push_constant) uniform Push {
    mat4 mvp;
    mat4 model;
    vec4 albedo;
    vec4 material;
} pc;

layout(location = 0) out vec3 v_world_pos;
layout(location = 1) out vec3 v_normal;

void main() {
    v_world_pos = vec3(pc.model * vec4(in_pos, 1.0));
    // Normal matrix: transpose(inverse(mat3(model))) for correct non-uniform scale
    mat3 nmat = transpose(inverse(mat3(pc.model)));
    v_normal = normalize(nmat * in_normal);
    gl_Position = pc.mvp * vec4(in_pos, 1.0);
}