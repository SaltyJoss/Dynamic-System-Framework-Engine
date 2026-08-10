#version 450
layout(location = 0) in vec3 inPos;
layout(location = 1) in vec3 inColour;
layout(push_constant) uniform PC {
    mat4 mvp;
    mat4 model;
    vec4 albedo;
    vec4 material;
} pc;
layout(location = 0) out vec3 vColour;
void main() {
    gl_Position = pc.mvp * vec4(inPos, 1.0);
    vColour = inColour;
}