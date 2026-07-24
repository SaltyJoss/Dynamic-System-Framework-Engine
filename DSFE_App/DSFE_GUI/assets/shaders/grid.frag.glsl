#version 450

layout(location = 0) in vec3 v_world_pos;

layout(set = 0, binding = 0) uniform Camera {
    mat4 view;
    mat4 proj;
    vec4 cam_pos;
} cam;

layout(location = 0) out vec4 o_colour;

const vec3 BG    = vec3(0.02, 0.02, 0.025);  // MUST match the clear colour
const vec3 FLOOR = vec3(0.11, 0.11, 0.12);
const vec3 MINOR = vec3(0.16, 0.16, 0.17);
const vec3 MAJOR = vec3(0.21, 0.21, 0.23);

float gridLine(vec2 p, float cell) {
    vec2 q = p / cell;
    vec2 g = abs(fract(q - 0.5) - 0.5) / fwidth(q);
    return 1.0 - min(min(g.x, g.y), 1.0);
}

void main() {
    vec2 p = v_world_pos.xz;

    vec3 col = FLOOR;
    col = mix(col, MINOR, gridLine(p, 0.1) * 0.6);
    col = mix(col, MAJOR, gridLine(p, 1.0) * 0.8);

    // Coloured axes through the origin (Isaac-style)
    vec2 aw = fwidth(p); // High multiplier to make the axes more visible -> 
    if (abs(v_world_pos.z) < aw.y) { col = mix(col, vec3(0.55, 0.15, 0.15), 0.75); }  // X axis
    if (abs(v_world_pos.x) < aw.x) { col = mix(col, vec3(0.15, 0.45, 0.20), 0.75); }  // Z axis

    // Fade the plane into the background = fake horizon
    float d = length(v_world_pos - cam.cam_pos.xyz);
    col = mix(col, BG, smoothstep(30.0, 90.0, d));

    o_colour = vec4(col, 1.0);
}