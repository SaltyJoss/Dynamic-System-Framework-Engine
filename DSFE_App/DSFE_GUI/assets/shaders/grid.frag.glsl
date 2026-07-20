#version 450

layout(location = 0) in vec3 v_world_pos;

layout(set = 0, binding = 0) uniform Camera {
    mat4 view;
    mat4 proj;
    vec4 cam_pos;
} cam;

layout(location = 0) out vec4 o_colour;

// Antialiased grid line intensity for a given cell size
float gridLine(vec2 p, float cell) {
    vec2 q = p / cell;
    vec2 g = abs(fract(q - 0.5) - 0.5) / fwidth(q);
    return 1.0 - min(min(g.x, g.y), 1.0);
}

void main() {
    vec2 p = v_world_pos.xz;

    float minor = gridLine(p, 0.1) * 0.25;   // 10cm lines, faint
    float major = gridLine(p, 1.0) * 0.5;    // 1m lines, stronger
    float axis  = 0.0;
    // X/Z axes highlighted
    vec2 aw = fwidth(p);
    if (abs(v_world_pos.z) < aw.y * 1.5) { axis = 0.8; }   // X axis line
    if (abs(v_world_pos.x) < aw.x * 1.5) { axis = 0.8; }   // Z axis line

    float line = max(max(minor, major), axis);

    // Distance fade from the camera
    float d = length(v_world_pos - cam.cam_pos.xyz);
    float fade = 1.0 - smoothstep(20.0, 80.0, d);

    vec3 col = mix(vec3(0.35), vec3(0.7), axis);   // axes slightly brighter
    float alpha = line * fade;
    if (alpha < 0.01) { discard; }
    o_colour = vec4(col, alpha);
}