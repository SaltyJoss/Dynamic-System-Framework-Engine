#version 450

layout(location = 0) in vec3 v_world_pos;

layout(set = 0, binding = 0) uniform Camera {
    mat4 view;
    mat4 proj;
    mat4 light_space;
    vec4 cam_pos;
} cam;

layout(set = 0, binding = 1) uniform sampler2DShadow shadow_map;

layout(location = 0) out vec4 o_colour;

// Poisson disk offsets for PCF sampling of the shadow map
const vec2 poissonDisk[12] = vec2[](
    vec2(-0.326212, -0.405805), vec2(-0.840144, -0.07358),
    vec2(-0.695914,  0.457137), vec2(-0.203345,  0.620716),
    vec2( 0.96234,  -0.194983), vec2( 0.473434, -0.480026),
    vec2( 0.519456,  0.767022), vec2( 0.185461, -0.893124),
    vec2( 0.507431,  0.064425), vec2( 0.89642,   0.412458),
    vec2(-0.32194,  -0.932615), vec2(-0.791559, -0.597705)
);

const vec3 BG    = vec3(0.02, 0.02, 0.025);  // MUST match the clear colour
const vec3 FLOOR = vec3(0.11, 0.11, 0.12);
const vec3 MINOR = vec3(0.16, 0.16, 0.17);
const vec3 MAJOR = vec3(0.21, 0.21, 0.23);

float computeShadow(vec3 world_pos, vec3 N, vec3 L) {
    vec4 lsp = cam.light_space * vec4(world_pos, 1.0);
    vec3 proj_coords = lsp.xyz / lsp.w;
    proj_coords.xy = proj_coords.xy * 0.5 + 0.5;
    if (proj_coords.z > 1.0) { return 0.0; }

    float bias = max(0.003 * (1.0 - max(dot(N, L), 0.0)), 0.0005);
    float depth = proj_coords.z - bias;
    vec2 texel = 1.0 / vec2(textureSize(shadow_map, 0));

    float lit = 0.0;
    for (int i = 0; i < 12; ++i) {
        lit += texture(shadow_map, vec3(proj_coords.xy + poissonDisk[i] * texel * 2.5, depth));
    }
    return 1.0 - (lit / 12.0);
}

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

    vec3 L = -normalize(vec3(-0.4, -1.0, -0.3));
    float sh = computeShadow(v_world_pos, vec3(0.0, 1.0, 0.0), L);
    col *= (1.0 - sh * 0.55); // Soft shadows for the grid plane, don't darken the axes too much

    // Fade the plane into the background = fake horizon
    float d = length(v_world_pos - cam.cam_pos.xyz);
    col = mix(col, BG, smoothstep(30.0, 90.0, d));

    o_colour = vec4(col, 1.0);
}