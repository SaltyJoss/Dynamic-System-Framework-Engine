#version 460 core

out vec2 uv;

// Fullscreen triangle vertices
const vec2 verts[3] = vec2[3](
    vec2(-1.0, -1.0),
    vec2( 3.0, -1.0),
    vec2(-1.0,  3.0)
);

// Main vertex shader entry point
void main() {
    vec2 position = verts[gl_VertexID];
    uv = position * 0.5 + 0.5;
    gl_Position = vec4(position, 0.0, 1.0);
}