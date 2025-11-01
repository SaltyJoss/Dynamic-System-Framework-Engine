#version 330

out vec4 FragColour;
in vec3 FragPos;   // world-space from vertex shader
in vec2 TexCoords;

uniform float scale = 40.0;
uniform vec3 colour1 = vec3(0.95);
uniform vec3 colour2 = vec3(0.05);

void main() {
    float pattern = mod(floor(FragPos.x * 0.5) + floor(FragPos.z * 0.5), 2.0);
    vec3 col = mix(colour1, colour2, pattern);
    FragColour = vec4(col, 1.0);
}