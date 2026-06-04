#version 460 core

in vec2 uv;

out vec4 FragColour;

uniform sampler2D screenTexture;

void main() {
    FragColour = vec4(1,0,0,1);
}