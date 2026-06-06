#version 460 core

in vec2 uv;

out vec4 FragColour;

uniform sampler2D screenTexture;

void main() {
    FragColour = texture(screenTexture, uv);
}