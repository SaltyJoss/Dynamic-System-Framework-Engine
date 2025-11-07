#version 330 core

in vec3 WorldPos;

layout(location = 0) out vec4 FragColour;

uniform float gGridCellSize = 0.025;
uniform vec4 gGridColourThin = vec4(0.5, 0.5, 0.5, 1.0);
uniform vec4 gGridColourThick = vec4(0.0, 0.0, 0.0, 1.0);

void main() {
    float Lod0a = mod(WorldPos.z, gGridCellSize);

    vec4 Colour;

    Colour = gGridColourThick;
    Colour.a *= Lod0a;

    FragColour = Colour;
}