#version 460 core

in vec3 TexCoords;
out vec4 FragColour;

uniform samplerCube environmentMap;

void main()
{
    FragColour = texture(environmentMap, TexCoords);
}