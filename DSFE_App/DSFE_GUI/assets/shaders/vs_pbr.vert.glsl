#version 460 core

layout(location = 0) in vec3 aPosition;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec2 aTexCoord;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
uniform vec3 colour;
uniform vec4 uClipPlane;

out vec3 WorldPos;
out vec3 Normal;
out vec2 TexCoords;
out vec3 Colour;
out float gl_ClipDistance[1];

void main()
{
  Colour = colour;
  mat3 normalMatrix = transpose(inverse(mat3(model)));
  WorldPos = vec3(model * vec4(aPosition, 1.0));
  Normal   = normalize(normalMatrix * aNormal);
  TexCoords = aTexCoord;

  gl_ClipDistance[0] = dot(vec4(WorldPos, 1.0), uClipPlane);

  gl_Position = projection * view * vec4(WorldPos, 1.0);
}