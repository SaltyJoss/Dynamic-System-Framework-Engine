#version 460 core

layout(location = 0) in vec3 aPos;
out vec3 WorldDir;

uniform mat4 vp;

void main()
{
	WorldDir = aPos;
	gl_Position = vp * vec4(aPos, 1.0);
}