#version 460 core

in vec3 WorldDir;
out vec4 FragColour;

uniform sampler2D equirectMap;

const float PI = 3.14159265359;

void main()
{
	vec3 N = normalize(WorldDir);

	// Convert cubemap direction to spherical coordinates
	float theta = acos(clamp(N.y, -1.0, 1.0)); // polar angle
	float phi = atan(N.z, N.x);         // azimuthal angle
	if (phi < 0.0)
		phi += 2.0 * PI;
	
	// Map spherical coordinates to equirectangular UVs
	float u = phi / (2.0 * PI);
	float v = 1.0 - (theta / PI);

	vec3 colour = texture(equirectMap, vec2(u, v)).rgb;
	FragColour = vec4(colour, 1.0);
}