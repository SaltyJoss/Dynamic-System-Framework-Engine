#version 460 core

in vec3 WorldDir;
out vec4 FragColour;

uniform samplerCube equirectMap;

const float PI = 3.14159265359;

void main()
{
	vec3 N = normalize(WorldDir);
	
	// Build tangent basis around N
	vec3 up = abs(N.y) < 0.999 ? vec3(0.0, 1.0, 0.0) : vec3(1.0, 0.0, 0.0);
	vec3 tangent = normalize(cross(up, N));
	vec3 bitangent = cross(N, tangent);

	const uint SAMPLE_COUNT = 1024;
	vec3 irradiance = vec3(0.0);

	for (uint i = 0u; i < SAMPLE_COUNT; ++i)
	{
		float Xi1 = float(i) / float(SAMPLE_COUNT);
		float Xi2 = fract(sin(float(i) * 12.9898) * 43758.5453);

		vec2 Xi = vec2(Xi1, Xi2);

		// Spherical coords on hemisphere
		float phi = 2.0 * PI * Xi.x;
		float cosTheta = sqrt(1.0 - Xi.y);
		float sinTheta = sqrt(1.0 - cosTheta * cosTheta);

		// Sample in tangent space
		vec3 H = vec3(
			cos(phi) * sinTheta,
			sin(phi) * sinTheta,
			cosTheta
		);

		// Rotate H to world space
		vec3 L = normalize(tangent * H.x + bitangent * H.y + N * H.z);
		float NdotL = max(dot(N, L), 0.0);

		if (NdotL > 0.0)
		{
			vec3 sampleColor = texture(equirectMap, L).rgb;
			// Lambertian Integrand
			irradiance += sampleColor * NdotL;
		}
	}

	irradiance = PI * irradiance / float(SAMPLE_COUNT);
	FragColour = vec4(irradiance, 1.0);
}

