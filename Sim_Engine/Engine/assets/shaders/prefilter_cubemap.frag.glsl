#version 460 core

in vec3 WorldDir;
out vec4 FragColour;

uniform samplerCube environmentMap;
uniform float roughness;

const float PI = 3.14159265359;

vec3 ImportanceSampleGGX(vec2 Xi, vec3 N, float roughness)
{
	float a = roughness * roughness;

	float phi = 2.0 * PI * Xi.x;
	float cosTheta = sqrt((1.0 - Xi.y) / (1.0 + (a * a - 1.0) * Xi.y));
	float sinTheta = sqrt(1.0 - cosTheta * cosTheta);

	// Sample in tangent space
	vec3 H;
	H.x = cos(phi) * sinTheta;
	H.y = sin(phi) * sinTheta;
	H.z = cosTheta;

	// Tangent space -> world space
	vec3 up = abs(N.z) < 0.999 ? vec3(0.0, 0.0, 1.0)
		: vec3(1.0, 0.0, 0.0);
	vec3 tangent = normalize(cross(up, N));
	vec3 bitangent = cross(N, tangent);

	vec3 sampleVec = tangent * H.x + bitangent * H.y + N * H.z;
	return normalize(sampleVec);
}

void main()
{
	vec3 N = normalize(WorldDir);
	vec3 R = N;
	vec3 V = R;

	const uint SAMPLE_COUNT = 1024u;
	vec3 prefilteredColour = vec3(0.0);
	float totalWeight = 0.0;
	
	for(uint i = 0u; i < SAMPLE_COUNT; ++i)
	{
		vec2 Xi = vec2(float(i)/float(SAMPLE_COUNT), fract(sin(i) * 43758.543123));
		vec3 H = ImportanceSampleGGX(Xi, N, roughness);
		vec3 L = normalize(2.0 * dot(V, H) * H - V);
		
		float NdotL = max(dot(N, L), 0.0);
		if(NdotL > 0.0)
		{
			prefilteredColour += texture(environmentMap, L).rgb * NdotL;
			totalWeight += NdotL;
		}
	}
	prefilteredColour = prefilteredColour / totalWeight;
	FragColour = vec4(prefilteredColour, 1.0);
}