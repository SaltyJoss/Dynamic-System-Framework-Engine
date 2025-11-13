#version 460 core

in vec3 WorldDir;
out vec4 FragColour;

uniform samplerCube equirectMap;
uniform float roughness;

const float PI = 3.14159265359;

vec3 ImportanceSampleGGX(vec2 Xi, vec3 N, float roughness);

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
			prefilteredColour += texture(equirectMap, L).rgb * NdotL;
			totalWeight += NdotL;
		}
	}
	prefilteredColour = prefilteredColour / totalWeight;
	FragColour = vec4(prefilteredColour, 1.0);
}