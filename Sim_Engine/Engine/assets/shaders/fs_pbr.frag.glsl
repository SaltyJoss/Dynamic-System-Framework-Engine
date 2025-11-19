#version 460 core

// ------------------------ OUTPUTS & INPUTS ------------------------

out vec4 FragColour;

in vec3 WorldPos;
in vec3 Normal;
in vec2 TexCoords;

// ------------------------ UNIFORMS ------------------------

// material parameters
uniform vec3 albedo;
uniform float metallic;
uniform float roughness;
uniform float ao;

uniform bool useTexture;
uniform sampler2D albedoTex;

// Textures
uniform sampler2D dayTex;
uniform sampler2D nightTex;
uniform sampler2D cloudTex;
uniform sampler2D normalTex;
uniform sampler2D specularTex;

// CheckerBoard Plane parameters
uniform float checkSize;
uniform vec3 colour1;
uniform vec3 colour2;
uniform bool isFloor; // true for the checker floor, false for objects

// lights
uniform vec3 lightPosition;
uniform vec3 lightDirection;
uniform vec3 lightColour;
uniform float lightSize;
uniform float lightIntensity;

// Shadow map (Cascaded)
uniform sampler2DShadow cascadeShadowMap[2];
uniform mat4 lightSpaceMatrix[2];
uniform float cascadeSplits[2];
uniform vec3 camPos;

// IBL Maps
uniform samplerCube irradianceMap; // diffuse
uniform samplerCube prefilterMap;  // specular
uniform sampler2D brdfLUT;         // BRDF lookup texture

// ------------------------ MAIN FUNCTION ------------------------

void main()
{
	vec3 L1 = normalize(lightPosition - WorldPos);
	FragColour = vec4(normalize(L1) * 0.5 + 0.5, 1.0);
}

