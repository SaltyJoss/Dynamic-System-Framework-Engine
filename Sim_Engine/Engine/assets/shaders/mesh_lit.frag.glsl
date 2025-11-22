#version 460 core

in vec3 WorldPos;
in vec3 Normal;
in vec2 TexCoords;

out vec4 FragColour;

// Basic material
uniform vec3 albedo;

// Simple point or directional light approximated by a position
uniform vec3 lightPosition;   // world-space
uniform vec3 lightColour;     // usually vec3(1.0)
uniform float lightIntensity; // e.g. 1.0–5.0

// Camera, for specular
uniform vec3 camPos;

void main()
{
    // 1. Normal, view, light vectors
    vec3 N = normalize(Normal);
    vec3 L = normalize(lightPosition - WorldPos);
    vec3 V = normalize(camPos - WorldPos);

    // 2. Diffuse term (Lambert)
    float NdotL = max(dot(N, L), 0.0);
    vec3 diffuse = albedo * lightColour * NdotL * lightIntensity;

    // 3. Specular term (Blinn–Phong, cheap)
    vec3 H = normalize(V + L);
    float spec = pow(max(dot(N, H), 0.0), 16.0);   // shininess = 16
    vec3 specular = 0.2 * spec * lightColour;      // 0.2 = specular strength

    // 4. Ambient term (constant)
    vec3 ambient = 0.1 * albedo;

    // 5. Final colour
    vec3 colour = ambient + diffuse + specular;
    FragColour = vec4(colour, 1.0);
}

/*
BREAKDOWN FOR MY OWN SANTIY (tutorials linked with resources, gpt helped compile info):

Inputs
-------
- in vec3 WorldPos: fragment position in world space (from vs_pbr.vert.glsl)
- in vec3 Normal:   interpolated normal in world space
- in vec2 TexCoords: available if you later want textures (unused here)

Uniforms
--------
- albedo:          base RGB colour of the material (no textures here)
- lightPosition:   world-space position of a point-like light source
- lightColour:     RGB intensity of the light (1,1,1 = white)
- lightIntensity:  scalar multiplier for light strength
- camPos:          camera position in world space (for specular highlights)

Main steps
-----------
1. Reconstruct N, L, V:
   - N = normalised surface normal
   - L = direction from surface to light
   - V = direction from surface to camera

2. Diffuse:
   - NdotL = max(dot(N, L), 0)
   - diffuse = albedo * lightColour * NdotL * lightIntensity

3. Specular:
   - H = normalise(V + L) (Blinn half-vector)
   - spec  = max(dot(N, H), 0)^16  (shininess exponent)
   - specular = 0.2 * spec * lightColour

4. Ambient:
   - ambient = 0.1 * albedo (constant fake GI)

5. Combine:
   - colour = ambient + diffuse + specular
   - FragColour = vec4(colour, 1)
*/
