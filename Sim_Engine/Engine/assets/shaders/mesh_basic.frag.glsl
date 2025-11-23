#version 460 core

in vec3 WorldPos;
in vec3 Normal;
in vec2 TexCoords;

out vec4 FragColour;

uniform vec3 colour; // simple RGB override

void main()
{
    FragColour = vec4(colour, 1.0);
}

/*
BREAKDOWN FOR MY OWN SANITY (tutorials linked with resources, gpt helped compile info):

Purpose:
--------
A fallback visual shader. Never fails.
Used for debugging robot meshes, transforms, import orientation, etc.

Inputs:
-------
WorldPos, Normal, TexCoords (ignored)

Uniforms:
---------
color: vec3 - single flat RGB colour (e.g. vec3(1,0,0))

What it does:
-------------
- Outputs that exact colour for every pixel.
- No lighting, no PBR, no shadows, no textures.
- Always visible.

When to use:
------------
- meshes disappear under PBR
- transforms look incorrect
- shadows over-darken the scene
- IBL or LUT not bound
- debugging axes and robot FK

Switching back to this shader is step 0 whenever
visual debugging is needed.
*/
