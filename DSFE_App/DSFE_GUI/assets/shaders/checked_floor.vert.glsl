#version 460 core

out vec3 WorldPos;
out float vViewZ;
out vec4 vScreenPos;

uniform mat4 gVP = mat4(1.0);
uniform mat4 gView;

uniform float gGridSize = 2500.0;
uniform vec3 gCameraWorldPos;

uniform vec4 uClipPlane;

const vec3 Pos[4] = vec3[4](
	vec3(-0.5, 0.0, -1.0),
	vec3( 0.5, 0.0,	-1.0),
	vec3( 0.5, 0.0,  1.0),
	vec3(-0.5, 0.0,  1.0)
);

const int Indices[6] = int[6](0, 1, 2, 0, 2, 3);

void main()
{
	int Index = Indices[gl_VertexID];
	vec3 local = Pos[Index] * gGridSize;
	vec3 vPos3 = local;

	// Snap floor center to camera XZ so it's always beneath the viewer
	// vPos3.x += gCameraWorldPos.x;
	// vPos3.z += gCameraWorldPos.z;
	vPos3.y += 0.001; // Tiny height offset to prevent z-fighting with raw zero planes

	WorldPos = vPos3;

	// Recompute view Z for the horizon distance fade check
	vec4 viewPos = gView * vec4(vPos3, 1.0);
	vViewZ = -viewPos.z;

	gl_ClipDistance[0] = dot(vec4(WorldPos, 1.0), uClipPlane);
    
	gl_Position = gVP * vec4(vPos3, 1.0);
    vScreenPos = gl_Position;
}
