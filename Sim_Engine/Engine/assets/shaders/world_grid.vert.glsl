#version 460 core

out vec3 WorldPos;
out float vViewZ;

uniform mat4 gVP = mat4(1.0);
uniform mat4 gView;

uniform float gGridSize = 1000.0;
uniform vec3 gCameraWorldPos;

uniform float gGridY = 0.0;
uniform float gGridYOffset = 0.001;

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

	// Snap grid center to camera XZ so it's always beneath the viewer
	vPos3.x += gCameraWorldPos.x;
	vPos3.z += gCameraWorldPos.z;

	vPos3.y += gGridY + gGridYOffset;

	// Output
	WorldPos = vPos3;

	// Recompute view Z for fragment shader
	vec4 viewPos = gView * vec4(vPos3, 1.0);
	vViewZ = -viewPos.z;
	gl_Position = gVP * vec4(vPos3, 1.0);
}