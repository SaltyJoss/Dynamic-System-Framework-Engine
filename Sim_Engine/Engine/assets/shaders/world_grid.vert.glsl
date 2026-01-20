#version 460 core

out vec2 GridXZ;
out vec3 WorldPos;
out float vViewZ;

uniform mat4 gVP = mat4(1.0);
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

//	vPos3.x += gCameraWorldPos.x;
//	vPos3.z += gCameraWorldPos.z;
	vPos3.y += gGridY + gGridYOffset;

	gl_Position = gVP * vec4(vPos3, 1.0);
	WorldPos = vPos3;
	GridXZ = local.xz;
    vViewZ = -gl_Position.z; // positive forward for a standard RH view where camera looks -Z
}