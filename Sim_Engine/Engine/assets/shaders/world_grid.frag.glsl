#version 460 core

in vec3 WorldPos;

layout(location = 0) out vec4 FragColour;

uniform float gGridMinPixelsBetweenCells = 2.0;
uniform float gGridCellSize = 0.25;
uniform float gGridSize = 1000.0;

uniform vec3 gCameraWorldPos;
uniform vec4 gGridColourThin = vec4(0.55, 0.55, 0.55, 1.0);
uniform vec4 gGridColourThick = vec4(0.15, 0.15, 0.15, 1.0);

float worldUnitsPerPixel(vec2 worldXZ)
{
    float dx = length(vec2(dFdx(worldXZ.x), dFdy(worldXZ.x)));
    float dz = length(vec2(dFdx(worldXZ.y), dFdy(worldXZ.y)));
    return max(max(dx, dz), 1e-6);
}

float gridAA(vec2 worldXZ, float spacing)
{
    vec2 p = worldXZ / spacing;
    vec2 g = abs(fract(p - 0.5) - 0.5) / fwidth(p);
    float line = 1.0 - clamp(min(g.x, g.y), 0.0, 1.0);
    return line;
}

float visibility(float spacing, float wupp)
{
    float ratio = (wupp * gGridMinPixelsBetweenCells) / spacing;
    return 1.0 - smoothstep(1.0, 2.0, ratio);
}

void main() {
    vec2 xz = WorldPos.xz;
    float wupp = worldUnitsPerPixel(xz);

    float s0 = gGridCellSize;
    float s1 = gGridCellSize * 10.0;
    float s2 = gGridCellSize * 100.0;

    float a0 = gridAA(xz, s0) * visibility(s0, wupp);
    float a1 = gridAA(xz, s1) * visibility(s1, wupp);
    float a2 = gridAA(xz, s2) * visibility(s2, wupp);

    float xAxis = 1.0 - smoothstep(0.0, wupp * 2.0, abs(xz.y));
    float zAxis = 1.0 - smoothstep(0.0, wupp * 2.0, abs(xz.x));

    float d = length(xz - gCameraWorldPos.xz);
    float fade = exp(-d / (gGridSize * 0.35));

    float majorMask = max(a1, a2);
    vec3 rgb = mix(gGridColourThin.rgb, gGridColourThick.rgb, majorMask);

    rgb = mix(rgb, vec3(0.75, 0.25, 0.25), xAxis * 0.5);
    rgb = mix(rgb, vec3(0.25, 0.45, 0.75), zAxis * 0.5);

    float alpha = 0.35 * a0 + 0.75 * a1 + 1.00 * a2;
    alpha = max(alpha, 0.65 * max(xAxis, zAxis));
    alpha = clamp(alpha, 0.0, 1.0) * fade;

    FragColour = vec4(rgb, alpha);
}