#version 460 core

in vec2 GridXZ;
in vec3 WorldPos;

layout(location = 0) out vec4 FragColour;

uniform float gGridMinPixelsBetweenCells = 2.0;
uniform float gGridCellSize = 0.1;
uniform float gGridSize = 1000.0;

uniform vec3 gCameraWorldPos;
uniform vec4 gGridColourThin = vec4(0.55, 0.55, 0.55, 1.0);
uniform vec4 gGridColourThick = vec4(0.15, 0.15, 0.15, 1.0);

// Compute world units per pixel at given world XZ position
float worldUnitsPerPixel(vec2 worldXZ)
{
    float dx = length(vec2(dFdx(worldXZ.x), dFdy(worldXZ.x)));
    float dz = length(vec2(dFdx(worldXZ.y), dFdy(worldXZ.y)));
    return max(max(dx, dz), 1e-6);
}

// Anti-aliased grid line computation
float gridAA(vec2 worldXZ, float spacing)
{
    vec2 p = worldXZ / spacing;
    vec2 g = abs(fract(p - 0.5) - 0.5) / fwidth(p);
    float line = 1.0 - clamp(min(g.x, g.y), 0.0, 1.0);
    return line;
}

// Compute visibility based on spacing and world units per pixel
float visibility(float spacing, float wupp)
{
    float ratio = (wupp * gGridMinPixelsBetweenCells) / spacing;
    return 1.0 - smoothstep(1.0, 2.0, ratio);
}

// Main fragment shader entry point
void main() {
    // Pattern uses GridXZ
    vec2 xz = GridXZ;

    float s0 = gGridCellSize;
    float s1 = gGridCellSize * 10.0;
    float s2 = gGridCellSize * 100.0;

    float a0 = gridAA(xz, s0);
    float a1 = gridAA(xz, s1);
    float a2 = gridAA(xz, s2);

    // Axes should be in WORLD space, not local pattern space
    vec2 worldXZ = WorldPos.xz;
    float w = worldUnitsPerPixel(worldXZ);
    float axisWidth = clamp(w * 2.0, 0.00005, gGridCellSize * 5.0);

    float xAxis = 1.0 - smoothstep(0.0, w * 2.0, abs(worldXZ.y));
    float zAxis = 1.0 - smoothstep(0.0, w * 2.0, abs(worldXZ.x));

    // Fade by distance in WORLD space
    float d = length(worldXZ - gCameraWorldPos.xz);
    float fade = exp(-d / (gGridSize * 0.35));

    xAxis *= fade;
    zAxis *= fade;

    float majorMask = max(a1, a2); // major grid lines
    vec3 rgb = mix(gGridColourThin.rgb, gGridColourThick.rgb, majorMask); // base grid color

    // axis tint
    rgb = mix(rgb, vec3(0.75, 0.25, 0.25), xAxis * 0.6);
    rgb = mix(rgb, vec3(0.25, 0.45, 0.75), zAxis * 0.6);

    // Smooth alpha (NO DISCARD)
    float alpha = (0.25 * a0 + 0.45 * a1 + 0.65 * a2); // base alpha from grid lines
    alpha = max(alpha, 0.8 * max(xAxis, zAxis));    // ensure axes are more visible
    alpha = clamp(alpha, 0.0, 1.0) * fade;  // fade by distance

    FragColour = vec4(rgb, alpha);
}
