#version 460 core

in vec2 GridXZ;
in vec3 WorldPos;
in float vViewZ;

layout(location = 0) out vec4 FragColour;

uniform float gGridMinPixelsBetweenCells = 2.0;
uniform float gGridCellSize = 0.1;
uniform float gGridSize = 2500.0;
uniform float gRenderScale = 1.0; // 0.75 on low, 1.0 at medium, 1.25 on high/ultra

uniform vec3 gCameraWorldPos;
uniform vec4 gGridColourThin = vec4(0.55, 0.55, 0.55, 1.0);
uniform vec4 gGridColourThick = vec4(0.15, 0.15, 0.15, 1.0);
uniform float uRenderScale; // 1.0 at native, 0.75 on low, etc.

// Compute world units per pixel at given world XZ position
float worldUnitsPerPixel(vec2 worldXZ)
{
    float dx = length(vec2(dFdx(worldXZ.x), dFdy(worldXZ.x)));
    float dz = length(vec2(dFdx(worldXZ.y), dFdy(worldXZ.y)));
    return max(max(dx, dz), 1e-6);
}

// Compute how many pixels correspond to a cell size at the current fragment
float pixelsPerCell(float cellSize)
{
    float dx = length(dFdx(WorldPos.xz));
    float dy = length(dFdy(WorldPos.xz));
    float worldPerPixel = max(dx, dy);
    return cellSize / worldPerPixel;
}

// Anti-aliased grid line computation
float gridAA(vec2 worldXZ, float spacing)
{
    vec2 p = worldXZ / spacing;
    vec2 fw = fwidth(p);
    vec2 g = abs(fract(p - 0.5) - 0.5) / fw;
    float line = 1.0 - clamp(min(g.x, g.y), 0.0, 1.0);
    return line;
}

// Main fragment shader entry point
void main() {
    // Pattern uses GridXZ
    vec2 xz = GridXZ;

    float s0 = gGridCellSize;
    float s1 = gGridCellSize * 10.0;
    float s2 = gGridCellSize * 100.0;

        // Compute grid line alpha for each scale
    float p0 = pixelsPerCell(s0);
    float p1 = pixelsPerCell(s1);
    float p2 = pixelsPerCell(s2);

    const float MIN_PIXELS = 1.5; // tune: 1.0–1.5

    if (p0 < MIN_PIXELS && p1 < MIN_PIXELS && p2 < MIN_PIXELS) { discard; }

    float a0 = gridAA(xz, s0);
    float a1 = gridAA(xz, s1);
    float a2 = gridAA(xz, s2);

    // Combine contributions
    float a = 0.0;
    float weight = 0.0;

    // Axes should be in WORLD space, not local pattern space
    vec2 worldXZ = WorldPos.xz;
    float w = worldUnitsPerPixel(worldXZ);
    float axisWidth = clamp(w * 2.0, 0.00005, gGridCellSize * 5.0);

       // Shimmer control
    const float MIN_PX = 2.5; // higher = less shimmer

    // screen-space AA gate (this is the important bit)
    float px = pixelsPerCell(gGridCellSize);
    float vis = smoothstep(1.0, 2.0, px); // first args: tune for fade-in, second args: tune for full visibility

    float hold = 200.0;         // meters: no fade before this
    float falloff = 1800.0;     // meters: how slowly it fades after hold

    float t = max(vViewZ - hold, 0.0);
    float fade = exp(-t / falloff);
    fade *= 1.0 - smoothstep(300.0, 900.0, vViewZ);

    if (p0 >= MIN_PX) { a = a0; weight = 0.25; } // thin grid
    else if (p1 >= MIN_PX) { a = a1; weight = 0.35; } // medium grid
    else if (p2 >= MIN_PX) { a = a2; weight = 0.55; } // thick grid
    else { a = 0.0; weight = 0.0; }

    // Final colour
    float majorMask = 0.0;
    if (p1 >= MIN_PX) majorMask = 0.6;
    if (p2 >= MIN_PX) majorMask = 1.0;
    vec3 rgb = mix(gGridColourThin.rgb, gGridColourThick.rgb, majorMask); // base grid color

    // Smooth alpha (NO DISCARD)
    float alpha = (0.35 * a0 + 0.5 * a1 + 0.7 * a2); // base alpha from grid lines
    alpha *= (vis * fade);
    if (alpha < (0.1 / 2500.0)) discard;

    FragColour = vec4(rgb * alpha, alpha);
}