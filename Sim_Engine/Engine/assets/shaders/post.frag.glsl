#version 460 core

in vec2 uv;
out vec4 FragColour;

uniform sampler2D hdrScene;
uniform sampler2D ssaoTex;
uniform bool ssaoEnabled;
uniform vec2 uRes;
uniform float exposure = 1.0;
uniform float whitePoint;

vec3 tonemapReinhard(vec3 x) { return x / (x + vec3(1.0)); }

vec3 tonemapACES(vec3 x) {
    const float a = 2.51;
    const float b = 0.03;
    const float c = 2.43;
    const float d = 0.59;
    const float e = 0.14;
    return clamp((x*(a*x + b)) / (x*(c*x + d) + e), 0.0, 1.0);
}

vec3 tonemapSample(vec3 hdr) {
    hdr *= exposure;
    hdr /= max(whitePoint, 1e-4);
    return tonemapACES(hdr);
}

// FXAA implementation
vec3 fxaaTonemapped(sampler2D tex, vec2 uv, vec2 res)
{
    // Calculate pixel size
    vec2 px = 1.0 / res;

    // Sample the surrounding pixels
    vec3 rgbNW = tonemapSample(texture(tex, uv + vec2(-px.x, -px.y)).rgb);
    vec3 rgbNE = tonemapSample(texture(tex, uv + vec2( px.x, -px.y)).rgb);
    vec3 rgbSW = tonemapSample(texture(tex, uv + vec2(-px.x,  px.y)).rgb);
    vec3 rgbSE = tonemapSample(texture(tex, uv + vec2( px.x,  px.y)).rgb);
    vec3 rgbM  = tonemapSample(texture(tex, uv).rgb);

    // Compute luminance
    vec3 lumaW = vec3(0.299, 0.587, 0.114);
    float lumaNW = dot(rgbNW, lumaW);
    float lumaNE = dot(rgbNE, lumaW);
    float lumaSW = dot(rgbSW, lumaW);
    float lumaSE = dot(rgbSE, lumaW);
    float lumaM  = dot(rgbM,  lumaW);

    // Compute edge detection
    float lumaMin = min(lumaM, min(min(lumaNW, lumaNE), min(lumaSW, lumaSE)));
    float lumaMax = max(lumaM, max(max(lumaNW, lumaNE), max(lumaSW, lumaSE)));

    // Early exit if no edge
    vec2 dir;
    dir.x = -((lumaNW + lumaNE) - (lumaSW + lumaSE));
    dir.y =  ((lumaNW + lumaSW) - (lumaNE + lumaSE));

    // Normalize direction
    float dirReduce = max((lumaNW + lumaNE + lumaSW + lumaSE) * (0.25 * 0.5), 1.0 / 128.0);
    float rcpDirMin = 1.0 / (min(abs(dir.x), abs(dir.y)) + dirReduce);

    dir = clamp(dir * rcpDirMin * 1.0, vec2(-8.0), vec2(8.0)) * px;

    // Sample along the edge direction
    vec3 rgbA = 0.5 * (
        tonemapSample(texture(tex, uv + dir * (1.0 / 3.0 - 0.5)).rgb) +
        tonemapSample(texture(tex, uv + dir * (2.0 / 3.0 - 0.5)).rgb)
    );

    vec3 rgbB = rgbA * 0.5 + 0.25 * (
        tonemapSample(texture(tex, uv + dir * -0.5).rgb) +
        tonemapSample(texture(tex, uv + dir *  0.5).rgb)
    );

    // Choose final color based on luminance
    float lumaB = dot(rgbB, lumaW);
    if ((lumaB < lumaMin) || (lumaB > lumaMax)) { return rgbA; }
    return rgbB;

}

// Main fragment shader entry point
void main() {
    // FXAA on tonemapped LDR (linear)
    vec3 ldrAA = fxaaTonemapped(hdrScene, uv, uRes);

    // Apply SSAO softly: lerp between full color and AO-darkened color
    if (ssaoEnabled) {
        float ao = texture(ssaoTex, uv).r;
        // Soften: don't let AO go below 0.3 to avoid crushing blacks
        ao = mix(1.0, ao, 0.6); // 0.6 = blend factor, lower = subtler
        ldrAA *= ao;
    }

    // Gamma last
    vec3 srgb = pow(ldrAA, vec3(1.0/2.2));
    FragColour = vec4(srgb, 1.0);
}