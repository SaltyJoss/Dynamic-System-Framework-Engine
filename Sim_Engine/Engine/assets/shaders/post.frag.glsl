#version 460 core

in vec2 uv;
out vec4 FragColour;

uniform sampler2D hdrScene;
uniform vec2 uRes;
uniform float exposure = 1.0;

vec3 tonemapReinhard(vec3 x) { return x / (x + vec3(1.0)); }

// FXAA implementation
vec3 fxaa(sampler2D tex, vec2 uv, vec2 res)
{
    // Calculate pixel size
    vec2 px = 1.0 / res;

    // Sample the surrounding pixels
    vec3 rgbNW = texture(tex, uv + vec2(-px.x, -px.y)).rgb;
    vec3 rgbNE = texture(tex, uv + vec2( px.x, -px.y)).rgb;
    vec3 rgbSW = texture(tex, uv + vec2(-px.x,  px.y)).rgb;
    vec3 rgbSE = texture(tex, uv + vec2( px.x,  px.y)).rgb;
    vec3 rgbM  = texture(tex, uv).rgb;

    // Compute luminance
    vec3 luma = vec3(0.299, 0.587, 0.114);
    float lumaNW = dot(rgbNW, luma);
    float lumaNE = dot(rgbNE, luma);
    float lumaSW = dot(rgbSW, luma);
    float lumaSE = dot(rgbSE, luma);
    float lumaM  = dot(rgbM,  luma);

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
        texture(tex, uv + dir * (1.0 / 3.0 - 0.5)).rgb +
        texture(tex, uv + dir * (2.0 / 3.0 - 0.5)).rgb);

    vec3 rgbB = rgbA * 0.5 + 0.25 * (
        texture(tex, uv + dir * -0.5).rgb +
        texture(tex, uv + dir * 0.5).rgb);

    // Choose final color based on luminance
    float lumaB = dot(rgbB, luma);
    if ((lumaB < lumaMin) || (lumaB > lumaMax)) { return rgbA; }
    return rgbB;

}

// Main fragment shader entry point
void main()
{
    vec3 col = fxaa(hdrScene, uv, uRes);
    col *= exposure;
    
    col = tonemapReinhard(col);

    col = pow(col, vec3(1.0/2.2)); // Gamma to sRGB
    FragColour = vec4(col, 1.0);

    //  FragColour = vec4(1, 0, 1, 1); -> debugging 
}