#version 460 core

in vec2 uv;
out vec4 FragColour;

uniform sampler2D hdrScene;
uniform float exposure = 1.0;

vec3 tonemapReinhard(vec3 x) { return x / (x + vec3(1.0)); }

void main()
{
    vec3 col = texture(hdrScene, uv).rgb;
    col *= exposure;
    col = tonemapReinhard(col);
    col = pow(col, vec3(1.0/2.2)); // Gamma to sRGB
    FragColour = vec4(col, 1.0);
    //  FragColour = vec4(1, 0, 1, 1); -> debugging 
}