#version 460 core

in vec2 uv;
out float FragOcclusion;

uniform sampler2D ssaoInput;

void main() {
    vec2 texelSize = 1.0 / vec2(textureSize(ssaoInput, 0));
    float result = 0.0;

    // 4x4 box blur (simple and fast; bilateral not needed at low sample counts)
    for (int x = -2; x < 2; ++x) {
        for (int y = -2; y < 2; ++y) {
            vec2 offset = vec2(float(x), float(y)) * texelSize;
            result += texture(ssaoInput, uv + offset).r;
        }
    }
    FragOcclusion = result / 16.0;
}
