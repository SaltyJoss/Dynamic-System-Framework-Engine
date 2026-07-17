struct VSOutput {
    float4 positions : SV_POSITION;
    float3 colour : COLOR0;
};

static const float2 positions[3] = {
    float2( 0.0f, -0.5f),
    float2( 0.5f,  0.5f),
    float2(-0.5f,  0.5f)
};

static const float3 colours[3] = {
    float3(1.0f, 0.0f, 0.0f),
    float3(0.0f, 1.0f, 0.0f),
    float3(0.0f, 0.0f, 1.0f)  
};

VSOutput VSMain(uint vertex_idx : SV_VertexID) {
    VSOutput output;
    output.positions = float4(positions[vertex_idx], 0.0f, 1.0f);
    output.colour = colours[vertex_idx];
    return output;
}

float4 PSMain(VSOutput input) : SV_TARGET {
    return float4(input.colour, 1.0f);
}