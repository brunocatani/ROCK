#pragma once

namespace rock::debug_overlay_shaders
{
    inline constexpr char kInstancedBodyVertex[] = R"(
struct VS_INPUT {
    float3 vPos : POS;
    float4 modelRow0 : IROW0;
    float4 modelRow1 : IROW1;
    float4 modelRow2 : IROW2;
    float4 modelRow3 : IROW3;
    float4 color : ICOLOR0;
    uint instanceId : SV_InstanceID;
};

struct VS_OUTPUT {
    float4 vPos : SV_POSITION;
    float4 vColor : COLOR0;
    float clipDistance : SV_ClipDistance0;
    float cullDistance : SV_CullDistance0;
};

cbuffer Camera : register(b0) {
    column_major float4x4 matProjView[2];
    float4 posAdjust[2];
};

VS_OUTPUT main(VS_INPUT input) {
    const float4 eyeClipEdge[2] = { { -1, 0, 0, 1 }, { 1, 0, 0, 1 } };
    const float eyeOffsetScale[2] = { -0.5, 0.5 };
    const uint eyeIndex = input.instanceId & 1;
    row_major float4x4 matModel = float4x4(input.modelRow0, input.modelRow1, input.modelRow2, input.modelRow3);

    float4 pos = float4(input.vPos.xyz, 1.0f);
    pos = mul(pos, matModel);
    pos.xyz -= posAdjust[eyeIndex].xyz;
    pos = mul(matProjView[eyeIndex], pos);

    VS_OUTPUT output;
    output.vColor = input.color;
    output.clipDistance = dot(pos, eyeClipEdge[eyeIndex]);
    output.cullDistance = output.clipDistance;
    output.vPos = pos;
    output.vPos.x *= 0.5;
    output.vPos.x += eyeOffsetScale[eyeIndex] * output.vPos.w;
    return output;
}
)";

    inline constexpr char kStereoColorVertex[] = R"(
struct VS_INPUT {
    float3 vPos : POS;
    float4 color : COLOR0;
    uint instanceId : SV_InstanceID;
};

struct VS_OUTPUT {
    float4 vPos : SV_POSITION;
    float4 vColor : COLOR0;
    float clipDistance : SV_ClipDistance0;
    float cullDistance : SV_CullDistance0;
};

cbuffer Camera : register(b0) {
    column_major float4x4 matProjView[2];
    float4 posAdjust[2];
};

VS_OUTPUT main(VS_INPUT input) {
    const float4 eyeClipEdge[2] = { { -1, 0, 0, 1 }, { 1, 0, 0, 1 } };
    const float eyeOffsetScale[2] = { -0.5, 0.5 };
    const uint eyeIndex = input.instanceId & 1;

    float4 pos = float4(input.vPos.xyz, 1.0f);
    pos.xyz -= posAdjust[eyeIndex].xyz;
    pos = mul(matProjView[eyeIndex], pos);

    VS_OUTPUT output;
    output.vColor = input.color;
    output.clipDistance = dot(pos, eyeClipEdge[eyeIndex]);
    output.cullDistance = output.clipDistance;
    output.vPos = pos;
    output.vPos.x *= 0.5;
    output.vPos.x += eyeOffsetScale[eyeIndex] * output.vPos.w;
    return output;
}
)";

    inline constexpr char kScreenTextVertex[] = R"(
struct VS_INPUT {
    float3 vPos : POS;
    float4 color : COLOR0;
};

struct VS_OUTPUT {
    float4 vPos : SV_POSITION;
    float4 vColor : COLOR0;
};

VS_OUTPUT main(VS_INPUT input) {
    VS_OUTPUT output;
    output.vPos = float4(input.vPos.xy, 0.0f, 1.0f);
    output.vColor = input.color;
    return output;
}
)";

    inline constexpr char kPixel[] = R"(
struct PS_INPUT {
    float4 pos : SV_POSITION;
    float4 color : COLOR0;
};

float4 main(PS_INPUT input) : SV_Target {
    return input.color;
}
)";
}
