#include "physics-interaction/debug/DebugBodyOverlay.h"

#include <DirectXMath.h>
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <tuple>
#include <unordered_map>
#include <vector>

#include <d3d11.h>
#include <d3dcompiler.h>
#include <wrl/client.h>

#include "physics-interaction/native/HavokOffsets.h"
#include "physics-interaction/debug/DebugOverlayPolicy.h"
#include "physics-interaction/PhysicsBodyFrame.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/native/PhysicsUtils.h"
#include "RockConfig.h"

#include "RE/Bethesda/BSGraphics.h"
#include "RE/Havok/hknpShape.h"

#include <F4SE/F4SE.h>
#include <REL/Relocation.h>
#include <vrcf/VRControllersManager.h>
#include <windows.h>

#include "physics-interaction/debug/DebugAxisMath.h"

namespace frik::rock::debug
{
    namespace
    {
        constexpr std::uintptr_t kBodyArrayOffset = 0x20;
        constexpr std::uintptr_t kHighWaterMarkOffset = 0x70;
        constexpr std::uintptr_t kMotionArrayOffset = 0xE0;
        constexpr std::uintptr_t kBodyStride = 0x90;
        constexpr std::uintptr_t kMotionStride = 0x80;
        constexpr std::uintptr_t kBodyFlagsOffset = 0x40;
        constexpr std::uintptr_t kBodyFilterOffset = 0x44;
        constexpr std::uintptr_t kBodyShapeOffset = 0x48;
        constexpr std::uintptr_t kBodyMotionIndexOffset = 0x68;
        constexpr std::uintptr_t kBodyIdOffset = 0x6C;
        constexpr std::uintptr_t kBodyMotionPropertiesOffset = 0x72;
        constexpr std::uintptr_t kMotionPositionOffset = 0x00;
        constexpr std::uintptr_t kMotionOrientationOffset = 0x10;
        constexpr std::uint32_t kInvalidBodyId = 0x7FFF'FFFF;
        constexpr std::uint32_t kFreeMotionIndex = 0x7FFF'FFFF;
        constexpr std::uint32_t kMaxBodyIndex = 8192;
        constexpr std::uint32_t kMaxMotionIndex = 4096;
        constexpr float kPlanarDuplicateOffset = 0.01f;
        constexpr float kPlanarDotThreshold = 0.999f;
        constexpr float kDedupThreshold = 0.001f;
        constexpr float kRawAxisLength = 8.0f;
        constexpr float kColliderAxisLength = 12.0f;
        constexpr float kBodyAxisLength = 16.0f;
        constexpr float kTargetAxisLength = 20.0f;
        constexpr std::uint32_t kTextVertexCapacity = 131072;

        struct Vertex
        {
            float x;
            float y;
            float z;
        };

        struct MeshData
        {
            std::vector<Vertex> vertices;
            std::vector<std::uint16_t> indices;
            bool valid = false;
        };

        struct ShapeKey
        {
            std::uintptr_t bodyAddress = 0;
            std::uintptr_t shapeAddress = 0;
            std::uint32_t bodyId = kInvalidBodyId;

            bool operator==(const ShapeKey&) const = default;
        };

        struct ShapeKeyHash
        {
            std::size_t operator()(const ShapeKey& key) const
            {
                std::size_t h = 0;
                h ^= std::hash<std::uintptr_t>{}(key.bodyAddress) + 0x9e3779b9 + (h << 6) + (h >> 2);
                h ^= std::hash<std::uintptr_t>{}(key.shapeAddress) + 0x9e3779b9 + (h << 6) + (h >> 2);
                h ^= std::hash<std::uint32_t>{}(key.bodyId) + 0x9e3779b9 + (h << 6) + (h >> 2);
                return h;
            }
        };

        struct GpuShape
        {
            Microsoft::WRL::ComPtr<ID3D11Buffer> vertexBuffer;
            Microsoft::WRL::ComPtr<ID3D11Buffer> indexBuffer;
            std::uint32_t indexCount = 0;
        };

        struct BodyRenderInfo
        {
            std::uintptr_t bodyAddress = 0;
            std::uintptr_t shapeAddress = 0;
            std::uint32_t bodyId = kInvalidBodyId;
            std::uint32_t motionIndex = kFreeMotionIndex;
            std::uint32_t flags = 0;
            std::uint32_t filterInfo = 0;
            std::uint16_t motionPropertiesId = 0;
            DirectX::XMMATRIX worldMatrix = DirectX::XMMatrixIdentity();
        };

        struct OverlayRuntimeStats
        {
            std::uint32_t bodyEntries = 0;
            std::uint32_t bodiesDrawn = 0;
            std::uint32_t shapeCacheHits = 0;
            std::uint32_t shapeCacheMisses = 0;
            std::uint32_t shapeGenerations = 0;
        };

        struct alignas(16) PerFrameVSData
        {
            DirectX::XMMATRIX matProjView[2];
            DirectX::XMFLOAT4 posAdjust[2];
        };

        struct alignas(16) PerObjectVSData
        {
            DirectX::XMMATRIX matModel;
            float color[4];
        };

        struct SavedState
        {
            ID3D11VertexShader* vs = nullptr;
            ID3D11PixelShader* ps = nullptr;
            ID3D11ClassInstance* vsInstances[256] = {};
            ID3D11ClassInstance* psInstances[256] = {};
            UINT vsInstanceCount = 0;
            UINT psInstanceCount = 0;
            ID3D11Buffer* vsCBs[2] = {};
            ID3D11InputLayout* inputLayout = nullptr;
            D3D11_PRIMITIVE_TOPOLOGY topology = D3D11_PRIMITIVE_TOPOLOGY_UNDEFINED;
            ID3D11RasterizerState* rasterizerState = nullptr;
            ID3D11DepthStencilState* depthStencilState = nullptr;
            UINT stencilRef = 0;
            ID3D11BlendState* blendState = nullptr;
            FLOAT blendFactor[4] = {};
            UINT sampleMask = 0;
            ID3D11RenderTargetView* rtvs[D3D11_SIMULTANEOUS_RENDER_TARGET_COUNT] = {};
            ID3D11DepthStencilView* dsv = nullptr;
            D3D11_VIEWPORT viewports[D3D11_VIEWPORT_AND_SCISSORRECT_OBJECT_COUNT_PER_PIPELINE] = {};
            UINT numViewports = 0;
            ID3D11Buffer* vertexBuffer = nullptr;
            UINT vbStride = 0;
            UINT vbOffset = 0;
            ID3D11Buffer* indexBuffer = nullptr;
            DXGI_FORMAT ibFormat = DXGI_FORMAT_UNKNOWN;
            UINT ibOffset = 0;
        };

        static BodyOverlayFrame s_frame{};
        static std::mutex s_frameMutex;
        static std::atomic<bool> s_enabled{ false };
        static bool s_initialized = false;
        static bool s_submitHookInstalled = false;
        static bool s_installAttemptedWithoutDevice = false;
        static std::uintptr_t s_previousWorld = 0;
        static std::uint64_t s_previousOverlaySettingsKey = 0;
        static std::uint32_t s_overlayStatsLogCounter = 0;

        static ID3D11VertexShader* s_vertexShader = nullptr;
        static ID3D11VertexShader* s_screenTextVertexShader = nullptr;
        static ID3D11PixelShader* s_pixelShader = nullptr;
        static ID3D11InputLayout* s_inputLayout = nullptr;
        static ID3D11Buffer* s_cameraCB = nullptr;
        static ID3D11Buffer* s_modelCB = nullptr;
        static ID3D11Buffer* s_axisLineVB = nullptr;
        static ID3D11Buffer* s_textVB = nullptr;
        static ID3D11RasterizerState* s_wireRasterizer = nullptr;
        static ID3D11RasterizerState* s_solidRasterizer = nullptr;
        static ID3D11DepthStencilState* s_depthStencil = nullptr;
        static ID3D11BlendState* s_blendState = nullptr;
        static SavedState s_saved{};
        static std::unordered_map<ShapeKey, GpuShape, ShapeKeyHash> s_shapeCache;
        static std::uint32_t s_frameShapeGenerations = 0;

        using MainRenderCandidate_t = void (*)(void*);
        static MainRenderCandidate_t s_originalMainRenderCandidate = nullptr;

        using VRSubmit_t = vr::EVRCompositorError(__thiscall*)(vr::IVRCompositor*, vr::EVREye, const vr::Texture_t*, const vr::VRTextureBounds_t*, vr::EVRSubmitFlags);
        static VRSubmit_t s_originalVRSubmit = nullptr;
        static void** s_vrCompositorVTable = nullptr;

        const char* kVertexShaderSource = R"(
struct VS_INPUT {
    float3 vPos : POS;
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

cbuffer Model : register(b1) {
    row_major float4x4 matModel;
    float4 color;
};

VS_OUTPUT main(VS_INPUT input) {
    const float4 eyeClipEdge[2] = { { -1, 0, 0, 1 }, { 1, 0, 0, 1 } };
    const float eyeOffsetScale[2] = { -0.5, 0.5 };

    float4 pos = float4(input.vPos.xyz, 1.0f);
    pos = mul(pos, matModel);
    pos.xyz -= posAdjust[input.instanceId].xyz;
    pos = mul(matProjView[input.instanceId], pos);

    VS_OUTPUT output;
    output.vColor = color;
    output.clipDistance = dot(pos, eyeClipEdge[input.instanceId]);
    output.cullDistance = output.clipDistance;
    output.vPos = pos;
    output.vPos.x *= 0.5;
    output.vPos.x += eyeOffsetScale[input.instanceId] * output.vPos.w;
    return output;
}
)";

        const char* kScreenTextVertexShaderSource = R"(
struct VS_INPUT {
    float3 vPos : POS;
};

struct VS_OUTPUT {
    float4 vPos : SV_POSITION;
    float4 vColor : COLOR0;
};

cbuffer Model : register(b1) {
    row_major float4x4 matModel;
    float4 color;
};

VS_OUTPUT main(VS_INPUT input) {
    VS_OUTPUT output;
    output.vPos = float4(input.vPos.xy, 0.0f, 1.0f);
    output.vColor = color;
    return output;
}
)";

        const char* kPixelShaderSource = R"(
struct PS_INPUT {
    float4 pos : SV_POSITION;
    float4 color : COLOR0;
};

float4 main(PS_INPUT input) : SV_Target {
    return input.color;
}
)";

        Vertex sub(const Vertex& a, const Vertex& b) { return Vertex{ a.x - b.x, a.y - b.y, a.z - b.z }; }
        Vertex add(const Vertex& a, const Vertex& b) { return Vertex{ a.x + b.x, a.y + b.y, a.z + b.z }; }
        Vertex scale(const Vertex& value, float scalar) { return Vertex{ value.x * scalar, value.y * scalar, value.z * scalar }; }
        float dot(const Vertex& a, const Vertex& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
        Vertex cross(const Vertex& a, const Vertex& b) { return Vertex{ a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x }; }
        float lengthSq(const Vertex& value) { return dot(value, value); }
        Vertex toVertex(const RE::NiPoint3& value) { return Vertex{ value.x, value.y, value.z }; }

        Vertex normalized(const Vertex& value)
        {
            const float length = std::sqrt(lengthSq(value));
            if (length < 1.0e-8f) {
                return Vertex{ 0.0f, 0.0f, 1.0f };
            }
            return Vertex{ value.x / length, value.y / length, value.z / length };
        }

        RE::NiPoint3 normalizedNi(const RE::NiPoint3& value)
        {
            const float length = std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
            if (length < 1.0e-8f) {
                return RE::NiPoint3(0.0f, 0.0f, 1.0f);
            }
            return RE::NiPoint3(value.x / length, value.y / length, value.z / length);
        }

        bool areCoplanar(const std::vector<Vertex>& vertices)
        {
            if (vertices.size() <= 3) {
                return false;
            }

            const Vertex normal = normalized(cross(sub(vertices[1], vertices[0]), sub(vertices[2], vertices[0])));
            for (std::size_t i = 3; i < vertices.size(); i++) {
                const Vertex testNormal = normalized(cross(sub(vertices[i], vertices[0]), sub(vertices[i], vertices[1])));
                if (std::fabs(dot(normal, testNormal)) < kPlanarDotThreshold) {
                    return false;
                }
            }
            return true;
        }

        MeshData generateBoundsBox(const std::vector<Vertex>& vertices)
        {
            MeshData mesh;
            if (vertices.empty()) {
                return mesh;
            }

            Vertex minVertex = vertices.front();
            Vertex maxVertex = vertices.front();
            for (const auto& vertex : vertices) {
                minVertex.x = (std::min)(minVertex.x, vertex.x);
                minVertex.y = (std::min)(minVertex.y, vertex.y);
                minVertex.z = (std::min)(minVertex.z, vertex.z);
                maxVertex.x = (std::max)(maxVertex.x, vertex.x);
                maxVertex.y = (std::max)(maxVertex.y, vertex.y);
                maxVertex.z = (std::max)(maxVertex.z, vertex.z);
            }

            mesh.vertices = {
                { minVertex.x, minVertex.y, minVertex.z },
                { maxVertex.x, minVertex.y, minVertex.z },
                { maxVertex.x, maxVertex.y, minVertex.z },
                { minVertex.x, maxVertex.y, minVertex.z },
                { minVertex.x, minVertex.y, maxVertex.z },
                { maxVertex.x, minVertex.y, maxVertex.z },
                { maxVertex.x, maxVertex.y, maxVertex.z },
                { minVertex.x, maxVertex.y, maxVertex.z },
            };
            mesh.indices = {
                0, 1, 2, 0, 2, 3,
                4, 6, 5, 4, 7, 6,
                0, 4, 5, 0, 5, 1,
                1, 5, 6, 1, 6, 2,
                2, 6, 7, 2, 7, 3,
                3, 7, 4, 3, 4, 0,
            };
            mesh.valid = true;
            return mesh;
        }

        std::vector<std::tuple<int, int, int>> triangulateConvexHull(const std::vector<Vertex>& vertices)
        {
            std::vector<std::tuple<int, int, int>> triangles;
            const int count = static_cast<int>(vertices.size());
            if (count < 3) {
                return triangles;
            }

            Vertex centroid{ 0.0f, 0.0f, 0.0f };
            for (const auto& vertex : vertices) {
                centroid = add(centroid, vertex);
            }
            centroid = scale(centroid, 1.0f / static_cast<float>(count));

            for (int i = 0; i < count - 2; i++) {
                for (int j = i + 1; j < count - 1; j++) {
                    for (int k = j + 1; k < count; k++) {
                        const Vertex triNormal = cross(sub(vertices[j], vertices[i]), sub(vertices[k], vertices[i]));
                        bool isHullTriangle = true;
                        int previousSide = 0;

                        for (int p = 0; p < count; p++) {
                            if (p == i || p == j || p == k) {
                                continue;
                            }

                            const float d = dot(sub(vertices[p], vertices[i]), triNormal);
                            if (std::fabs(d) < 0.001f) {
                                continue;
                            }

                            const int side = d > 0.0f ? 1 : -1;
                            if (side + previousSide == 0) {
                                isHullTriangle = false;
                                break;
                            }
                            previousSide = side;
                        }

                        if (isHullTriangle) {
                            if (dot(triNormal, sub(vertices[i], centroid)) < 0.0f) {
                                triangles.emplace_back(k, j, i);
                            } else {
                                triangles.emplace_back(i, j, k);
                            }
                        }
                    }
                }
            }

            return triangles;
        }

        MeshData generateSphere(float havokRadius)
        {
            MeshData mesh;
            constexpr int segments = 12;
            constexpr int rings = 12;
            const float radius = havokRadius * havokToGameScale();
            constexpr float pi = 3.14159265358979323846f;

            for (int ring = 0; ring <= rings; ring++) {
                const float v = ring / static_cast<float>(rings);
                const float theta = v * pi;
                for (int segment = 0; segment <= segments; segment++) {
                    const float u = segment / static_cast<float>(segments);
                    const float phi = u * pi * 2.0f;
                    mesh.vertices.push_back(Vertex{ std::cos(phi) * std::sin(theta) * radius, std::cos(theta) * radius, std::sin(phi) * std::sin(theta) * radius });
                }
            }

            for (int ring = 0; ring < rings; ring++) {
                for (int segment = 0; segment < segments; segment++) {
                    const int index = ring * (segments + 1) + segment;
                    mesh.indices.push_back(static_cast<std::uint16_t>(index));
                    mesh.indices.push_back(static_cast<std::uint16_t>(index + segments + 1));
                    mesh.indices.push_back(static_cast<std::uint16_t>(index + 1));
                    mesh.indices.push_back(static_cast<std::uint16_t>(index + 1));
                    mesh.indices.push_back(static_cast<std::uint16_t>(index + segments + 1));
                    mesh.indices.push_back(static_cast<std::uint16_t>(index + segments + 2));
                }
            }

            mesh.valid = true;
            return mesh;
        }

        MeshData generateCapsule(const float* vertexA, const float* vertexB, float havokRadius)
        {
            MeshData mesh;
            constexpr int segments = 12;
            constexpr int rings = 6;
            constexpr float pi = 3.14159265358979323846f;
            const float radius = havokRadius * havokToGameScale();

            Vertex va{ vertexA[0] * havokToGameScale(), vertexA[1] * havokToGameScale(), vertexA[2] * havokToGameScale() };
            Vertex vb{ vertexB[0] * havokToGameScale(), vertexB[1] * havokToGameScale(), vertexB[2] * havokToGameScale() };
            Vertex direction = sub(vb, va);
            const float capsuleLength = std::sqrt(lengthSq(direction));
            if (capsuleLength < 0.001f) {
                return generateSphere(havokRadius);
            }
            direction = scale(direction, 1.0f / capsuleLength);

            Vertex perpendicular{ 0.0f, 0.0f, 1.0f };
            if (std::fabs(dot(direction, perpendicular)) > 0.999f) {
                perpendicular = Vertex{ 1.0f, 0.0f, 0.0f };
            }
            Vertex axis = normalized(cross(direction, perpendicular));
            perpendicular = normalized(cross(axis, direction));

            for (int segment = 0; segment <= segments; segment++) {
                const float theta = segment / static_cast<float>(segments) * pi * 2.0f;
                for (int i = 0; i < 2; i++) {
                    mesh.vertices.push_back(add(add(va, scale(direction, i * capsuleLength)), add(scale(axis, std::cos(theta) * radius), scale(perpendicular, std::sin(theta) * radius))));
                }
            }

            for (int segment = 0; segment < segments; segment++) {
                const int index = segment * 2;
                mesh.indices.push_back(static_cast<std::uint16_t>(index));
                mesh.indices.push_back(static_cast<std::uint16_t>(index + 1));
                mesh.indices.push_back(static_cast<std::uint16_t>(index + 2));
                mesh.indices.push_back(static_cast<std::uint16_t>(index + 2));
                mesh.indices.push_back(static_cast<std::uint16_t>(index + 1));
                mesh.indices.push_back(static_cast<std::uint16_t>(index + 3));
            }

            const auto addCap = [&](const Vertex& center, bool upper) {
                const int base = static_cast<int>(mesh.vertices.size());
                for (int ring = 0; ring <= rings; ring++) {
                    const float v = ring / static_cast<float>(rings);
                    const float theta = upper ? v * pi / 2.0f : pi / 2.0f + v * pi / 2.0f;
                    for (int segment = 0; segment <= segments; segment++) {
                        const float u = segment / static_cast<float>(segments);
                        const float phi = u * pi * 2.0f;
                        const float sx = std::cos(phi) * std::sin(theta);
                        const float sy = std::cos(theta);
                        const float sz = std::sin(phi) * std::sin(theta);
                        mesh.vertices.push_back(add(center, add(add(scale(axis, sx * radius), scale(direction, sy * radius)), scale(perpendicular, sz * radius))));
                    }
                }

                for (int ring = 0; ring < rings; ring++) {
                    for (int segment = 0; segment < segments; segment++) {
                        const int index = base + ring * (segments + 1) + segment;
                        mesh.indices.push_back(static_cast<std::uint16_t>(index));
                        mesh.indices.push_back(static_cast<std::uint16_t>(index + segments + 1));
                        mesh.indices.push_back(static_cast<std::uint16_t>(index + 1));
                        mesh.indices.push_back(static_cast<std::uint16_t>(index + 1));
                        mesh.indices.push_back(static_cast<std::uint16_t>(index + segments + 1));
                        mesh.indices.push_back(static_cast<std::uint16_t>(index + segments + 2));
                    }
                }
            };

            addCap(vb, true);
            addCap(va, false);
            mesh.valid = true;
            return mesh;
        }

        MeshData generateConvexPolytope(std::uintptr_t shapeAddress)
        {
            MeshData mesh;
            if (!shapeAddress) {
                return mesh;
            }

            auto* shape = reinterpret_cast<RE::hknpShape*>(shapeAddress);
            const float convexRadius = *reinterpret_cast<float*>(shapeAddress + 0x14);
            const int vertexCount = shape->GetNumberOfSupportVertices();
            if (vertexCount < 3 || vertexCount > 256) {
                return mesh;
            }

            std::vector<RE::hkVector4f> supportVertices(vertexCount);
            auto* supportBuffer = reinterpret_cast<RE::hkcdVertex*>(supportVertices.data());
            const auto* resultRaw = shape->GetSupportVertices(supportBuffer, vertexCount);
            if (!resultRaw) {
                return mesh;
            }

            const auto* resultVertices = reinterpret_cast<const RE::hkVector4f*>(resultRaw);
            std::vector<Vertex> vertices;
            vertices.reserve(vertexCount);

            for (int i = 0; i < vertexCount; i++) {
                const float* values = reinterpret_cast<const float*>(&resultVertices[i]);
                Vertex vertex{ values[0] * havokToGameScale(), values[1] * havokToGameScale(), values[2] * havokToGameScale() };
                bool duplicate = false;
                for (const auto& existing : vertices) {
                    if (lengthSq(sub(existing, vertex)) < kDedupThreshold) {
                        duplicate = true;
                        break;
                    }
                }
                if (!duplicate) {
                    vertices.push_back(vertex);
                }
            }

            if (vertices.size() < 3) {
                return mesh;
            }

            if (debug_overlay_policy::shouldUseBoundsForHeavyConvex(
                    static_cast<std::uint32_t>(vertices.size()),
                    g_rockConfig.rockDebugMaxConvexSupportVertices,
                    g_rockConfig.rockDebugUseBoundsForHeavyConvex)) {
                ROCK_LOG_DEBUG(Hand, "Debug overlay using bounds LOD for heavy convex: shape=0x{:X} supportVertices={} cap={}", shapeAddress, vertices.size(),
                    debug_overlay_policy::clampMaxConvexSupportVertices(g_rockConfig.rockDebugMaxConvexSupportVertices));
                return generateBoundsBox(vertices);
            }

            if (areCoplanar(vertices)) {
                const Vertex normal = normalized(cross(sub(vertices[1], vertices[0]), sub(vertices[2], vertices[0])));
                const std::size_t originalSize = vertices.size();
                for (std::size_t i = 0; i < originalSize; i++) {
                    vertices.push_back(add(vertices[i], scale(normal, kPlanarDuplicateOffset)));
                }
            }

            auto triangles = triangulateConvexHull(vertices);
            if (convexRadius > 0.0f) {
                const float radius = convexRadius * havokToGameScale();
                std::vector<Vertex> inflated;
                for (const auto& triangle : triangles) {
                    auto [a, b, c] = triangle;
                    const Vertex normal = normalized(cross(sub(vertices[b], vertices[a]), sub(vertices[c], vertices[a])));
                    inflated.push_back(add(vertices[a], scale(normal, radius)));
                    inflated.push_back(add(vertices[b], scale(normal, radius)));
                    inflated.push_back(add(vertices[c], scale(normal, radius)));
                }
                vertices = std::move(inflated);
                triangles = triangulateConvexHull(vertices);
            }

            for (const auto& triangle : triangles) {
                auto [a, b, c] = triangle;
                mesh.indices.push_back(static_cast<std::uint16_t>(a));
                mesh.indices.push_back(static_cast<std::uint16_t>(b));
                mesh.indices.push_back(static_cast<std::uint16_t>(c));
            }

            mesh.vertices = std::move(vertices);
            mesh.valid = !mesh.vertices.empty() && !mesh.indices.empty();
            return mesh;
        }

        MeshData generateShape(std::uintptr_t shapeAddress)
        {
            if (!shapeAddress) {
                return {};
            }

            try {
                auto* shape = reinterpret_cast<RE::hknpShape*>(shapeAddress);
                const int shapeType = static_cast<int>(shape->GetType());
                switch (shapeType) {
                case 0:
                case 1:
                    return generateConvexPolytope(shapeAddress);
                case 2:
                    return generateSphere(*reinterpret_cast<float*>(shapeAddress + 0x14));
                case 3:
                    return generateCapsule(reinterpret_cast<const float*>(shapeAddress + 0x50), reinterpret_cast<const float*>(shapeAddress + 0x60),
                        *reinterpret_cast<float*>(shapeAddress + 0x14));
                case 4:
                    return generateConvexPolytope(shapeAddress);
                case 11: {
                    const auto innerShape = *reinterpret_cast<std::uintptr_t*>(shapeAddress + 0x30);
                    if (!innerShape) {
                        return {};
                    }
                    MeshData mesh = generateShape(innerShape);
                    if (!mesh.valid) {
                        return mesh;
                    }
                    const auto* scaleVec = reinterpret_cast<const float*>(shapeAddress + 0x38);
                    for (auto& vertex : mesh.vertices) {
                        vertex.x *= scaleVec[0];
                        vertex.y *= scaleVec[1];
                        vertex.z *= scaleVec[2];
                    }
                    return mesh;
                }
                default:
                    return {};
                }
            } catch (...) {
                ROCK_LOG_WARN(Hand, "Debug overlay shape generation failed for shape=0x{:X}", shapeAddress);
                return {};
            }
        }

        DirectX::XMMATRIX quaternionToMatrix(const float* quaternion)
        {
            return DirectX::XMMatrixRotationQuaternion(DirectX::XMVectorSet(quaternion[0], quaternion[1], quaternion[2], quaternion[3]));
        }

        DirectX::XMMATRIX motionToWorldMatrix(const float* position, const float* orientation)
        {
            const DirectX::XMMATRIX rotation = quaternionToMatrix(orientation);
            const DirectX::XMMATRIX translation =
                DirectX::XMMatrixTranslation(position[0] * havokToGameScale(), position[1] * havokToGameScale(), position[2] * havokToGameScale());
            return DirectX::XMMatrixMultiply(rotation, translation);
        }

        DirectX::XMMATRIX bodyToWorldMatrix(const float* transform)
        {
            const DirectX::XMMATRIX rotation = DirectX::XMMatrixSet(transform[0], transform[4], transform[8], 0.0f, transform[1], transform[5], transform[9], 0.0f,
                transform[2], transform[6], transform[10], 0.0f, 0.0f, 0.0f, 0.0f, 1.0f);
            const DirectX::XMMATRIX translation =
                DirectX::XMMatrixTranslation(transform[12] * havokToGameScale(), transform[13] * havokToGameScale(), transform[14] * havokToGameScale());
            return DirectX::XMMatrixMultiply(rotation, translation);
        }

        bool extractBody(RE::hknpWorld* world, RE::hknpBodyId bodyId, BodyRenderInfo& out)
        {
            if (!world || bodyId.value == kInvalidBodyId || bodyId.value > kMaxBodyIndex) {
                return false;
            }

            auto worldAddress = reinterpret_cast<std::uintptr_t>(world);
            auto bodyArray = *reinterpret_cast<std::uintptr_t*>(worldAddress + kBodyArrayOffset);
            auto motionArray = *reinterpret_cast<std::uintptr_t*>(worldAddress + kMotionArrayOffset);
            auto highWaterMark = *reinterpret_cast<std::uint32_t*>(worldAddress + kHighWaterMarkOffset);
            if (!bodyArray || !motionArray || bodyId.value > highWaterMark || highWaterMark > kMaxBodyIndex) {
                return false;
            }

            const auto bodyAddress = bodyArray + static_cast<std::uintptr_t>(bodyId.value) * kBodyStride;
            const auto motionIndex = *reinterpret_cast<std::uint32_t*>(bodyAddress + kBodyMotionIndexOffset);
            if (motionIndex == kFreeMotionIndex) {
                return false;
            }

            const auto shapeAddress = *reinterpret_cast<std::uintptr_t*>(bodyAddress + kBodyShapeOffset);
            if (!shapeAddress) {
                return false;
            }

            out.bodyAddress = bodyAddress;
            out.shapeAddress = shapeAddress;
            out.bodyId = *reinterpret_cast<std::uint32_t*>(bodyAddress + kBodyIdOffset);
            out.motionIndex = motionIndex;
            out.flags = *reinterpret_cast<std::uint32_t*>(bodyAddress + kBodyFlagsOffset);
            out.filterInfo = *reinterpret_cast<std::uint32_t*>(bodyAddress + kBodyFilterOffset);
            out.motionPropertiesId = *reinterpret_cast<std::uint16_t*>(bodyAddress + kBodyMotionPropertiesOffset);

            const auto frameSource = body_frame::chooseColliderFrameSource(true, body_frame::hasUsableMotionIndex(motionIndex));
            if (frameSource == body_frame::BodyFrameSource::BodyTransform) {
                const auto* transform = reinterpret_cast<const float*>(bodyAddress);
                out.worldMatrix = bodyToWorldMatrix(transform);
            } else if (motionIndex > 0 && motionIndex < kMaxMotionIndex) {
                const auto motionAddress = motionArray + static_cast<std::uintptr_t>(motionIndex) * kMotionStride;
                const auto* position = reinterpret_cast<const float*>(motionAddress + kMotionPositionOffset);
                const auto* orientation = reinterpret_cast<const float*>(motionAddress + kMotionOrientationOffset);
                out.worldMatrix = motionToWorldMatrix(position, orientation);
            } else {
                const auto* transform = reinterpret_cast<const float*>(bodyAddress);
                out.worldMatrix = bodyToWorldMatrix(transform);
            }

            return true;
        }

        bool createBuffers(ID3D11Device* device, const MeshData& mesh, GpuShape& out)
        {
            D3D11_BUFFER_DESC vertexDesc{};
            vertexDesc.Usage = D3D11_USAGE_DEFAULT;
            vertexDesc.ByteWidth = static_cast<UINT>(sizeof(Vertex) * mesh.vertices.size());
            vertexDesc.BindFlags = D3D11_BIND_VERTEX_BUFFER;

            D3D11_SUBRESOURCE_DATA vertexData{};
            vertexData.pSysMem = mesh.vertices.data();

            HRESULT hr = device->CreateBuffer(&vertexDesc, &vertexData, out.vertexBuffer.GetAddressOf());
            if (FAILED(hr)) {
                return false;
            }

            D3D11_BUFFER_DESC indexDesc{};
            indexDesc.Usage = D3D11_USAGE_DEFAULT;
            indexDesc.ByteWidth = static_cast<UINT>(sizeof(std::uint16_t) * mesh.indices.size());
            indexDesc.BindFlags = D3D11_BIND_INDEX_BUFFER;

            D3D11_SUBRESOURCE_DATA indexData{};
            indexData.pSysMem = mesh.indices.data();

            hr = device->CreateBuffer(&indexDesc, &indexData, out.indexBuffer.GetAddressOf());
            if (FAILED(hr)) {
                out.vertexBuffer.Reset();
                return false;
            }

            out.indexCount = static_cast<std::uint32_t>(mesh.indices.size());
            return true;
        }

        const GpuShape* getOrCreateShape(ID3D11Device* device, const BodyRenderInfo& body, OverlayRuntimeStats& stats)
        {
            ShapeKey key{ body.bodyAddress, body.shapeAddress, body.bodyId };
            auto it = s_shapeCache.find(key);
            if (it != s_shapeCache.end()) {
                ++stats.shapeCacheHits;
                return it->second.indexCount > 0 ? &it->second : nullptr;
            }

            ++stats.shapeCacheMisses;
            const std::uint32_t maxGenerations = debug_overlay_policy::clampShapeGenerationsPerFrame(g_rockConfig.rockDebugMaxShapeGenerationsPerFrame);
            if (s_frameShapeGenerations >= maxGenerations) {
                return nullptr;
            }
            s_frameShapeGenerations++;
            ++stats.shapeGenerations;

            MeshData mesh = generateShape(body.shapeAddress);
            GpuShape gpuShape;
            if (!mesh.valid || mesh.vertices.empty() || mesh.indices.empty() || !createBuffers(device, mesh, gpuShape)) {
                s_shapeCache.emplace(key, GpuShape{});
                return nullptr;
            }

            auto [inserted, _] = s_shapeCache.emplace(key, std::move(gpuShape));
            return &inserted->second;
        }

        void releaseSavedState()
        {
            if (s_saved.vs) {
                s_saved.vs->Release();
            }
            if (s_saved.ps) {
                s_saved.ps->Release();
            }
            for (UINT i = 0; i < s_saved.vsInstanceCount; i++) {
                if (s_saved.vsInstances[i]) {
                    s_saved.vsInstances[i]->Release();
                }
            }
            for (UINT i = 0; i < s_saved.psInstanceCount; i++) {
                if (s_saved.psInstances[i]) {
                    s_saved.psInstances[i]->Release();
                }
            }
            for (auto* cb : s_saved.vsCBs) {
                if (cb) {
                    cb->Release();
                }
            }
            if (s_saved.inputLayout) {
                s_saved.inputLayout->Release();
            }
            if (s_saved.rasterizerState) {
                s_saved.rasterizerState->Release();
            }
            if (s_saved.depthStencilState) {
                s_saved.depthStencilState->Release();
            }
            if (s_saved.blendState) {
                s_saved.blendState->Release();
            }
            for (auto* rtv : s_saved.rtvs) {
                if (rtv) {
                    rtv->Release();
                }
            }
            if (s_saved.dsv) {
                s_saved.dsv->Release();
            }
            if (s_saved.vertexBuffer) {
                s_saved.vertexBuffer->Release();
            }
            if (s_saved.indexBuffer) {
                s_saved.indexBuffer->Release();
            }
            std::memset(&s_saved, 0, sizeof(s_saved));
        }

        bool getEyeViewProjMatrices(DirectX::XMMATRIX& outEye0, DirectX::XMMATRIX& outEye1, DirectX::XMFLOAT4& outAdjust0, DirectX::XMFLOAT4& outAdjust1)
        {
            static REL::Relocation<std::uintptr_t*> skyVRPtr{ REL::Offset(0x6235AC8) };
            const std::uintptr_t skyVR = *skyVRPtr;
            if (!skyVR) {
                return false;
            }

            const auto cameraData = *reinterpret_cast<std::uintptr_t*>(skyVR + 0x25D0);
            if (!cameraData) {
                return false;
            }

            outEye0 = DirectX::XMLoadFloat4x4(reinterpret_cast<const DirectX::XMFLOAT4X4*>(cameraData + 0xD0));
            outEye1 = DirectX::XMLoadFloat4x4(reinterpret_cast<const DirectX::XMFLOAT4X4*>(cameraData + 0x2E0));

            const float* adjust0 = reinterpret_cast<const float*>(skyVR + 0x2590);
            const float* adjust1 = reinterpret_cast<const float*>(skyVR + 0x25C0);
            outAdjust0 = DirectX::XMFLOAT4(adjust0[0], adjust0[1], adjust0[2], 0.0f);
            outAdjust1 = DirectX::XMFLOAT4(adjust1[0], adjust1[1], adjust1[2], 0.0f);
            return true;
        }

        bool initializeD3D(ID3D11Device* device)
        {
            if (!device) {
                return false;
            }

            ID3DBlob* vsBlob = nullptr;
            ID3DBlob* psBlob = nullptr;
            ID3DBlob* errorBlob = nullptr;
            HRESULT hr = D3DCompile(kVertexShaderSource, std::strlen(kVertexShaderSource), "ROCKDebugBodyVS", nullptr, nullptr, "main", "vs_5_0",
                D3DCOMPILE_ENABLE_STRICTNESS | D3DCOMPILE_PACK_MATRIX_COLUMN_MAJOR, 0, &vsBlob, &errorBlob);
            if (FAILED(hr)) {
                if (errorBlob) {
                    ROCK_LOG_ERROR(Hand, "Debug overlay vertex shader compile failed: {}", static_cast<const char*>(errorBlob->GetBufferPointer()));
                    errorBlob->Release();
                }
                return false;
            }

            hr = device->CreateVertexShader(vsBlob->GetBufferPointer(), vsBlob->GetBufferSize(), nullptr, &s_vertexShader);
            if (FAILED(hr)) {
                vsBlob->Release();
                return false;
            }

            D3D11_INPUT_ELEMENT_DESC layoutDesc[] = { { "POS", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0 } };
            hr = device->CreateInputLayout(layoutDesc, 1, vsBlob->GetBufferPointer(), vsBlob->GetBufferSize(), &s_inputLayout);
            vsBlob->Release();
            if (FAILED(hr)) {
                return false;
            }

            hr = D3DCompile(kScreenTextVertexShaderSource, std::strlen(kScreenTextVertexShaderSource), "ROCKDebugTextVS", nullptr, nullptr, "main", "vs_5_0",
                D3DCOMPILE_ENABLE_STRICTNESS | D3DCOMPILE_PACK_MATRIX_COLUMN_MAJOR, 0, &vsBlob, &errorBlob);
            if (FAILED(hr)) {
                if (errorBlob) {
                    ROCK_LOG_ERROR(Hand, "Debug overlay text vertex shader compile failed: {}", static_cast<const char*>(errorBlob->GetBufferPointer()));
                    errorBlob->Release();
                }
                return false;
            }

            hr = device->CreateVertexShader(vsBlob->GetBufferPointer(), vsBlob->GetBufferSize(), nullptr, &s_screenTextVertexShader);
            vsBlob->Release();
            if (FAILED(hr)) {
                return false;
            }

            hr = D3DCompile(kPixelShaderSource, std::strlen(kPixelShaderSource), "ROCKDebugBodyPS", nullptr, nullptr, "main", "ps_5_0",
                D3DCOMPILE_ENABLE_STRICTNESS | D3DCOMPILE_PACK_MATRIX_COLUMN_MAJOR, 0, &psBlob, &errorBlob);
            if (FAILED(hr)) {
                if (errorBlob) {
                    ROCK_LOG_ERROR(Hand, "Debug overlay pixel shader compile failed: {}", static_cast<const char*>(errorBlob->GetBufferPointer()));
                    errorBlob->Release();
                }
                return false;
            }

            hr = device->CreatePixelShader(psBlob->GetBufferPointer(), psBlob->GetBufferSize(), nullptr, &s_pixelShader);
            psBlob->Release();
            if (FAILED(hr)) {
                return false;
            }

            D3D11_BUFFER_DESC cameraDesc{};
            cameraDesc.Usage = D3D11_USAGE_DYNAMIC;
            cameraDesc.ByteWidth = sizeof(PerFrameVSData);
            cameraDesc.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
            cameraDesc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
            if (FAILED(device->CreateBuffer(&cameraDesc, nullptr, &s_cameraCB))) {
                return false;
            }

            D3D11_BUFFER_DESC modelDesc{};
            modelDesc.Usage = D3D11_USAGE_DYNAMIC;
            modelDesc.ByteWidth = sizeof(PerObjectVSData);
            modelDesc.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
            modelDesc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
            if (FAILED(device->CreateBuffer(&modelDesc, nullptr, &s_modelCB))) {
                return false;
            }

            D3D11_BUFFER_DESC axisLineDesc{};
            axisLineDesc.Usage = D3D11_USAGE_DYNAMIC;
            axisLineDesc.ByteWidth = sizeof(Vertex) * 2;
            axisLineDesc.BindFlags = D3D11_BIND_VERTEX_BUFFER;
            axisLineDesc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
            if (FAILED(device->CreateBuffer(&axisLineDesc, nullptr, &s_axisLineVB))) {
                return false;
            }

            D3D11_BUFFER_DESC textDesc{};
            textDesc.Usage = D3D11_USAGE_DYNAMIC;
            textDesc.ByteWidth = sizeof(Vertex) * kTextVertexCapacity;
            textDesc.BindFlags = D3D11_BIND_VERTEX_BUFFER;
            textDesc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
            if (FAILED(device->CreateBuffer(&textDesc, nullptr, &s_textVB))) {
                return false;
            }

            D3D11_RASTERIZER_DESC rasterDesc{};
            rasterDesc.FillMode = D3D11_FILL_WIREFRAME;
            rasterDesc.CullMode = D3D11_CULL_NONE;
            rasterDesc.FrontCounterClockwise = TRUE;
            rasterDesc.DepthClipEnable = TRUE;
            if (FAILED(device->CreateRasterizerState(&rasterDesc, &s_wireRasterizer))) {
                return false;
            }

            rasterDesc.FillMode = D3D11_FILL_SOLID;
            if (FAILED(device->CreateRasterizerState(&rasterDesc, &s_solidRasterizer))) {
                return false;
            }

            D3D11_DEPTH_STENCIL_DESC depthDesc{};
            depthDesc.DepthEnable = FALSE;
            depthDesc.DepthWriteMask = D3D11_DEPTH_WRITE_MASK_ZERO;
            depthDesc.DepthFunc = D3D11_COMPARISON_ALWAYS;
            if (FAILED(device->CreateDepthStencilState(&depthDesc, &s_depthStencil))) {
                return false;
            }

            D3D11_BLEND_DESC blendDesc{};
            blendDesc.RenderTarget[0].BlendEnable = TRUE;
            blendDesc.RenderTarget[0].SrcBlend = D3D11_BLEND_SRC_ALPHA;
            blendDesc.RenderTarget[0].DestBlend = D3D11_BLEND_INV_SRC_ALPHA;
            blendDesc.RenderTarget[0].BlendOp = D3D11_BLEND_OP_ADD;
            blendDesc.RenderTarget[0].SrcBlendAlpha = D3D11_BLEND_ONE;
            blendDesc.RenderTarget[0].DestBlendAlpha = D3D11_BLEND_ZERO;
            blendDesc.RenderTarget[0].BlendOpAlpha = D3D11_BLEND_OP_ADD;
            blendDesc.RenderTarget[0].RenderTargetWriteMask = D3D11_COLOR_WRITE_ENABLE_ALL;
            if (FAILED(device->CreateBlendState(&blendDesc, &s_blendState))) {
                return false;
            }

            return true;
        }

        ID3D11Device* getDevice()
        {
            auto* renderer = RE::BSGraphics::RendererData::GetSingleton();
            return renderer ? reinterpret_cast<ID3D11Device*>(renderer->device) : nullptr;
        }

        ID3D11DeviceContext* getContext()
        {
            auto* renderer = RE::BSGraphics::RendererData::GetSingleton();
            return renderer ? reinterpret_cast<ID3D11DeviceContext*>(renderer->context) : nullptr;
        }

        void beginFrame(ID3D11DeviceContext* context)
        {
            std::memset(&s_saved, 0, sizeof(s_saved));
            s_saved.vsInstanceCount = 256;
            s_saved.psInstanceCount = 256;
            context->VSGetShader(&s_saved.vs, s_saved.vsInstances, &s_saved.vsInstanceCount);
            context->PSGetShader(&s_saved.ps, s_saved.psInstances, &s_saved.psInstanceCount);
            context->VSGetConstantBuffers(0, 2, s_saved.vsCBs);
            context->IAGetInputLayout(&s_saved.inputLayout);
            context->IAGetPrimitiveTopology(&s_saved.topology);
            context->RSGetState(&s_saved.rasterizerState);
            context->OMGetDepthStencilState(&s_saved.depthStencilState, &s_saved.stencilRef);
            context->OMGetBlendState(&s_saved.blendState, s_saved.blendFactor, &s_saved.sampleMask);
            context->OMGetRenderTargets(D3D11_SIMULTANEOUS_RENDER_TARGET_COUNT, s_saved.rtvs, &s_saved.dsv);
            s_saved.numViewports = D3D11_VIEWPORT_AND_SCISSORRECT_OBJECT_COUNT_PER_PIPELINE;
            context->RSGetViewports(&s_saved.numViewports, s_saved.viewports);
            context->IAGetVertexBuffers(0, 1, &s_saved.vertexBuffer, &s_saved.vbStride, &s_saved.vbOffset);
            context->IAGetIndexBuffer(&s_saved.indexBuffer, &s_saved.ibFormat, &s_saved.ibOffset);

            context->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
            context->IASetInputLayout(s_inputLayout);
            context->VSSetShader(s_vertexShader, nullptr, 0);
            context->PSSetShader(s_pixelShader, nullptr, 0);
            context->RSSetState(s_wireRasterizer);
            FLOAT blendFactor[4] = {};
            context->OMSetBlendState(s_blendState, blendFactor, 0xFFFFFFFF);
            context->OMSetDepthStencilState(s_depthStencil, 0);
        }

        void endFrame(ID3D11DeviceContext* context)
        {
            context->VSSetShader(s_saved.vs, s_saved.vsInstances, s_saved.vsInstanceCount);
            context->PSSetShader(s_saved.ps, s_saved.psInstances, s_saved.psInstanceCount);
            context->VSSetConstantBuffers(0, 2, s_saved.vsCBs);
            context->IASetInputLayout(s_saved.inputLayout);
            context->IASetPrimitiveTopology(s_saved.topology);
            context->RSSetState(s_saved.rasterizerState);
            context->OMSetDepthStencilState(s_saved.depthStencilState, s_saved.stencilRef);
            context->OMSetBlendState(s_saved.blendState, s_saved.blendFactor, s_saved.sampleMask);
            context->OMSetRenderTargets(D3D11_SIMULTANEOUS_RENDER_TARGET_COUNT, s_saved.rtvs, s_saved.dsv);
            context->RSSetViewports(s_saved.numViewports, s_saved.viewports);
            context->IASetVertexBuffers(0, 1, &s_saved.vertexBuffer, &s_saved.vbStride, &s_saved.vbOffset);
            context->IASetIndexBuffer(s_saved.indexBuffer, s_saved.ibFormat, s_saved.ibOffset);
            releaseSavedState();
        }

        void uploadCamera(ID3D11DeviceContext* context, const DirectX::XMMATRIX& eye0, const DirectX::XMMATRIX& eye1, const DirectX::XMFLOAT4& adjust0,
            const DirectX::XMFLOAT4& adjust1)
        {
            D3D11_MAPPED_SUBRESOURCE mapped{};
            if (SUCCEEDED(context->Map(s_cameraCB, 0, D3D11_MAP_WRITE_DISCARD, 0, &mapped))) {
                auto* data = static_cast<PerFrameVSData*>(mapped.pData);
                data->matProjView[0] = eye0;
                data->matProjView[1] = eye1;
                data->posAdjust[0] = adjust0;
                data->posAdjust[1] = adjust1;
                context->Unmap(s_cameraCB, 0);
            }
            context->VSSetConstantBuffers(0, 1, &s_cameraCB);
        }

        void uploadColorModel(ID3D11DeviceContext* context, const DirectX::XMMATRIX& model, const float color[4])
        {
            D3D11_MAPPED_SUBRESOURCE mapped{};
            if (SUCCEEDED(context->Map(s_modelCB, 0, D3D11_MAP_WRITE_DISCARD, 0, &mapped))) {
                auto* data = static_cast<PerObjectVSData*>(mapped.pData);
                data->matModel = model;
                data->color[0] = color[0];
                data->color[1] = color[1];
                data->color[2] = color[2];
                data->color[3] = color[3];
                context->Unmap(s_modelCB, 0);
            }
            context->VSSetConstantBuffers(1, 1, &s_modelCB);
        }

        void uploadModel(ID3D11DeviceContext* context, const DirectX::XMMATRIX& model, BodyOverlayRole role)
        {
            float color[4] = { 1.0f, 1.0f, 1.0f, 0.85f };
            switch (role) {
            case BodyOverlayRole::RightHand:
                color[0] = 0.0f;
                color[1] = 0.85f;
                color[2] = 1.0f;
                break;
            case BodyOverlayRole::LeftHand:
                color[0] = 1.0f;
                color[1] = 0.25f;
                color[2] = 0.9f;
                break;
            case BodyOverlayRole::RightHandSegment:
                color[0] = 0.10f;
                color[1] = 0.65f;
                color[2] = 1.0f;
                color[3] = 0.55f;
                break;
            case BodyOverlayRole::LeftHandSegment:
                color[0] = 1.0f;
                color[1] = 0.20f;
                color[2] = 0.85f;
                color[3] = 0.55f;
                break;
            case BodyOverlayRole::Weapon:
                color[0] = 0.35f;
                color[1] = 1.0f;
                color[2] = 0.25f;
                break;
            case BodyOverlayRole::Target:
                color[0] = 1.0f;
                color[1] = 0.85f;
                color[2] = 0.05f;
                break;
            }

            uploadColorModel(context, model, color);
        }

        float axisLengthForRole(AxisOverlayRole role)
        {
            switch (role) {
            case AxisOverlayRole::RightHandRaw:
            case AxisOverlayRole::LeftHandRaw:
                return kRawAxisLength;
            case AxisOverlayRole::RightHandBody:
            case AxisOverlayRole::LeftHandBody:
                return kBodyAxisLength;
            case AxisOverlayRole::WeaponAuthority:
            case AxisOverlayRole::RightWeaponPrimaryGrip:
            case AxisOverlayRole::LeftWeaponSupportGrip:
            case AxisOverlayRole::RightFrikAppliedHand:
            case AxisOverlayRole::LeftFrikAppliedHand:
            case AxisOverlayRole::RightGrabHiggsReverse:
            case AxisOverlayRole::LeftGrabHiggsReverse:
            case AxisOverlayRole::RightGrabConstraintReverse:
            case AxisOverlayRole::LeftGrabConstraintReverse:
            case AxisOverlayRole::RightGrabRockVisualTarget:
            case AxisOverlayRole::LeftGrabRockVisualTarget:
            case AxisOverlayRole::RightGrabDesiredObject:
            case AxisOverlayRole::LeftGrabDesiredObject:
            case AxisOverlayRole::RightGrabHeldNode:
            case AxisOverlayRole::LeftGrabHeldNode:
                return kColliderAxisLength;
            case AxisOverlayRole::TargetBody:
                return kTargetAxisLength;
            }
            return kBodyAxisLength;
        }

        float axisAlphaForRole(AxisOverlayRole role)
        {
            switch (role) {
            case AxisOverlayRole::RightHandRaw:
            case AxisOverlayRole::LeftHandRaw:
                return 0.55f;
            case AxisOverlayRole::WeaponAuthority:
            case AxisOverlayRole::RightWeaponPrimaryGrip:
            case AxisOverlayRole::LeftWeaponSupportGrip:
            case AxisOverlayRole::RightFrikAppliedHand:
            case AxisOverlayRole::LeftFrikAppliedHand:
            case AxisOverlayRole::RightGrabHiggsReverse:
            case AxisOverlayRole::LeftGrabHiggsReverse:
            case AxisOverlayRole::RightGrabConstraintReverse:
            case AxisOverlayRole::LeftGrabConstraintReverse:
            case AxisOverlayRole::RightGrabRockVisualTarget:
            case AxisOverlayRole::LeftGrabRockVisualTarget:
            case AxisOverlayRole::RightGrabDesiredObject:
            case AxisOverlayRole::LeftGrabDesiredObject:
            case AxisOverlayRole::RightGrabHeldNode:
            case AxisOverlayRole::LeftGrabHeldNode:
                return 0.92f;
            default:
                return 1.0f;
            }
        }

        void markerColorForRole(MarkerOverlayRole role, float color[4])
        {
            color[0] = 1.0f;
            color[1] = 1.0f;
            color[2] = 1.0f;
            color[3] = 1.0f;

            switch (role) {
            case MarkerOverlayRole::RightGrabAnchor:
                color[0] = 0.35f;
                color[1] = 1.0f;
                color[2] = 1.0f;
                break;
            case MarkerOverlayRole::LeftGrabAnchor:
                color[0] = 1.0f;
                color[1] = 0.35f;
                color[2] = 0.95f;
                break;
            case MarkerOverlayRole::RightPalmNormal:
            case MarkerOverlayRole::LeftPalmNormal:
                color[0] = 1.0f;
                color[1] = 0.82f;
                color[2] = 0.05f;
                break;
            case MarkerOverlayRole::RightPointing:
            case MarkerOverlayRole::LeftPointing:
                color[0] = 0.20f;
                color[1] = 0.55f;
                color[2] = 1.0f;
                color[3] = 0.85f;
                break;
            case MarkerOverlayRole::RightGrabPivotA:
                color[0] = 0.0f;
                color[1] = 1.0f;
                color[2] = 1.0f;
                break;
            case MarkerOverlayRole::LeftGrabPivotA:
                color[0] = 1.0f;
                color[1] = 0.25f;
                color[2] = 1.0f;
                break;
            case MarkerOverlayRole::RightGrabPivotB:
            case MarkerOverlayRole::LeftGrabPivotB:
                color[0] = 1.0f;
                color[1] = 0.95f;
                color[2] = 0.15f;
                break;
            case MarkerOverlayRole::RightGrabPivotError:
            case MarkerOverlayRole::LeftGrabPivotError:
                color[0] = 1.0f;
                color[1] = 0.08f;
                color[2] = 0.04f;
                break;
            case MarkerOverlayRole::RightGrabSurfacePoint:
                color[0] = 0.10f;
                color[1] = 1.0f;
                color[2] = 0.90f;
                color[3] = 0.95f;
                break;
            case MarkerOverlayRole::LeftGrabSurfacePoint:
                color[0] = 1.0f;
                color[1] = 0.25f;
                color[2] = 0.95f;
                color[3] = 0.95f;
                break;
            case MarkerOverlayRole::RightGrabSurfaceNormal:
            case MarkerOverlayRole::LeftGrabSurfaceNormal:
                color[0] = 1.0f;
                color[1] = 0.90f;
                color[2] = 0.05f;
                color[3] = 0.95f;
                break;
            case MarkerOverlayRole::RightGrabSurfaceTangent:
            case MarkerOverlayRole::LeftGrabSurfaceTangent:
                color[0] = 0.25f;
                color[1] = 0.65f;
                color[2] = 1.0f;
                color[3] = 0.90f;
                break;
            case MarkerOverlayRole::RightGrabSurfaceBitangent:
            case MarkerOverlayRole::LeftGrabSurfaceBitangent:
                color[0] = 0.35f;
                color[1] = 1.0f;
                color[2] = 0.35f;
                color[3] = 0.90f;
                break;
            case MarkerOverlayRole::RightGrabContactPatchSample:
                color[0] = 0.05f;
                color[1] = 0.80f;
                color[2] = 1.0f;
                color[3] = 0.80f;
                break;
            case MarkerOverlayRole::LeftGrabContactPatchSample:
                color[0] = 1.0f;
                color[1] = 0.20f;
                color[2] = 0.90f;
                color[3] = 0.80f;
                break;
            case MarkerOverlayRole::RightGrabFingerProbe:
                color[0] = 0.45f;
                color[1] = 1.0f;
                color[2] = 0.45f;
                color[3] = 0.75f;
                break;
            case MarkerOverlayRole::LeftGrabFingerProbe:
                color[0] = 1.0f;
                color[1] = 0.45f;
                color[2] = 0.95f;
                color[3] = 0.75f;
                break;
            case MarkerOverlayRole::RightHandBoneContact:
                color[0] = 0.0f;
                color[1] = 1.0f;
                color[2] = 1.0f;
                color[3] = 0.95f;
                break;
            case MarkerOverlayRole::LeftHandBoneContact:
                color[0] = 1.0f;
                color[1] = 0.25f;
                color[2] = 1.0f;
                color[3] = 0.95f;
                break;
            case MarkerOverlayRole::RightWeaponPrimaryGrip:
                color[0] = 0.10f;
                color[1] = 0.95f;
                color[2] = 1.0f;
                break;
            case MarkerOverlayRole::LeftWeaponSupportGrip:
                color[0] = 1.0f;
                color[1] = 0.40f;
                color[2] = 0.85f;
                break;
            case MarkerOverlayRole::RightWeaponAuthorityMismatch:
            case MarkerOverlayRole::LeftWeaponAuthorityMismatch:
                color[0] = 1.0f;
                color[1] = 0.15f;
                color[2] = 0.05f;
                break;
            case MarkerOverlayRole::RightRootFlattenedFingerSkeleton:
                color[0] = 0.05f;
                color[1] = 0.95f;
                color[2] = 1.0f;
                color[3] = 0.88f;
                break;
            case MarkerOverlayRole::LeftRootFlattenedFingerSkeleton:
                color[0] = 1.0f;
                color[1] = 0.25f;
                color[2] = 0.95f;
                color[3] = 0.88f;
                break;
            case MarkerOverlayRole::TargetVisualOrigin:
                color[0] = 0.20f;
                color[1] = 1.0f;
                color[2] = 0.35f;
                color[3] = 0.95f;
                break;
            case MarkerOverlayRole::TargetRawBodyOrigin:
                color[0] = 1.0f;
                color[1] = 0.45f;
                color[2] = 0.05f;
                color[3] = 0.95f;
                break;
            case MarkerOverlayRole::TargetBodyTransformOrigin:
                color[0] = 1.0f;
                color[1] = 0.85f;
                color[2] = 0.10f;
                color[3] = 0.95f;
                break;
            case MarkerOverlayRole::TargetMotionOrigin:
                color[0] = 0.95f;
                color[1] = 0.15f;
                color[2] = 1.0f;
                color[3] = 0.95f;
                break;
            case MarkerOverlayRole::TargetBestOriginCandidate:
                color[0] = 0.10f;
                color[1] = 0.75f;
                color[2] = 1.0f;
                color[3] = 0.95f;
                break;
            case MarkerOverlayRole::TargetOriginErrorLine:
                color[0] = 1.0f;
                color[1] = 0.05f;
                color[2] = 0.05f;
                color[3] = 0.95f;
                break;
            case MarkerOverlayRole::RightGrabHiggsReverseError:
            case MarkerOverlayRole::LeftGrabHiggsReverseError:
                color[0] = 0.15f;
                color[1] = 1.0f;
                color[2] = 0.25f;
                color[3] = 0.95f;
                break;
            case MarkerOverlayRole::RightGrabConstraintReverseError:
            case MarkerOverlayRole::LeftGrabConstraintReverseError:
                color[0] = 0.20f;
                color[1] = 0.55f;
                color[2] = 1.0f;
                color[3] = 0.95f;
                break;
            case MarkerOverlayRole::RightGrabRockVisualError:
            case MarkerOverlayRole::LeftGrabRockVisualError:
                color[0] = 1.0f;
                color[1] = 0.75f;
                color[2] = 0.05f;
                color[3] = 0.95f;
                break;
            case MarkerOverlayRole::RightGrabHeldDesiredError:
            case MarkerOverlayRole::LeftGrabHeldDesiredError:
                color[0] = 1.0f;
                color[1] = 0.35f;
                color[2] = 0.90f;
                color[3] = 0.95f;
                break;
            case MarkerOverlayRole::RightGrabTelemetryLabelAnchor:
            case MarkerOverlayRole::LeftGrabTelemetryLabelAnchor:
                color[0] = 0.95f;
                color[1] = 1.0f;
                color[2] = 0.20f;
                color[3] = 0.95f;
                break;
            }
        }

        void skeletonColorForRole(SkeletonOverlayRole role, bool inPowerArmor, float color[4])
        {
            color[0] = 0.90f;
            color[1] = 0.90f;
            color[2] = 0.90f;
            color[3] = inPowerArmor ? 0.95f : 0.78f;

            switch (role) {
            case SkeletonOverlayRole::Core:
                color[0] = inPowerArmor ? 0.95f : 0.85f;
                color[1] = inPowerArmor ? 0.80f : 0.85f;
                color[2] = inPowerArmor ? 0.30f : 0.95f;
                break;
            case SkeletonOverlayRole::Head:
                color[0] = 1.0f;
                color[1] = 0.95f;
                color[2] = 0.65f;
                break;
            case SkeletonOverlayRole::RightArm:
                color[0] = 0.10f;
                color[1] = 0.95f;
                color[2] = 1.0f;
                break;
            case SkeletonOverlayRole::LeftArm:
                color[0] = 1.0f;
                color[1] = 0.25f;
                color[2] = 0.95f;
                break;
            case SkeletonOverlayRole::RightFinger:
                color[0] = 0.10f;
                color[1] = 1.0f;
                color[2] = 0.55f;
                break;
            case SkeletonOverlayRole::LeftFinger:
                color[0] = 1.0f;
                color[1] = 0.55f;
                color[2] = 0.95f;
                break;
            case SkeletonOverlayRole::RightLeg:
                color[0] = 0.30f;
                color[1] = 0.55f;
                color[2] = 1.0f;
                break;
            case SkeletonOverlayRole::LeftLeg:
                color[0] = 0.75f;
                color[1] = 0.45f;
                color[2] = 1.0f;
                break;
            }
        }

        Vertex transformPoint(const DirectX::XMMATRIX& matrix, float x, float y, float z)
        {
            DirectX::XMFLOAT3 out{};
            DirectX::XMStoreFloat3(&out, DirectX::XMVector3TransformCoord(DirectX::XMVectorSet(x, y, z, 1.0f), matrix));
            return Vertex{ out.x, out.y, out.z };
        }

        void drawDebugLine(ID3D11DeviceContext* context, const Vertex& start, const Vertex& end, const float color[4])
        {
            if (!s_axisLineVB) {
                return;
            }

            D3D11_MAPPED_SUBRESOURCE mapped{};
            if (FAILED(context->Map(s_axisLineVB, 0, D3D11_MAP_WRITE_DISCARD, 0, &mapped))) {
                return;
            }

            auto* vertices = static_cast<Vertex*>(mapped.pData);
            vertices[0] = start;
            vertices[1] = end;
            context->Unmap(s_axisLineVB, 0);

            constexpr UINT stride = sizeof(Vertex);
            constexpr UINT offset = 0;
            ID3D11Buffer* vertexBuffer = s_axisLineVB;
            context->IASetVertexBuffers(0, 1, &vertexBuffer, &stride, &offset);
            context->IASetIndexBuffer(nullptr, DXGI_FORMAT_UNKNOWN, 0);
            uploadColorModel(context, DirectX::XMMatrixIdentity(), color);
            context->DrawInstanced(2, 2, 0, 0);
        }

        void drawPointMarker(ID3D11DeviceContext* context, const RE::NiPoint3& position, float size, const float color[4])
        {
            const Vertex center = toVertex(position);
            drawDebugLine(context, add(center, Vertex{ -size, 0.0f, 0.0f }), add(center, Vertex{ size, 0.0f, 0.0f }), color);
            drawDebugLine(context, add(center, Vertex{ 0.0f, -size, 0.0f }), add(center, Vertex{ 0.0f, size, 0.0f }), color);
            drawDebugLine(context, add(center, Vertex{ 0.0f, 0.0f, -size }), add(center, Vertex{ 0.0f, 0.0f, size }), color);
        }

        void drawAxisTripod(ID3D11DeviceContext* context, const Vertex& origin, const Vertex& xEnd, const Vertex& yEnd, const Vertex& zEnd, AxisOverlayRole role)
        {
            const float alpha = axisAlphaForRole(role);
            const float xColor[4] = { 1.0f, 0.05f, 0.05f, alpha };
            const float yColor[4] = { 0.05f, 1.0f, 0.10f, alpha };
            const float zColor[4] = { 0.10f, 0.35f, 1.0f, alpha };
            drawDebugLine(context, origin, xEnd, xColor);
            drawDebugLine(context, origin, yEnd, yColor);
            drawDebugLine(context, origin, zEnd, zColor);
        }

        void drawTransformAxisEntry(ID3D11DeviceContext* context, const AxisOverlayEntry& entry)
        {
            const float length = axisLengthForRole(entry.role);
            const RE::NiPoint3 origin = entry.transform.translate;
            const RE::NiPoint3 xAxis = normalizedNi(debug_axis_math::rotateNiLocalToWorld(entry.transform.rotate, RE::NiPoint3(1.0f, 0.0f, 0.0f)));
            const RE::NiPoint3 yAxis = normalizedNi(debug_axis_math::rotateNiLocalToWorld(entry.transform.rotate, RE::NiPoint3(0.0f, 1.0f, 0.0f)));
            const RE::NiPoint3 zAxis = normalizedNi(debug_axis_math::rotateNiLocalToWorld(entry.transform.rotate, RE::NiPoint3(0.0f, 0.0f, 1.0f)));

            drawAxisTripod(context, toVertex(origin), toVertex(origin + xAxis * length), toVertex(origin + yAxis * length), toVertex(origin + zAxis * length), entry.role);

            if (entry.drawTranslationLine) {
                const float color[4] = { 1.0f, 0.86f, 0.05f, axisAlphaForRole(entry.role) };
                drawDebugLine(context, toVertex(entry.translationStart), toVertex(origin), color);
            }
        }

        void drawBodyAxisEntry(ID3D11DeviceContext* context, RE::hknpWorld* world, const AxisOverlayEntry& entry)
        {
            BodyRenderInfo body{};
            if (!extractBody(world, entry.bodyId, body)) {
                return;
            }

            const float length = axisLengthForRole(entry.role);
            const Vertex origin = transformPoint(body.worldMatrix, 0.0f, 0.0f, 0.0f);
            drawAxisTripod(context, origin, transformPoint(body.worldMatrix, length, 0.0f, 0.0f), transformPoint(body.worldMatrix, 0.0f, length, 0.0f),
                transformPoint(body.worldMatrix, 0.0f, 0.0f, length), entry.role);

            if (entry.drawTranslationLine) {
                const float color[4] = { 1.0f, 0.86f, 0.05f, axisAlphaForRole(entry.role) };
                drawDebugLine(context, toVertex(entry.translationStart), origin, color);
            }
        }

        void drawAxisOverlays(ID3D11DeviceContext* context, RE::hknpWorld* world, const BodyOverlayFrame& frame)
        {
            if (!frame.drawAxes || frame.axisCount == 0) {
                return;
            }

            context->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_LINELIST);
            for (std::uint32_t i = 0; i < frame.axisCount && i < frame.axisEntries.size(); i++) {
                const auto& entry = frame.axisEntries[i];
                if (entry.source == AxisOverlaySource::Body) {
                    drawBodyAxisEntry(context, world, entry);
                } else {
                    drawTransformAxisEntry(context, entry);
                }
            }
        }

        void drawMarkerOverlays(ID3D11DeviceContext* context, const BodyOverlayFrame& frame)
        {
            if (!frame.drawMarkers || frame.markerCount == 0) {
                return;
            }

            context->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_LINELIST);
            for (std::uint32_t i = 0; i < frame.markerCount && i < frame.markerEntries.size(); i++) {
                const auto& entry = frame.markerEntries[i];
                float color[4]{};
                markerColorForRole(entry.role, color);

                if (entry.drawLine) {
                    drawDebugLine(context, toVertex(entry.position), toVertex(entry.lineEnd), color);
                }
                if (entry.drawPoint) {
                    drawPointMarker(context, entry.position, entry.size, color);
                }
            }
        }

        void drawSkeletonOverlays(ID3D11DeviceContext* context, const BodyOverlayFrame& frame)
        {
            if (!frame.drawSkeleton || frame.skeletonCount == 0) {
                return;
            }

            context->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_LINELIST);
            for (std::uint32_t i = 0; i < frame.skeletonCount && i < frame.skeletonEntries.size(); i++) {
                const auto& entry = frame.skeletonEntries[i];
                float color[4]{};
                skeletonColorForRole(entry.role, entry.inPowerArmor, color);

                if (entry.hasParent) {
                    drawDebugLine(context, toVertex(entry.parentPosition), toVertex(entry.transform.translate), color);
                }
                if (entry.drawPoint) {
                    drawPointMarker(context, entry.transform.translate, entry.pointSize, color);
                }
                if (entry.drawAxis) {
                    const auto endpoints = skeleton_bone_debug_math::computeAxisEndpoints(entry.transform, entry.axisLength);
                    const float axisAlpha = entry.inPowerArmor ? 0.95f : 0.70f;
                    const float xColor[4] = { 1.0f, 0.08f, 0.06f, axisAlpha };
                    const float yColor[4] = { 0.05f, 1.0f, 0.10f, axisAlpha };
                    const float zColor[4] = { 0.10f, 0.35f, 1.0f, axisAlpha };
                    drawDebugLine(context, toVertex(entry.transform.translate), toVertex(endpoints.xEnd), xColor);
                    drawDebugLine(context, toVertex(entry.transform.translate), toVertex(endpoints.yEnd), yColor);
                    drawDebugLine(context, toVertex(entry.transform.translate), toVertex(endpoints.zEnd), zColor);
                }
            }
        }

        std::array<std::uint8_t, 7> glyphRows(char ch)
        {
            if (ch >= 'a' && ch <= 'z') {
                ch = static_cast<char>(ch - 'a' + 'A');
            }

            switch (ch) {
            case '0':
                return { 0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E };
            case '1':
                return { 0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E };
            case '2':
                return { 0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F };
            case '3':
                return { 0x1E, 0x01, 0x01, 0x0E, 0x01, 0x01, 0x1E };
            case '4':
                return { 0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02 };
            case '5':
                return { 0x1F, 0x10, 0x10, 0x1E, 0x01, 0x01, 0x1E };
            case '6':
                return { 0x0E, 0x10, 0x10, 0x1E, 0x11, 0x11, 0x0E };
            case '7':
                return { 0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08 };
            case '8':
                return { 0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E };
            case '9':
                return { 0x0E, 0x11, 0x11, 0x0F, 0x01, 0x01, 0x0E };
            case 'A':
                return { 0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11 };
            case 'B':
                return { 0x1E, 0x11, 0x11, 0x1E, 0x11, 0x11, 0x1E };
            case 'C':
                return { 0x0F, 0x10, 0x10, 0x10, 0x10, 0x10, 0x0F };
            case 'D':
                return { 0x1E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1E };
            case 'E':
                return { 0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x1F };
            case 'F':
                return { 0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x10 };
            case 'G':
                return { 0x0F, 0x10, 0x10, 0x13, 0x11, 0x11, 0x0F };
            case 'H':
                return { 0x11, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11 };
            case 'I':
                return { 0x0E, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0E };
            case 'J':
                return { 0x01, 0x01, 0x01, 0x01, 0x11, 0x11, 0x0E };
            case 'K':
                return { 0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11 };
            case 'L':
                return { 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1F };
            case 'M':
                return { 0x11, 0x1B, 0x15, 0x15, 0x11, 0x11, 0x11 };
            case 'N':
                return { 0x11, 0x19, 0x15, 0x13, 0x11, 0x11, 0x11 };
            case 'O':
                return { 0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E };
            case 'P':
                return { 0x1E, 0x11, 0x11, 0x1E, 0x10, 0x10, 0x10 };
            case 'Q':
                return { 0x0E, 0x11, 0x11, 0x11, 0x15, 0x12, 0x0D };
            case 'R':
                return { 0x1E, 0x11, 0x11, 0x1E, 0x14, 0x12, 0x11 };
            case 'S':
                return { 0x0F, 0x10, 0x10, 0x0E, 0x01, 0x01, 0x1E };
            case 'T':
                return { 0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04 };
            case 'U':
                return { 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E };
            case 'V':
                return { 0x11, 0x11, 0x11, 0x11, 0x11, 0x0A, 0x04 };
            case 'W':
                return { 0x11, 0x11, 0x11, 0x15, 0x15, 0x15, 0x0A };
            case 'X':
                return { 0x11, 0x11, 0x0A, 0x04, 0x0A, 0x11, 0x11 };
            case 'Y':
                return { 0x11, 0x11, 0x0A, 0x04, 0x04, 0x04, 0x04 };
            case 'Z':
                return { 0x1F, 0x01, 0x02, 0x04, 0x08, 0x10, 0x1F };
            case '-':
                return { 0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00 };
            case '+':
                return { 0x00, 0x04, 0x04, 0x1F, 0x04, 0x04, 0x00 };
            case '=':
                return { 0x00, 0x00, 0x1F, 0x00, 0x1F, 0x00, 0x00 };
            case '.':
                return { 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C };
            case ',':
                return { 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x08 };
            case ':':
                return { 0x00, 0x0C, 0x0C, 0x00, 0x0C, 0x0C, 0x00 };
            case '/':
                return { 0x01, 0x01, 0x02, 0x04, 0x08, 0x10, 0x10 };
            case '(':
                return { 0x02, 0x04, 0x08, 0x08, 0x08, 0x04, 0x02 };
            case ')':
                return { 0x08, 0x04, 0x02, 0x02, 0x02, 0x04, 0x08 };
            default:
                return { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
            }
        }

        void appendTextQuad(std::vector<Vertex>& vertices, float x, float y, float size, float textureWidth, float textureHeight)
        {
            if (vertices.size() + 6 > kTextVertexCapacity) {
                return;
            }

            const auto toClip = [&](float px, float py) {
                return Vertex{ (px / textureWidth) * 2.0f - 1.0f, 1.0f - (py / textureHeight) * 2.0f, 0.0f };
            };

            const Vertex a = toClip(x, y);
            const Vertex b = toClip(x + size, y);
            const Vertex c = toClip(x + size, y + size);
            const Vertex d = toClip(x, y + size);
            vertices.push_back(a);
            vertices.push_back(b);
            vertices.push_back(c);
            vertices.push_back(a);
            vertices.push_back(c);
            vertices.push_back(d);
        }

        float textPixelWidth(const TextOverlayEntry& entry)
        {
            std::size_t length = 0;
            while (length < sizeof(entry.text) && entry.text[length] != '\0') {
                ++length;
            }
            return static_cast<float>(length) * 6.0f * (std::max)(1.0f, entry.size);
        }

        bool projectWorldAnchorToScreen(const RE::NiPoint3& anchor,
            const DirectX::XMMATRIX& eyeViewProj,
            const DirectX::XMFLOAT4& adjust,
            std::uint32_t eyeIndex,
            float textureWidth,
            float textureHeight,
            float& outX,
            float& outY)
        {
            DirectX::XMFLOAT4 clip{};
            const DirectX::XMVECTOR world = DirectX::XMVectorSet(anchor.x - adjust.x, anchor.y - adjust.y, anchor.z - adjust.z, 1.0f);
            DirectX::XMStoreFloat4(&clip, DirectX::XMVector4Transform(world, eyeViewProj));
            if (!std::isfinite(clip.x) || !std::isfinite(clip.y) || !std::isfinite(clip.w) || std::fabs(clip.w) < 1.0e-5f) {
                return false;
            }

            const float invW = 1.0f / clip.w;
            const float ndcX = clip.x * invW;
            const float ndcY = clip.y * invW;
            if (clip.w < 0.0f || ndcX < -2.0f || ndcX > 2.0f || ndcY < -2.0f || ndcY > 2.0f) {
                return false;
            }

            const float halfWidth = textureWidth * 0.5f;
            const float eyeMinX = eyeIndex == 0 ? 0.0f : halfWidth;
            outX = eyeMinX + (ndcX * 0.5f + 0.5f) * halfWidth;
            outY = (-ndcY * 0.5f + 0.5f) * textureHeight;
            return true;
        }

        void appendTextGlyphs(std::vector<Vertex>& vertices, const TextOverlayEntry& entry, float baseX, float baseY, float maxX, float textureWidth, float textureHeight)
        {
            constexpr float kGlyphColumns = 5.0f;
            constexpr float kGlyphAdvanceColumns = 6.0f;
            const float pixel = (std::max)(1.0f, entry.size);
            float cursorX = baseX;
            const float cursorY = baseY;
            for (std::size_t i = 0; i < sizeof(entry.text) && entry.text[i] != '\0'; ++i) {
                const auto rows = glyphRows(entry.text[i]);
                for (std::size_t row = 0; row < rows.size(); ++row) {
                    for (std::uint8_t col = 0; col < static_cast<std::uint8_t>(kGlyphColumns); ++col) {
                        const std::uint8_t bit = static_cast<std::uint8_t>(1u << (4u - col));
                        if ((rows[row] & bit) != 0) {
                            appendTextQuad(vertices, cursorX + static_cast<float>(col) * pixel, cursorY + static_cast<float>(row) * pixel, pixel, textureWidth, textureHeight);
                        }
                    }
                }
                cursorX += kGlyphAdvanceColumns * pixel;
                if (cursorX >= maxX || cursorX >= textureWidth - 8.0f) {
                    break;
                }
            }
        }

        void appendWorldAnchoredTextGlyphs(std::vector<Vertex>& vertices,
            const TextOverlayEntry& entry,
            const DirectX::XMMATRIX& eye0,
            const DirectX::XMMATRIX& eye1,
            const DirectX::XMFLOAT4& adjust0,
            const DirectX::XMFLOAT4& adjust1,
            float textureWidth,
            float textureHeight)
        {
            const bool stereo = g_rockConfig.rockDebugGrabTransformTelemetryTextMode == 0;
            const float halfWidth = textureWidth * 0.5f;
            const float approximateWidth = textPixelWidth(entry);
            auto appendEye = [&](std::uint32_t eyeIndex, const DirectX::XMMATRIX& eye, const DirectX::XMFLOAT4& adjust) {
                float projectedX = 0.0f;
                float projectedY = 0.0f;
                if (!projectWorldAnchorToScreen(entry.worldAnchor, eye, adjust, eyeIndex, textureWidth, textureHeight, projectedX, projectedY)) {
                    return;
                }

                const float eyeMinX = eyeIndex == 0 ? 0.0f : halfWidth;
                const float eyeMaxX = eyeMinX + halfWidth;
                const float minX = eyeMinX + 24.0f;
                const float maxX = (std::max)(minX, eyeMaxX - approximateWidth - 24.0f);
                const float baseX = std::clamp(projectedX + entry.x, minX, maxX);
                const float baseY = std::clamp(projectedY + entry.y, 24.0f, (std::max)(24.0f, textureHeight - 64.0f));
                appendTextGlyphs(vertices, entry, baseX, baseY, eyeMaxX - 8.0f, textureWidth, textureHeight);
            };

            appendEye(0, eye0, adjust0);
            if (stereo) {
                appendEye(1, eye1, adjust1);
            }
        }

        void drawTextOverlays(ID3D11DeviceContext* context,
            float textureWidth,
            float textureHeight,
            const BodyOverlayFrame& frame,
            const DirectX::XMMATRIX& eye0,
            const DirectX::XMMATRIX& eye1,
            const DirectX::XMFLOAT4& adjust0,
            const DirectX::XMFLOAT4& adjust1)
        {
            if (!frame.drawText || frame.textCount == 0 || !s_textVB || !s_screenTextVertexShader || textureWidth <= 0.0f || textureHeight <= 0.0f) {
                return;
            }

            context->IASetInputLayout(s_inputLayout);
            context->VSSetShader(s_screenTextVertexShader, nullptr, 0);
            context->PSSetShader(s_pixelShader, nullptr, 0);
            context->RSSetState(s_solidRasterizer);
            context->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

            constexpr UINT stride = sizeof(Vertex);
            constexpr UINT offset = 0;
            ID3D11Buffer* vertexBuffer = s_textVB;
            context->IASetVertexBuffers(0, 1, &vertexBuffer, &stride, &offset);
            context->IASetIndexBuffer(nullptr, DXGI_FORMAT_UNKNOWN, 0);

            const bool duplicatePerEye = g_rockConfig.rockDebugGrabTransformTelemetryTextMode == 0;
            const float eyeWidth = duplicatePerEye ? textureWidth * 0.5f : textureWidth;
            for (std::uint32_t i = 0; i < frame.textCount && i < frame.textEntries.size(); ++i) {
                const auto& entry = frame.textEntries[i];
                std::vector<Vertex> vertices;
                vertices.reserve(4096);
                if (entry.worldAnchored) {
                    appendWorldAnchoredTextGlyphs(vertices, entry, eye0, eye1, adjust0, adjust1, textureWidth, textureHeight);
                } else {
                    appendTextGlyphs(vertices, entry, entry.x, entry.y, eyeWidth - 8.0f, textureWidth, textureHeight);
                    if (duplicatePerEye) {
                        appendTextGlyphs(vertices, entry, entry.x + eyeWidth, entry.y, textureWidth - 8.0f, textureWidth, textureHeight);
                    }
                }
                if (vertices.empty()) {
                    continue;
                }
                if (vertices.size() > kTextVertexCapacity) {
                    vertices.resize(kTextVertexCapacity);
                }

                D3D11_MAPPED_SUBRESOURCE mapped{};
                if (FAILED(context->Map(s_textVB, 0, D3D11_MAP_WRITE_DISCARD, 0, &mapped))) {
                    continue;
                }
                std::memcpy(mapped.pData, vertices.data(), vertices.size() * sizeof(Vertex));
                context->Unmap(s_textVB, 0);
                uploadColorModel(context, DirectX::XMMatrixIdentity(), entry.color);
                context->Draw(static_cast<UINT>(vertices.size()), 0);
            }
        }

        void drawOverlayToSubmittedTexture(const vr::Texture_t* texture)
        {
            BodyOverlayFrame frame{};
            {
                std::scoped_lock lock(s_frameMutex);
                frame = s_frame;
            }

            const bool hasBodiesToDraw = (frame.drawRockBodies || frame.drawTargetBodies) && frame.count > 0;
            const bool hasAxesToDraw = frame.drawAxes && frame.axisCount > 0;
            const bool hasMarkersToDraw = frame.drawMarkers && frame.markerCount > 0;
            const bool hasSkeletonToDraw = frame.drawSkeleton && frame.skeletonCount > 0;
            const bool hasTextToDraw = frame.drawText && frame.textCount > 0;
            if ((!hasBodiesToDraw && !hasAxesToDraw && !hasMarkersToDraw && !hasSkeletonToDraw && !hasTextToDraw) || !frame.world) {
                return;
            }

            auto* device = getDevice();
            auto* context = getContext();
            if (!device || !context || !s_initialized || !texture || !texture->handle || texture->eType != vr::TextureType_DirectX) {
                return;
            }

            const std::uint64_t overlaySettingsKey = debug_overlay_policy::makeOverlaySettingsKey(frame.drawRockBodies,
                frame.drawTargetBodies,
                frame.drawAxes,
                frame.drawMarkers,
                frame.drawSkeleton,
                frame.drawText,
                g_rockConfig.rockDebugDrawHandColliders,
                g_rockConfig.rockDebugDrawHandBoneColliders,
                g_rockConfig.rockDebugDrawWeaponColliders,
                g_rockConfig.rockDebugMaxHandBoneBodiesDrawn,
                g_rockConfig.rockDebugMaxWeaponBodiesDrawn,
                g_rockConfig.rockDebugMaxShapeGenerationsPerFrame,
                g_rockConfig.rockDebugMaxConvexSupportVertices,
                g_rockConfig.rockDebugUseBoundsForHeavyConvex);
            if (overlaySettingsKey != s_previousOverlaySettingsKey) {
                ClearShapeCache();
                s_previousOverlaySettingsKey = overlaySettingsKey;
            }

            if (reinterpret_cast<std::uintptr_t>(frame.world) != s_previousWorld) {
                ClearShapeCache();
                s_previousWorld = reinterpret_cast<std::uintptr_t>(frame.world);
            }

            auto* submittedTexture = reinterpret_cast<ID3D11Texture2D*>(texture->handle);
            D3D11_TEXTURE2D_DESC textureDesc{};
            submittedTexture->GetDesc(&textureDesc);

            ID3D11RenderTargetView* rtv = nullptr;
            if (FAILED(device->CreateRenderTargetView(submittedTexture, nullptr, &rtv)) || !rtv) {
                return;
            }

            ID3D11RenderTargetView* oldRtvs[D3D11_SIMULTANEOUS_RENDER_TARGET_COUNT] = {};
            ID3D11DepthStencilView* oldDsv = nullptr;
            context->OMGetRenderTargets(D3D11_SIMULTANEOUS_RENDER_TARGET_COUNT, oldRtvs, &oldDsv);
            UINT oldViewportCount = D3D11_VIEWPORT_AND_SCISSORRECT_OBJECT_COUNT_PER_PIPELINE;
            D3D11_VIEWPORT oldViewports[D3D11_VIEWPORT_AND_SCISSORRECT_OBJECT_COUNT_PER_PIPELINE] = {};
            context->RSGetViewports(&oldViewportCount, oldViewports);

            context->OMSetRenderTargets(1, &rtv, nullptr);
            D3D11_VIEWPORT viewport{};
            viewport.Width = static_cast<float>(textureDesc.Width);
            viewport.Height = static_cast<float>(textureDesc.Height);
            viewport.MinDepth = 0.0f;
            viewport.MaxDepth = 1.0f;
            context->RSSetViewports(1, &viewport);

            DirectX::XMMATRIX eye0;
            DirectX::XMMATRIX eye1;
            DirectX::XMFLOAT4 adjust0;
            DirectX::XMFLOAT4 adjust1;
            if (getEyeViewProjMatrices(eye0, eye1, adjust0, adjust1)) {
                beginFrame(context);
                uploadCamera(context, eye0, eye1, adjust0, adjust1);
                s_frameShapeGenerations = 0;
                OverlayRuntimeStats stats{};

                if (hasBodiesToDraw) {
                    context->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
                    for (std::uint32_t i = 0; i < frame.count && i < frame.entries.size(); i++) {
                        ++stats.bodyEntries;
                        const auto& entry = frame.entries[i];
                        const bool rockRole = entry.role == BodyOverlayRole::RightHand || entry.role == BodyOverlayRole::LeftHand ||
                            entry.role == BodyOverlayRole::RightHandSegment || entry.role == BodyOverlayRole::LeftHandSegment || entry.role == BodyOverlayRole::Weapon;
                        if ((rockRole && !frame.drawRockBodies) || (!rockRole && !frame.drawTargetBodies)) {
                            continue;
                        }

                        BodyRenderInfo body{};
                        if (!extractBody(frame.world, entry.bodyId, body)) {
                            continue;
                        }

                        const auto* gpuShape = getOrCreateShape(device, body, stats);
                        if (!gpuShape) {
                            continue;
                        }

                        UINT stride = sizeof(Vertex);
                        UINT offset = 0;
                        ID3D11Buffer* vertexBuffer = gpuShape->vertexBuffer.Get();
                        context->IASetVertexBuffers(0, 1, &vertexBuffer, &stride, &offset);
                        context->IASetIndexBuffer(gpuShape->indexBuffer.Get(), DXGI_FORMAT_R16_UINT, 0);
                        uploadModel(context, body.worldMatrix, entry.role);
                        context->DrawIndexedInstanced(gpuShape->indexCount, 2, 0, 0, 0);
                        ++stats.bodiesDrawn;
                    }
                }

                drawAxisOverlays(context, frame.world, frame);
                drawMarkerOverlays(context, frame);
                drawSkeletonOverlays(context, frame);
                drawTextOverlays(context, static_cast<float>(textureDesc.Width), static_cast<float>(textureDesc.Height), frame, eye0, eye1, adjust0, adjust1);

                if (g_rockConfig.rockDebugVerboseLogging && ++s_overlayStatsLogCounter >= 90) {
                    s_overlayStatsLogCounter = 0;
                    ROCK_LOG_DEBUG(Hand,
                        "Debug overlay frame: entries={} drawn={} axes={} markers={} skeleton={} text={} cacheHits={} cacheMisses={} shapeGenerations={} genCap={}",
                        stats.bodyEntries, stats.bodiesDrawn, frame.axisCount, frame.markerCount, frame.skeletonCount, frame.textCount, stats.shapeCacheHits, stats.shapeCacheMisses, stats.shapeGenerations,
                        debug_overlay_policy::clampShapeGenerationsPerFrame(g_rockConfig.rockDebugMaxShapeGenerationsPerFrame));
                }

                endFrame(context);
            }

            context->RSSetViewports(oldViewportCount, oldViewports);
            context->OMSetRenderTargets(D3D11_SIMULTANEOUS_RENDER_TARGET_COUNT, oldRtvs, oldDsv);

            for (auto* oldRtv : oldRtvs) {
                if (oldRtv) {
                    oldRtv->Release();
                }
            }
            if (oldDsv) {
                oldDsv->Release();
            }
            rtv->Release();
        }

        vr::EVRCompositorError VRSubmitHook(vr::IVRCompositor* compositor, vr::EVREye eye, const vr::Texture_t* texture, const vr::VRTextureBounds_t* bounds,
            vr::EVRSubmitFlags flags)
        {
            if (s_enabled.load(std::memory_order_relaxed) && eye == vr::Eye_Left) {
                drawOverlayToSubmittedTexture(texture);
            }
            return s_originalVRSubmit(compositor, eye, texture, bounds, flags);
        }

        void installSubmitHook()
        {
            if (s_submitHookInstalled) {
                return;
            }

            auto* compositor = vr::VRCompositor();
            if (!compositor) {
                ROCK_LOG_WARN(Hand, "Debug body overlay: VRCompositor unavailable; Submit hook not installed");
                return;
            }

            auto*** objectVTable = reinterpret_cast<void***>(compositor);
            s_vrCompositorVTable = *objectVTable;
            constexpr std::size_t kSubmitVTableIndex = 5;

            DWORD oldProtect = 0;
            if (!VirtualProtect(&s_vrCompositorVTable[kSubmitVTableIndex], sizeof(void*), PAGE_EXECUTE_READWRITE, &oldProtect)) {
                ROCK_LOG_WARN(Hand, "Debug body overlay: VirtualProtect failed; Submit hook not installed");
                return;
            }

            s_originalVRSubmit = reinterpret_cast<VRSubmit_t>(s_vrCompositorVTable[kSubmitVTableIndex]);
            s_vrCompositorVTable[kSubmitVTableIndex] = reinterpret_cast<void*>(&VRSubmitHook);
            VirtualProtect(&s_vrCompositorVTable[kSubmitVTableIndex], sizeof(void*), oldProtect, &oldProtect);

            s_submitHookInstalled = true;
            ROCK_LOG_INFO(Hand, "Debug body overlay: OpenVR Submit hook installed");
        }

        void renderCandidateHook(void* bsGraphicsState)
        {
            if (s_originalMainRenderCandidate) {
                s_originalMainRenderCandidate(bsGraphicsState);
            }
        }
    }

    void Install()
    {
        if (s_initialized) {
            installSubmitHook();
            return;
        }

        auto* device = getDevice();
        if (!device) {
            if (!s_installAttemptedWithoutDevice) {
                s_installAttemptedWithoutDevice = true;
                ROCK_LOG_WARN(Hand, "Debug body overlay: D3D11 device unavailable; install will retry on frame update");
            }
            return;
        }

        if (!initializeD3D(device)) {
            ROCK_LOG_ERROR(Hand, "Debug body overlay: D3D initialization failed");
            return;
        }

        auto hookAddress = REL::Offset(0xD844BC).address();
        auto& trampoline = F4SE::GetTrampoline();
        s_originalMainRenderCandidate = reinterpret_cast<MainRenderCandidate_t>(trampoline.write_call<5>(hookAddress, reinterpret_cast<std::uintptr_t>(&renderCandidateHook)));
        if (!s_originalMainRenderCandidate) {
            ROCK_LOG_ERROR(Hand, "Debug body overlay: render hook original was null");
            return;
        }

        installSubmitHook();
        s_initialized = true;
        ROCK_LOG_INFO(Hand, "Debug body overlay installed");
    }

    bool IsInstalled() { return s_initialized; }

        void PublishFrame(const BodyOverlayFrame& frame)
        {
            {
                std::scoped_lock lock(s_frameMutex);
                s_frame = frame;
            }
            const bool hasBodiesToDraw = (frame.drawRockBodies || frame.drawTargetBodies) && frame.count > 0;
            const bool hasAxesToDraw = frame.drawAxes && frame.axisCount > 0;
            const bool hasMarkersToDraw = frame.drawMarkers && frame.markerCount > 0;
            const bool hasSkeletonToDraw = frame.drawSkeleton && frame.skeletonCount > 0;
            const bool hasTextToDraw = frame.drawText && frame.textCount > 0;
            s_enabled.store(hasBodiesToDraw || hasAxesToDraw || hasMarkersToDraw || hasSkeletonToDraw || hasTextToDraw, std::memory_order_release);
        }

    void ClearFrame()
    {
        {
            std::scoped_lock lock(s_frameMutex);
            s_frame = BodyOverlayFrame{};
        }
        s_enabled.store(false, std::memory_order_release);
    }

    void ClearShapeCache() { s_shapeCache.clear(); }
}
