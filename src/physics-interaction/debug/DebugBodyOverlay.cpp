#include "physics-interaction/debug/DebugBodyOverlay.h"

#include <DirectXMath.h>
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <exception>
#include <limits>
#include <memory>
#include <mutex>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include <d3d11.h>
#include <d3dcompiler.h>
#include <wrl/client.h>

#include "physics-interaction/native/HavokOffsets.h"
#include "physics-interaction/debug/DebugConvexHullMesh.h"
#include "physics-interaction/debug/DebugOverlayFrameAdmission.h"
#include "physics-interaction/debug/DebugOverlayLineBatch.h"
#include "physics-interaction/debug/DebugOverlayPolicy.h"
#include "physics-interaction/debug/DebugOverlayShaders.h"
#include "physics-interaction/debug/DebugOverlaySnapshotPool.h"
#include "physics-interaction/PhysicsBodyFrame.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/native/PhysicsUtils.h"
#include "physics-interaction/performance/PerformanceProfiler.h"
#include "RockConfig.h"

#include "RE/Bethesda/BSGraphics.h"
#include "RE/Havok/hknpShape.h"
#include "RE/Havok/hknpWorld.h"

#include <F4SE/F4SE.h>
#include <REL/Relocation.h>
#include <vrcf/VRControllersManager.h>
#include <windows.h>

#include "physics-interaction/debug/DebugMath.h"

namespace rock::debug
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
        constexpr std::uint32_t kMaxBodyIndex = body_frame::kMaxReadableBodyIndex;
        constexpr std::uint32_t kMaxMotionIndex = 4096;
        constexpr float kRawAxisLength = 8.0f;
        constexpr float kColliderAxisLength = 12.0f;
        constexpr float kBodyAxisLength = 16.0f;
        constexpr float kTargetAxisLength = 20.0f;
        constexpr std::uint32_t kTextVertexCapacity = 131072;
        constexpr DWORD kPageExecuteReadWrite = 0x00000040u;
        constexpr UINT kMaxShaderClassInstances = 256;
        constexpr UINT kSavedVertexBufferSlots = 2;

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

        struct ShapeDecodeResult
        {
            MeshData mesh{};
            debug_overlay_policy::ShapeDecodeMode mode = debug_overlay_policy::ShapeDecodeMode::Unsupported;
            int shapeType = -1;
        };

        struct ShapeKey
        {
            std::uintptr_t shapeAddress = 0;
            std::uint64_t geometryFingerprint = 0;

            bool operator==(const ShapeKey&) const = default;
        };

        struct ShapeKeyHash
        {
            std::size_t operator()(const ShapeKey& key) const
            {
                std::size_t h = 0;
                h ^= std::hash<std::uintptr_t>{}(key.shapeAddress) + 0x9e3779b9 + (h << 6) + (h >> 2);
                h ^= std::hash<std::uint64_t>{}(key.geometryFingerprint) + 0x9e3779b9 + (h << 6) + (h >> 2);
                return h;
            }
        };

        struct GpuShape
        {
            Microsoft::WRL::ComPtr<ID3D11Buffer> vertexBuffer;
            Microsoft::WRL::ComPtr<ID3D11Buffer> indexBuffer;
            std::uint32_t indexCount = 0;
            debug_overlay_policy::ShapeDecodeMode decodeMode = debug_overlay_policy::ShapeDecodeMode::Detailed;
            int shapeType = -1;
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

        struct OverlayRenderSettings
        {
            std::uint32_t maxShapeGenerationsPerFrame{ 0 };
            int maxConvexSupportVertices{ 0 };
            std::uint64_t shapeDecodeSettingsKey{ 0 };
            bool useBoundsForHeavyConvex{ false };
            bool duplicateTextPerEye{ true };
            bool verboseLogging{ false };
        };

        struct CapturedShape
        {
            ShapeKey key{};
            ShapeDecodeResult decoded{};
            bool hasDecodedGeometry{ false };
        };

        struct PublishedBodyEntry
        {
            ShapeKey shapeKey{};
            DirectX::XMMATRIX worldMatrix = DirectX::XMMatrixIdentity();
            DirectX::XMFLOAT3 worldAabbMin{};
            DirectX::XMFLOAT3 worldAabbMax{};
            BodyOverlayRole role{ BodyOverlayRole::Target };
            std::uint32_t bodyId{ kInvalidBodyId };
            std::uint32_t capturedShapeIndex{ (std::numeric_limits<std::uint32_t>::max)() };
            bool hasValidWorldAabb{ false };
        };

        struct PublishedAxisEntry
        {
            AxisOverlayEntry entry{};
            DirectX::XMMATRIX bodyWorldMatrix = DirectX::XMMatrixIdentity();
        };

        struct PublishedOverlayFrame
        {
            std::vector<PublishedBodyEntry> bodies;
            std::vector<PublishedAxisEntry> axes;
            std::vector<MarkerOverlayEntry> markers;
            std::vector<SkeletonOverlayEntry> skeleton;
            std::vector<TextOverlayEntry> text;
            std::vector<CapturedShape> capturedShapes;
            OverlayRenderSettings settings{};
            std::uintptr_t worldIdentity{ 0 };
            std::uint32_t bodyExtractFailures{ 0 };
            std::uint32_t shapeGenerations{ 0 };
            std::uint32_t shapeGenerationDeferrals{ 0 };
            bool drawRockBodies{ false };
            bool drawTargetBodies{ false };
            bool drawAxes{ false };
            bool drawMarkers{ false };
            bool drawSkeleton{ false };
            bool drawText{ false };
        };

        enum class BodyOverlayFrameSource : std::uint8_t
        {
            LiveMotionWhenAvailable,
            BodyArrayTransform
        };

        struct OverlayRuntimeStats
        {
            std::uint32_t bodyEntries = 0;
            std::uint32_t bodiesDrawn = 0;
            std::uint32_t bodyMeshBinds = 0;
            std::uint32_t bodyDrawCalls = 0;
            std::uint32_t shapeCacheHits = 0;
            std::uint32_t shapeCacheMisses = 0;
            std::uint32_t shapeGenerations = 0;
            std::uint32_t shapeGenerationDeferrals = 0;
            std::uint32_t shapeCacheBudgetSkips = 0;
            std::uint32_t shapeProxyFallbacks = 0;
            std::uint32_t unsupportedShapeProxies = 0;
            std::uint32_t unsupportedShapeSkips = 0;
            std::uint32_t bodyExtractFailures = 0;
            std::uint32_t lineVertices = 0;
            std::uint32_t lineLogicalLines = 0;
            std::uint32_t lineDrawCalls = 0;
            std::uint32_t lineBudgetRejects = 0;
            std::uint32_t rtvCacheHits = 0;
            std::uint32_t rtvCacheMisses = 0;
            std::uint32_t textVertices = 0;
            std::uint32_t textDrawCalls = 0;
            std::uint32_t textVertexTruncations = 0;
            std::uint32_t textMapFailures = 0;
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
            ID3D11GeometryShader* gs = nullptr;
            ID3D11HullShader* hs = nullptr;
            ID3D11DomainShader* ds = nullptr;
            ID3D11ClassInstance* vsInstances[kMaxShaderClassInstances] = {};
            ID3D11ClassInstance* psInstances[kMaxShaderClassInstances] = {};
            ID3D11ClassInstance* gsInstances[kMaxShaderClassInstances] = {};
            ID3D11ClassInstance* hsInstances[kMaxShaderClassInstances] = {};
            ID3D11ClassInstance* dsInstances[kMaxShaderClassInstances] = {};
            UINT vsInstanceCount = 0;
            UINT psInstanceCount = 0;
            UINT gsInstanceCount = 0;
            UINT hsInstanceCount = 0;
            UINT dsInstanceCount = 0;
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
            ID3D11Buffer* vertexBuffers[kSavedVertexBufferSlots] = {};
            UINT vbStrides[kSavedVertexBufferSlots] = {};
            UINT vbOffsets[kSavedVertexBufferSlots] = {};
            ID3D11Buffer* indexBuffer = nullptr;
            DXGI_FORMAT ibFormat = DXGI_FORMAT_UNKNOWN;
            UINT ibOffset = 0;
        };

        struct D3DResources
        {
            Microsoft::WRL::ComPtr<ID3D11Device> device;
            Microsoft::WRL::ComPtr<ID3D11VertexShader> vertexShader;
            Microsoft::WRL::ComPtr<ID3D11VertexShader> screenTextVertexShader;
            Microsoft::WRL::ComPtr<ID3D11PixelShader> pixelShader;
            Microsoft::WRL::ComPtr<ID3D11InputLayout> inputLayout;
            Microsoft::WRL::ComPtr<ID3D11Buffer> cameraCB;
            Microsoft::WRL::ComPtr<ID3D11Buffer> modelCB;
            Microsoft::WRL::ComPtr<ID3D11Buffer> axisLineVB;
            Microsoft::WRL::ComPtr<ID3D11Buffer> textVB;
            Microsoft::WRL::ComPtr<ID3D11RasterizerState> wireRasterizer;
            Microsoft::WRL::ComPtr<ID3D11RasterizerState> solidRasterizer;
            Microsoft::WRL::ComPtr<ID3D11DepthStencilState> depthStencil;
            Microsoft::WRL::ComPtr<ID3D11BlendState> blendState;

            [[nodiscard]] bool ready() const noexcept
            {
                return device && vertexShader && screenTextVertexShader && pixelShader && inputLayout && cameraCB && modelCB && axisLineVB && textVB &&
                       wireRasterizer && solidRasterizer && depthStencil && blendState;
            }
        };

        class RenderPassGuard
        {
        public:
            RenderPassGuard(ID3D11DeviceContext* context, ID3D11RenderTargetView* renderTarget, UINT width, UINT height) noexcept;
            RenderPassGuard(const RenderPassGuard&) = delete;
            RenderPassGuard& operator=(const RenderPassGuard&) = delete;
            ~RenderPassGuard() noexcept;

            [[nodiscard]] bool active() const noexcept { return _active; }

        private:
            void restore() noexcept;

            ID3D11DeviceContext* _context{ nullptr };
            SavedState _saved{};
            bool _active{ false };
        };

        struct CachedRenderTargetView
        {
            ID3D11Texture2D* texture = nullptr;
            D3D11_TEXTURE2D_DESC desc{};
            Microsoft::WRL::ComPtr<ID3D11RenderTargetView> rtv;
        };

        constexpr std::size_t kPublishedFramePoolCapacity = 4;
        static debug_overlay_snapshot::SnapshotPool<PublishedOverlayFrame, kPublishedFramePoolCapacity> s_framePool{};
        static std::atomic<std::shared_ptr<const PublishedOverlayFrame>> s_publishedFrame{};
        static std::atomic<bool> s_enabled{ false };
        static std::atomic<bool> s_initialized{ false };
        static std::atomic<bool> s_submitHookInstalled{ false };
        static bool s_installAttemptedWithoutDevice = false;
        static std::uintptr_t s_previousWorld = 0;
        static std::uint64_t s_previousShapeDecodeSettingsKey = 0;
        static std::uint32_t s_overlayStatsLogCounter = 0;

        static D3DResources s_d3d{};
        static std::atomic_flag s_renderPassActive = ATOMIC_FLAG_INIT;
        static debug_overlay_frame_admission::FrameAdmission s_frameAdmission{};
        static std::atomic<bool> s_overlayExceptionReported{ false };
        static std::atomic<bool> s_cameraUploadFailureReported{ false };
        static std::atomic<bool> s_modelUploadFailureReported{ false };
        static std::atomic<bool> s_submitInstallFailureReported{ false };
        static std::atomic<bool> s_snapshotPoolExhaustionReported{ false };
        static std::unordered_map<ShapeKey, std::shared_ptr<const GpuShape>, ShapeKeyHash> s_shapeCache;
        static std::mutex s_shapeCacheMutex;
        static CachedRenderTargetView s_submittedTextureRtv{};

        using VRSubmit_t = vr::EVRCompositorError(__thiscall*)(vr::IVRCompositor*, vr::EVREye, const vr::Texture_t*, const vr::VRTextureBounds_t*, vr::EVRSubmitFlags);
        static std::atomic<VRSubmit_t> s_originalVRSubmit{ nullptr };
        static void** s_vrCompositorVTable = nullptr;

        Vertex sub(const Vertex& a, const Vertex& b) { return Vertex{ a.x - b.x, a.y - b.y, a.z - b.z }; }
        Vertex add(const Vertex& a, const Vertex& b) { return Vertex{ a.x + b.x, a.y + b.y, a.z + b.z }; }
        Vertex scale(const Vertex& value, float scalar) { return Vertex{ value.x * scalar, value.y * scalar, value.z * scalar }; }
        float dot(const Vertex& a, const Vertex& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
        Vertex cross(const Vertex& a, const Vertex& b) { return Vertex{ a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x }; }
        float lengthSq(const Vertex& value) { return dot(value, value); }
        Vertex toVertex(const RE::NiPoint3& value) { return Vertex{ value.x, value.y, value.z }; }
        Vertex toVertex(const debug_convex_hull_mesh::Vec3& value) { return Vertex{ value.x, value.y, value.z }; }

        std::uint32_t floatBits(float value)
        {
            std::uint32_t bits = 0;
            std::memcpy(&bits, &value, sizeof(bits));
            return bits;
        }

        std::uint64_t mixShapeFingerprint(std::uint64_t seed, std::uint64_t value)
        {
            value += 0x9e3779b97f4a7c15ull;
            value = (value ^ (value >> 30)) * 0xbf58476d1ce4e5b9ull;
            value = (value ^ (value >> 27)) * 0x94d049bb133111ebull;
            value ^= value >> 31;
            return seed ^ (value + 0x9e3779b97f4a7c15ull + (seed << 6) + (seed >> 2));
        }

        std::uint64_t mixShapeFloat(std::uint64_t seed, float value)
        {
            return mixShapeFingerprint(seed, floatBits(value));
        }

        std::uint64_t computeShapeGeometryFingerprint(std::uintptr_t shapeAddress, int depth = 0)
        {
            /*
             * Debug collider meshes are cached on the VR submit path, but
             * generated ROCK bodies are destroyed and recreated when profiles
             * such as power armor change. Havok can reuse the same shape
             * address for different support geometry, so the cache key must
             * include a small geometry fingerprint instead of trusting the
             * pointer alone.
             */
            if (!shapeAddress || depth > 4) {
                return 0;
            }

            try {
                auto* shape = reinterpret_cast<RE::hknpShape*>(shapeAddress);
                const int shapeType = static_cast<int>(shape->GetType());
                std::uint64_t fingerprint = 0xcbf29ce484222325ull;
                fingerprint = mixShapeFingerprint(fingerprint, static_cast<std::uint64_t>(shapeType));
                fingerprint = mixShapeFloat(fingerprint, *reinterpret_cast<float*>(shapeAddress + 0x14));

                switch (shapeType) {
                case 0:
                case 1:
                case 4: {
                    const int vertexCount = shape->GetNumberOfSupportVertices();
                    fingerprint = mixShapeFingerprint(fingerprint, static_cast<std::uint32_t>((std::max)(vertexCount, 0)));
                    if (vertexCount <= 0 || vertexCount > 256) {
                        return fingerprint;
                    }

                    std::vector<RE::hkVector4f> supportVertices(static_cast<std::size_t>(vertexCount));
                    auto* supportBuffer = reinterpret_cast<RE::hkcdVertex*>(supportVertices.data());
                    const auto* resultRaw = shape->GetSupportVertices(supportBuffer, vertexCount);
                    if (!resultRaw) {
                        return fingerprint;
                    }

                    const auto* resultVertices = reinterpret_cast<const RE::hkVector4f*>(resultRaw);
                    for (int i = 0; i < vertexCount; ++i) {
                        const auto* values = reinterpret_cast<const float*>(&resultVertices[i]);
                        fingerprint = mixShapeFloat(fingerprint, values[0]);
                        fingerprint = mixShapeFloat(fingerprint, values[1]);
                        fingerprint = mixShapeFloat(fingerprint, values[2]);
                    }
                    return fingerprint;
                }
                case 2:
                    return fingerprint;
                case 3: {
                    const auto* vertexA = reinterpret_cast<const float*>(shapeAddress + 0x50);
                    const auto* vertexB = reinterpret_cast<const float*>(shapeAddress + 0x60);
                    for (int i = 0; i < 3; ++i) {
                        fingerprint = mixShapeFloat(fingerprint, vertexA[i]);
                        fingerprint = mixShapeFloat(fingerprint, vertexB[i]);
                    }
                    return fingerprint;
                }
                case 11: {
                    const auto innerShape = *reinterpret_cast<std::uintptr_t*>(shapeAddress + 0x30);
                    fingerprint = mixShapeFingerprint(fingerprint, computeShapeGeometryFingerprint(innerShape, depth + 1));
                    const auto* scaleVec = reinterpret_cast<const float*>(shapeAddress + 0x38);
                    fingerprint = mixShapeFloat(fingerprint, scaleVec[0]);
                    fingerprint = mixShapeFloat(fingerprint, scaleVec[1]);
                    fingerprint = mixShapeFloat(fingerprint, scaleVec[2]);
                    return fingerprint;
                }
                default:
                    return fingerprint;
                }
            } catch (...) {
                return 0;
            }
        }

        ShapeKey makeShapeKey(std::uintptr_t shapeAddress)
        {
            return ShapeKey{ shapeAddress, computeShapeGeometryFingerprint(shapeAddress) };
        }

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

        RE::NiPoint3 storedColumnAxis(const RE::NiMatrix3& matrix, std::uint32_t column)
        {
            if (column > 2) {
                return RE::NiPoint3(0.0f, 0.0f, 1.0f);
            }

            return RE::NiPoint3(matrix.entry[0][column], matrix.entry[1][column], matrix.entry[2][column]);
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

        MeshData generateOriginProxyBox(float halfExtent = 3.0f)
        {
            return generateBoundsBox(std::vector<Vertex>{
                { -halfExtent, -halfExtent, -halfExtent },
                { halfExtent, halfExtent, halfExtent },
            });
        }

        ShapeDecodeResult makeShapeResult(MeshData mesh, debug_overlay_policy::ShapeDecodeMode mode, int shapeType)
        {
            ShapeDecodeResult result{};
            result.mesh = std::move(mesh);
            result.mode = mode;
            result.shapeType = shapeType;
            return result;
        }

        ShapeDecodeResult makeUnsupportedShapeResult(int shapeType)
        {
            ShapeDecodeResult result{};
            result.shapeType = shapeType;
            return result;
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

        ShapeDecodeResult generateConvexPolytope(std::uintptr_t shapeAddress, int shapeType, const OverlayRenderSettings& settings)
        {
            if (!shapeAddress) {
                return {};
            }

            auto* shape = reinterpret_cast<RE::hknpShape*>(shapeAddress);
            const float convexRadius = *reinterpret_cast<float*>(shapeAddress + 0x14);
            const int vertexCount = shape->GetNumberOfSupportVertices();
            if (vertexCount < 3) {
                if (settings.useBoundsForHeavyConvex) {
                    return makeShapeResult(generateOriginProxyBox(), debug_overlay_policy::ShapeDecodeMode::Proxy, shapeType);
                }
                return makeUnsupportedShapeResult(shapeType);
            }
            if (vertexCount > 256) {
                if (settings.useBoundsForHeavyConvex) {
                    return makeShapeResult(generateOriginProxyBox(), debug_overlay_policy::ShapeDecodeMode::Proxy, shapeType);
                }
                return makeUnsupportedShapeResult(shapeType);
            }

            std::vector<RE::hkVector4f> supportVertices(vertexCount);
            auto* supportBuffer = reinterpret_cast<RE::hkcdVertex*>(supportVertices.data());
            const auto* resultRaw = shape->GetSupportVertices(supportBuffer, vertexCount);
            if (!resultRaw) {
                if (settings.useBoundsForHeavyConvex) {
                    return makeShapeResult(generateOriginProxyBox(), debug_overlay_policy::ShapeDecodeMode::Proxy, shapeType);
                }
                return makeUnsupportedShapeResult(shapeType);
            }

            const auto* resultVertices = reinterpret_cast<const RE::hkVector4f*>(resultRaw);
            std::vector<debug_convex_hull_mesh::Vec3> rawVertices;
            rawVertices.reserve(vertexCount);

            for (int i = 0; i < vertexCount; i++) {
                const float* values = reinterpret_cast<const float*>(&resultVertices[i]);
                rawVertices.push_back(debug_convex_hull_mesh::Vec3{ values[0] * havokToGameScale(), values[1] * havokToGameScale(), values[2] * havokToGameScale() });
            }

            auto hullVertices = debug_convex_hull_mesh::deduplicateVertices(rawVertices);
            if (hullVertices.size() < 3) {
                if (settings.useBoundsForHeavyConvex) {
                    return makeShapeResult(generateOriginProxyBox(), debug_overlay_policy::ShapeDecodeMode::Proxy, shapeType);
                }
                return makeUnsupportedShapeResult(shapeType);
            }

            const auto decodeMode = debug_overlay_policy::chooseShapeDecodeMode(shapeType,
                    static_cast<std::uint32_t>(hullVertices.size()),
                    settings.maxConvexSupportVertices,
                    settings.useBoundsForHeavyConvex);
            if (decodeMode == debug_overlay_policy::ShapeDecodeMode::Proxy) {
                std::vector<Vertex> boundsVertices;
                boundsVertices.reserve(hullVertices.size());
                for (const auto& vertex : hullVertices) {
                    boundsVertices.push_back(toVertex(vertex));
                }

                ROCK_LOG_DEBUG(Hand, "Debug overlay using bounds LOD for heavy convex: shape=0x{:X} supportVertices={} cap={}", shapeAddress, hullVertices.size(),
                    settings.maxConvexSupportVertices);
                return makeShapeResult(generateBoundsBox(boundsVertices), decodeMode, shapeType);
            }

            if (convexRadius > 0.0f) {
                debug_convex_hull_mesh::inflateVerticesFromCentroid(hullVertices, convexRadius * havokToGameScale());
            }

            const auto triangles = debug_convex_hull_mesh::triangulateConvexHullFaces(hullVertices);
            if (triangles.empty() || triangles.size() > debug_convex_hull_mesh::maxExpectedConvexHullTriangles(static_cast<std::uint32_t>(hullVertices.size()))) {
                if (settings.useBoundsForHeavyConvex) {
                    std::vector<Vertex> boundsVertices;
                    boundsVertices.reserve(hullVertices.size());
                    for (const auto& vertex : hullVertices) {
                        boundsVertices.push_back(toVertex(vertex));
                    }
                    return makeShapeResult(generateBoundsBox(boundsVertices), debug_overlay_policy::ShapeDecodeMode::Proxy, shapeType);
                }
                return makeUnsupportedShapeResult(shapeType);
            }

            MeshData mesh;
            for (const auto& triangle : triangles) {
                mesh.indices.push_back(triangle[0]);
                mesh.indices.push_back(triangle[1]);
                mesh.indices.push_back(triangle[2]);
            }

            mesh.vertices.reserve(hullVertices.size());
            for (const auto& vertex : hullVertices) {
                mesh.vertices.push_back(toVertex(vertex));
            }
            mesh.valid = !mesh.vertices.empty() && !mesh.indices.empty();
            return makeShapeResult(std::move(mesh), debug_overlay_policy::ShapeDecodeMode::Detailed, shapeType);
        }

        ShapeDecodeResult generateShape(std::uintptr_t shapeAddress, const OverlayRenderSettings& settings)
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
                    return generateConvexPolytope(shapeAddress, shapeType, settings);
                case 2:
                    return makeShapeResult(generateSphere(*reinterpret_cast<float*>(shapeAddress + 0x14)), debug_overlay_policy::ShapeDecodeMode::Detailed, shapeType);
                case 3:
                    return makeShapeResult(generateCapsule(reinterpret_cast<const float*>(shapeAddress + 0x50), reinterpret_cast<const float*>(shapeAddress + 0x60),
                                               *reinterpret_cast<float*>(shapeAddress + 0x14)),
                        debug_overlay_policy::ShapeDecodeMode::Detailed,
                        shapeType);
                case 4:
                    return generateConvexPolytope(shapeAddress, shapeType, settings);
                case 11: {
                    const auto innerShape = *reinterpret_cast<std::uintptr_t*>(shapeAddress + 0x30);
                    if (!innerShape) {
                        return {};
                    }
                    ShapeDecodeResult inner = generateShape(innerShape, settings);
                    if (!inner.mesh.valid) {
                        inner.shapeType = shapeType;
                        return inner;
                    }
                    const auto* scaleVec = reinterpret_cast<const float*>(shapeAddress + 0x38);
                    for (auto& vertex : inner.mesh.vertices) {
                        vertex.x *= scaleVec[0];
                        vertex.y *= scaleVec[1];
                        vertex.z *= scaleVec[2];
                    }
                    inner.shapeType = shapeType;
                    return inner;
                }
                default:
                    if (debug_overlay_policy::chooseShapeDecodeMode(shapeType, 0, settings.maxConvexSupportVertices, settings.useBoundsForHeavyConvex) ==
                        debug_overlay_policy::ShapeDecodeMode::Proxy) {
                        return makeShapeResult(generateOriginProxyBox(), debug_overlay_policy::ShapeDecodeMode::Proxy, shapeType);
                    }
                    return makeUnsupportedShapeResult(shapeType);
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
            const DirectX::XMMATRIX rotation = DirectX::XMMatrixSet(transform[0], transform[1], transform[2], 0.0f, transform[4], transform[5], transform[6], 0.0f,
                transform[8], transform[9], transform[10], 0.0f, 0.0f, 0.0f, 0.0f, 1.0f);
            const DirectX::XMMATRIX translation =
                DirectX::XMMatrixTranslation(transform[12] * havokToGameScale(), transform[13] * havokToGameScale(), transform[14] * havokToGameScale());
            return DirectX::XMMatrixMultiply(rotation, translation);
        }

        BodyOverlayFrameSource targetBodyOverlayFrameSource(BodyOverlayRole role)
        {
            /*
             * Target colliders are grab diagnostics. Draw them from the hknp
             * BODY array transform, matching body-B authority, instead of the
             * generic live MOTION/COM frame used by non-target body readback.
             */
            return role == BodyOverlayRole::Target ?
                       BodyOverlayFrameSource::BodyArrayTransform :
                       BodyOverlayFrameSource::LiveMotionWhenAvailable;
        }

        BodyOverlayFrameSource targetAxisOverlayFrameSource(AxisOverlayRole role)
        {
            return role == AxisOverlayRole::TargetBody ?
                       BodyOverlayFrameSource::BodyArrayTransform :
                       BodyOverlayFrameSource::LiveMotionWhenAvailable;
        }

        bool extractBody(RE::hknpWorld* world, RE::hknpBodyId bodyId, BodyOverlayFrameSource frameSource, BodyRenderInfo& out)
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

            if (frameSource == BodyOverlayFrameSource::BodyArrayTransform) {
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

        bool captureBodyWorldAabb(
            RE::hknpWorld* world, RE::hknpBodyId bodyId, DirectX::XMFLOAT3& outMin, DirectX::XMFLOAT3& outMax)
        {
            // FO4VR 1.2.72 function 0x141539120 (CommonLibF4VR ID 249572)
            // zero-extends the body's eight compressed 16-bit AABB components,
            // decompresses them in Havok space, then writes min[0..3] followed
            // by max[0..3]. Keep this engine wrapper instead of duplicating the
            // version-sensitive raw world/body offsets or signed decoding.
            struct alignas(16) RawBodyAabb
            {
                float minimum[4]{};
                float maximum[4]{};
            };
            static_assert(sizeof(RawBodyAabb) == 32);

            if (!world || bodyId.value == kInvalidBodyId || bodyId.value > kMaxBodyIndex) {
                return false;
            }

            RawBodyAabb raw{};
            world->GetBodyAabb(bodyId, &raw);
            const float scale = havokToGameScale();
            const float minX = raw.minimum[0] * scale;
            const float minY = raw.minimum[1] * scale;
            const float minZ = raw.minimum[2] * scale;
            const float maxX = raw.maximum[0] * scale;
            const float maxY = raw.maximum[1] * scale;
            const float maxZ = raw.maximum[2] * scale;
            if (!std::isfinite(minX) || !std::isfinite(minY) || !std::isfinite(minZ) ||
                !std::isfinite(maxX) || !std::isfinite(maxY) || !std::isfinite(maxZ) ||
                minX > maxX || minY > maxY || minZ > maxZ) {
                return false;
            }

            outMin = DirectX::XMFLOAT3{ minX, minY, minZ };
            outMax = DirectX::XMFLOAT3{ maxX, maxY, maxZ };
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

        bool isShapeCached(const ShapeKey& key)
        {
            std::scoped_lock lock(s_shapeCacheMutex);
            return s_shapeCache.contains(key);
        }

        std::shared_ptr<const GpuShape> getOrCreateShape(
            ID3D11Device* device, const ShapeKey& key, const ShapeDecodeResult* decoded, OverlayRuntimeStats& stats)
        {
            std::scoped_lock lock(s_shapeCacheMutex);
            auto it = s_shapeCache.find(key);
            if (it != s_shapeCache.end()) {
                ++stats.shapeCacheHits;
                return it->second && it->second->indexCount > 0 ? it->second : std::shared_ptr<const GpuShape>{};
            }

            ++stats.shapeCacheMisses;
            if (!decoded) {
                ++stats.shapeGenerationDeferrals;
                return {};
            }

            const std::uint32_t shapeCacheBudget = debug_overlay_policy::clampShapeCacheBudget(static_cast<int>(debug_overlay_policy::kDefaultShapeCacheBudget));
            if (s_shapeCache.size() >= shapeCacheBudget) {
                ++stats.shapeCacheBudgetSkips;
                return {};
            }

            if (decoded->mode == debug_overlay_policy::ShapeDecodeMode::Proxy) {
                ++stats.shapeProxyFallbacks;
                if (!debug_overlay_policy::isSupportedDetailedShapeType(decoded->shapeType)) {
                    ++stats.unsupportedShapeProxies;
                }
            } else if (decoded->mode == debug_overlay_policy::ShapeDecodeMode::Unsupported) {
                ++stats.unsupportedShapeSkips;
            }

            auto gpuShape = std::make_shared<GpuShape>();
            gpuShape->decodeMode = decoded->mode;
            gpuShape->shapeType = decoded->shapeType;
            if (!decoded->mesh.valid || decoded->mesh.vertices.empty() || decoded->mesh.indices.empty() || !createBuffers(device, decoded->mesh, *gpuShape)) {
                s_shapeCache.emplace(key, gpuShape);
                return {};
            }

            auto [inserted, _] = s_shapeCache.emplace(key, gpuShape);
            return inserted->second;
        }

        bool isRockBodyRole(BodyOverlayRole role)
        {
            return role == BodyOverlayRole::RightHand || role == BodyOverlayRole::LeftHand ||
                   role == BodyOverlayRole::RightHandSegment || role == BodyOverlayRole::LeftHandSegment ||
                   role == BodyOverlayRole::BodyTorsoSegment || role == BodyOverlayRole::BodyArmSegment ||
                   role == BodyOverlayRole::BodyLegSegment || role == BodyOverlayRole::BodyFootSegment ||
                   role == BodyOverlayRole::Weapon ||
                   role == BodyOverlayRole::RightGrabAuthorityProxy ||
                   role == BodyOverlayRole::LeftGrabAuthorityProxy ||
                   role == BodyOverlayRole::RightGrabPivotSourceCollider ||
                   role == BodyOverlayRole::LeftGrabPivotSourceCollider;
        }

        OverlayRenderSettings captureOverlayRenderSettings()
        {
            OverlayRenderSettings settings{};
            settings.maxShapeGenerationsPerFrame =
                debug_overlay_policy::clampShapeGenerationsPerFrame(g_rockConfig.rockDebugMaxShapeGenerationsPerFrame);
            settings.maxConvexSupportVertices =
                static_cast<int>(debug_overlay_policy::clampMaxConvexSupportVertices(g_rockConfig.rockDebugMaxConvexSupportVertices));
            settings.useBoundsForHeavyConvex = g_rockConfig.rockDebugUseBoundsForHeavyConvex;
            settings.shapeDecodeSettingsKey = debug_overlay_policy::makeShapeDecodeSettingsKey(
                settings.maxConvexSupportVertices,
                settings.useBoundsForHeavyConvex);
            settings.duplicateTextPerEye = g_rockConfig.rockDebugGrabTransformTelemetryTextMode == 0;
            settings.verboseLogging = g_rockConfig.rockDebugVerboseLogging;
            return settings;
        }

        void resetPublishedFrame(PublishedOverlayFrame& frame)
        {
            frame.bodies.clear();
            frame.axes.clear();
            frame.markers.clear();
            frame.skeleton.clear();
            frame.text.clear();
            frame.capturedShapes.clear();
            frame.settings = {};
            frame.worldIdentity = 0;
            frame.bodyExtractFailures = 0;
            frame.shapeGenerations = 0;
            frame.shapeGenerationDeferrals = 0;
            frame.drawRockBodies = false;
            frame.drawTargetBodies = false;
            frame.drawAxes = false;
            frame.drawMarkers = false;
            frame.drawSkeleton = false;
            frame.drawText = false;
        }

        std::uint32_t captureShapeForFrame(PublishedOverlayFrame& frame, const ShapeKey& key, std::uintptr_t shapeAddress)
        {
            for (std::uint32_t index = 0; index < frame.capturedShapes.size(); ++index) {
                if (frame.capturedShapes[index].key == key) {
                    return index;
                }
            }

            CapturedShape captured{};
            captured.key = key;
            if (!isShapeCached(key)) {
                if (frame.shapeGenerations < frame.settings.maxShapeGenerationsPerFrame) {
                    captured.decoded = generateShape(shapeAddress, frame.settings);
                    captured.hasDecodedGeometry = true;
                    ++frame.shapeGenerations;
                } else {
                    ++frame.shapeGenerationDeferrals;
                }
            }

            const auto index = static_cast<std::uint32_t>(frame.capturedShapes.size());
            frame.capturedShapes.push_back(std::move(captured));
            return index;
        }

        bool buildPublishedFrame(const BodyOverlayFrame& source, PublishedOverlayFrame& destination)
        {
            resetPublishedFrame(destination);
            destination.settings = captureOverlayRenderSettings();
            destination.worldIdentity = reinterpret_cast<std::uintptr_t>(source.world);
            if (!destination.worldIdentity) {
                return false;
            }

            if (destination.worldIdentity != s_previousWorld ||
                destination.settings.shapeDecodeSettingsKey != s_previousShapeDecodeSettingsKey) {
                ClearShapeCache();
                s_previousWorld = destination.worldIdentity;
                s_previousShapeDecodeSettingsKey = destination.settings.shapeDecodeSettingsKey;
            }

            destination.drawRockBodies = source.drawRockBodies;
            destination.drawTargetBodies = source.drawTargetBodies;
            destination.drawAxes = source.drawAxes;
            destination.drawMarkers = source.drawMarkers;
            destination.drawSkeleton = source.drawSkeleton;
            destination.drawText = source.drawText;

            const auto bodyCount = (std::min)(source.count, static_cast<std::uint32_t>(source.entries.size()));
            destination.bodies.reserve(source.entries.size());
            destination.capturedShapes.reserve(source.entries.size());
            for (std::uint32_t index = 0; index < bodyCount; ++index) {
                const auto& entry = source.entries[index];
                const bool rockRole = isRockBodyRole(entry.role);
                if ((rockRole && !source.drawRockBodies) || (!rockRole && !source.drawTargetBodies)) {
                    continue;
                }

                BodyRenderInfo body{};
                if (!extractBody(source.world, entry.bodyId, targetBodyOverlayFrameSource(entry.role), body)) {
                    ++destination.bodyExtractFailures;
                    continue;
                }

                PublishedBodyEntry published{};
                published.shapeKey = makeShapeKey(body.shapeAddress);
                published.worldMatrix = body.worldMatrix;
                published.role = entry.role;
                published.bodyId = body.bodyId;
                published.hasValidWorldAabb =
                    captureBodyWorldAabb(source.world, entry.bodyId, published.worldAabbMin, published.worldAabbMax);
                published.capturedShapeIndex = captureShapeForFrame(destination, published.shapeKey, body.shapeAddress);
                destination.bodies.push_back(std::move(published));
            }

            const auto axisCount = (std::min)(source.axisCount, static_cast<std::uint32_t>(source.axisEntries.size()));
            destination.axes.reserve(source.axisEntries.size());
            if (source.drawAxes) {
                for (std::uint32_t index = 0; index < axisCount; ++index) {
                    PublishedAxisEntry published{};
                    published.entry = source.axisEntries[index];
                    if (published.entry.source == AxisOverlaySource::Body) {
                        BodyRenderInfo body{};
                        if (!extractBody(source.world, published.entry.bodyId, targetAxisOverlayFrameSource(published.entry.role), body)) {
                            ++destination.bodyExtractFailures;
                            continue;
                        }
                        published.bodyWorldMatrix = body.worldMatrix;
                    }
                    destination.axes.push_back(std::move(published));
                }
            }

            if (source.drawMarkers) {
                const auto count = (std::min)(source.markerCount, static_cast<std::uint32_t>(source.markerEntries.size()));
                destination.markers.assign(source.markerEntries.begin(), source.markerEntries.begin() + count);
            }
            if (source.drawSkeleton) {
                const auto count = (std::min)(source.skeletonCount, static_cast<std::uint32_t>(source.skeletonEntries.size()));
                destination.skeleton.assign(source.skeletonEntries.begin(), source.skeletonEntries.begin() + count);
            }
            if (source.drawText) {
                const auto count = (std::min)(source.textCount, static_cast<std::uint32_t>(source.textEntries.size()));
                destination.text.assign(source.textEntries.begin(), source.textEntries.begin() + count);
            }

            return !destination.bodies.empty() || !destination.axes.empty() || !destination.markers.empty() ||
                   !destination.skeleton.empty() || !destination.text.empty();
        }

        template <class T>
        void releaseSavedComReference(T*& value) noexcept
        {
            if (value) {
                value->Release();
                value = nullptr;
            }
        }

        template <std::size_t N>
        void releaseSavedClassInstances(ID3D11ClassInstance* (&instances)[N], UINT count) noexcept
        {
            const UINT boundedCount = (std::min)(count, static_cast<UINT>(N));
            for (UINT i = 0; i < boundedCount; ++i) {
                releaseSavedComReference(instances[i]);
            }
        }

        void releaseSavedState(SavedState& saved) noexcept
        {
            releaseSavedComReference(saved.vs);
            releaseSavedComReference(saved.ps);
            releaseSavedComReference(saved.gs);
            releaseSavedComReference(saved.hs);
            releaseSavedComReference(saved.ds);
            releaseSavedClassInstances(saved.vsInstances, saved.vsInstanceCount);
            releaseSavedClassInstances(saved.psInstances, saved.psInstanceCount);
            releaseSavedClassInstances(saved.gsInstances, saved.gsInstanceCount);
            releaseSavedClassInstances(saved.hsInstances, saved.hsInstanceCount);
            releaseSavedClassInstances(saved.dsInstances, saved.dsInstanceCount);
            for (auto*& cb : saved.vsCBs) {
                releaseSavedComReference(cb);
            }
            releaseSavedComReference(saved.inputLayout);
            releaseSavedComReference(saved.rasterizerState);
            releaseSavedComReference(saved.depthStencilState);
            releaseSavedComReference(saved.blendState);
            for (auto*& rtv : saved.rtvs) {
                releaseSavedComReference(rtv);
            }
            releaseSavedComReference(saved.dsv);
            for (auto*& vertexBuffer : saved.vertexBuffers) {
                releaseSavedComReference(vertexBuffer);
            }
            releaseSavedComReference(saved.indexBuffer);
            saved = SavedState{};
        }

        // FO4VR stereo state layout. The +0x2590/+0x25A0 pair is the CURRENT-frame left/right
        // eye-origin adjustment; the engine's own per-frame stereo constant-buffer upload copies
        // +0x2590/+0x25A0/+0x25B0/+0x25C0 as one four-slot family, with +0x25B0/+0x25C0 holding
        // the PREVIOUS-frame origins for temporal reprojection. Reading +0x25C0 as the right-eye
        // origin lags one frame of room translation and stutters under stick locomotion.
        constexpr std::uintptr_t kVrRuntimeRootRva = 0x6235AC8;
        constexpr std::uintptr_t kRootStereoSlot0OriginOffset = 0x2590;
        constexpr std::uintptr_t kRootStereoSlot1OriginOffset = 0x25A0;
        constexpr std::uintptr_t kRootStereoRecordsDataOffset = 0x25D0;
        constexpr std::uintptr_t kStereoRecordStride = 0x210;
        constexpr std::uintptr_t kStereoRecordCompositeOffset = 0xD0;
        constexpr std::uintptr_t kStereoSlot0CompositeOffset = kStereoRecordCompositeOffset;
        constexpr std::uintptr_t kStereoSlot1CompositeOffset = kStereoRecordStride + kStereoRecordCompositeOffset;
        constexpr std::size_t kStereoRecordsReadSize = kStereoSlot1CompositeOffset + sizeof(DirectX::XMFLOAT4X4);
        constexpr float kStereoMaximumReasonableMagnitude = 1.0e8f;

        static_assert(kRootStereoSlot1OriginOffset - kRootStereoSlot0OriginOffset == 0x10);
        static_assert(kRootStereoRecordsDataOffset - kRootStereoSlot0OriginOffset == 0x40);
        static_assert(kStereoSlot1CompositeOffset == 0x2E0);

        // One contiguous read of the root stereo fields starting at +0x2590.
        struct RootStereoFields
        {
            float slot0Origin[4]{};
            float slot1Origin[4]{};
            std::byte reserved[0x20]{};
            std::uintptr_t recordsData = 0;
        };

        static_assert(offsetof(RootStereoFields, slot1Origin) == 0x10);
        static_assert(offsetof(RootStereoFields, recordsData) == 0x40);
        static_assert(sizeof(RootStereoFields) == 0x48);

        enum class StereoCaptureStage : std::uint8_t
        {
            None,
            RelocationResolved,
            RuntimeRootRead,
            RootStereoStateRead,
            StereoRecordsRead,
            Validated,
        };

        constexpr const char* stereoStageName(StereoCaptureStage stage) noexcept
        {
            switch (stage) {
            case StereoCaptureStage::None:
                return "none";
            case StereoCaptureStage::RelocationResolved:
                return "relocation-resolved";
            case StereoCaptureStage::RuntimeRootRead:
                return "runtime-root-read";
            case StereoCaptureStage::RootStereoStateRead:
                return "root-stereo-state-read";
            case StereoCaptureStage::StereoRecordsRead:
                return "stereo-records-read";
            case StereoCaptureStage::Validated:
                return "validated";
            }
            return "unknown";
        }

        std::atomic<StereoCaptureStage> s_stereoLastStage{ StereoCaptureStage::None };
        std::atomic<bool> s_stereoLastCaptureSucceeded{ true };
        std::atomic<std::int64_t> s_stereoLastFailureLogMilliseconds{ 0 };

        [[nodiscard]] bool isStereoPlausiblePointer(std::uintptr_t address) noexcept
        {
            constexpr std::uintptr_t kMinimumUserAddress = 0x10000;
            constexpr std::uintptr_t kMaximumUserAddress = 0x00007FFFFFFFFFFF;
            return address >= kMinimumUserAddress && address <= kMaximumUserAddress && (address % alignof(void*)) == 0;
        }

        // ReadProcessMemory instead of raw dereference: the engine owns these pointers and can
        // retire them between frames; a stale pointer must degrade into a skipped overlay frame,
        // never a crash inside the compositor submit hook.
        [[nodiscard]] bool readStereoMemory(std::uintptr_t address, void* destination, std::size_t size, DWORD& error) noexcept
        {
            if (!isStereoPlausiblePointer(address) || !destination || size == 0 || address > (std::numeric_limits<std::uintptr_t>::max)() - size) {
                error = ERROR_INVALID_ADDRESS;
                return false;
            }

            SIZE_T bytesRead = 0;
            if (!ReadProcessMemory(GetCurrentProcess(), reinterpret_cast<const void*>(address), destination, size, &bytesRead) || bytesRead != size) {
                error = GetLastError();
                if (error == ERROR_SUCCESS) {
                    error = ERROR_PARTIAL_COPY;
                }
                return false;
            }
            error = ERROR_SUCCESS;
            return true;
        }

        [[nodiscard]] bool validateStereoVector3(const float* value) noexcept
        {
            if (!value) {
                return false;
            }
            for (std::size_t index = 0; index < 3; ++index) {
                if (!std::isfinite(value[index]) || std::fabs(value[index]) > kStereoMaximumReasonableMagnitude) {
                    return false;
                }
            }
            return true;
        }

        [[nodiscard]] bool validateStereoMatrix(const DirectX::XMFLOAT4X4& matrix) noexcept
        {
            const auto* values = reinterpret_cast<const float*>(&matrix);
            bool hasNonZeroElement = false;
            for (std::size_t index = 0; index < 16; ++index) {
                if (!std::isfinite(values[index]) || std::fabs(values[index]) > kStereoMaximumReasonableMagnitude) {
                    return false;
                }
                hasNonZeroElement = hasNonZeroElement || std::fabs(values[index]) > 1.0e-7f;
            }
            return hasNonZeroElement;
        }

        void reportStereoCaptureFailure(StereoCaptureStage deepestStage, DWORD error) noexcept
        {
            const auto now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
            const auto previousStage = s_stereoLastStage.exchange(deepestStage, std::memory_order_relaxed);
            const auto previousLog = s_stereoLastFailureLogMilliseconds.load(std::memory_order_relaxed);
            const bool stageChanged = previousStage != deepestStage;
            const bool intervalElapsed = now - previousLog >= 5000;
            s_stereoLastCaptureSucceeded.store(false, std::memory_order_relaxed);
            if (stageChanged || intervalElapsed) {
                s_stereoLastFailureLogMilliseconds.store(now, std::memory_order_relaxed);
                ROCK_LOG_WARN(Hand, "Debug overlay stereo snapshot unavailable; deepest stage='{}', Win32 error={}. Overlay frame skipped.",
                    stereoStageName(deepestStage), error);
            }
        }

        void reportStereoCaptureSuccess() noexcept
        {
            const bool previouslySucceeded = s_stereoLastCaptureSucceeded.exchange(true, std::memory_order_relaxed);
            s_stereoLastStage.store(StereoCaptureStage::Validated, std::memory_order_relaxed);
            if (!previouslySucceeded) {
                ROCK_LOG_INFO(Hand, "Debug overlay stereo snapshot recovered and validated.");
            }
        }

        // Shared root-stereo read for the overlay camera and the diagnostic origin
        // accessor. On failure returns false with the deepest stage reached and the
        // Win32 error; callers decide whether to feed the stereo health telemetry.
        bool readRootStereoFields(RootStereoFields& outFields, StereoCaptureStage& deepestStage, DWORD& readError)
        {
            deepestStage = StereoCaptureStage::None;
            readError = ERROR_SUCCESS;

            static REL::Relocation<std::uintptr_t> rootAddress{ REL::Offset(kVrRuntimeRootRva) };
            const std::uintptr_t relocationAddress = rootAddress.address();
            if (!isStereoPlausiblePointer(relocationAddress)) {
                readError = ERROR_INVALID_ADDRESS;
                return false;
            }
            deepestStage = StereoCaptureStage::RelocationResolved;

            std::uintptr_t runtimeRoot = 0;
            if (!readStereoMemory(relocationAddress, &runtimeRoot, sizeof(runtimeRoot), readError) || !isStereoPlausiblePointer(runtimeRoot)) {
                if (readError == ERROR_SUCCESS) {
                    readError = ERROR_INVALID_ADDRESS;
                }
                return false;
            }
            deepestStage = StereoCaptureStage::RuntimeRootRead;

            if (!readStereoMemory(runtimeRoot + kRootStereoSlot0OriginOffset, &outFields, sizeof(outFields), readError)) {
                return false;
            }
            deepestStage = StereoCaptureStage::RootStereoStateRead;
            return true;
        }

        bool getEyeViewProjMatrices(DirectX::XMMATRIX& outEye0, DirectX::XMMATRIX& outEye1, DirectX::XMFLOAT4& outAdjust0, DirectX::XMFLOAT4& outAdjust1)
        {
            StereoCaptureStage deepestStage = StereoCaptureStage::None;
            DWORD readError = ERROR_SUCCESS;

            RootStereoFields rootFields{};
            if (!readRootStereoFields(rootFields, deepestStage, readError)) {
                reportStereoCaptureFailure(deepestStage, readError);
                return false;
            }
            if (!isStereoPlausiblePointer(rootFields.recordsData)) {
                reportStereoCaptureFailure(deepestStage, ERROR_INVALID_ADDRESS);
                return false;
            }

            std::array<std::byte, kStereoRecordsReadSize> records{};
            if (!readStereoMemory(rootFields.recordsData, records.data(), records.size(), readError)) {
                reportStereoCaptureFailure(deepestStage, readError);
                return false;
            }
            deepestStage = StereoCaptureStage::StereoRecordsRead;

            DirectX::XMFLOAT4X4 composite0{};
            DirectX::XMFLOAT4X4 composite1{};
            std::memcpy(&composite0, records.data() + kStereoSlot0CompositeOffset, sizeof(composite0));
            std::memcpy(&composite1, records.data() + kStereoSlot1CompositeOffset, sizeof(composite1));

            if (!validateStereoVector3(rootFields.slot0Origin) || !validateStereoVector3(rootFields.slot1Origin) ||
                !validateStereoMatrix(composite0) || !validateStereoMatrix(composite1)) {
                reportStereoCaptureFailure(deepestStage, ERROR_INVALID_DATA);
                return false;
            }

            outEye0 = DirectX::XMLoadFloat4x4(&composite0);
            outEye1 = DirectX::XMLoadFloat4x4(&composite1);
            outAdjust0 = DirectX::XMFLOAT4(rootFields.slot0Origin[0], rootFields.slot0Origin[1], rootFields.slot0Origin[2], 0.0f);
            outAdjust1 = DirectX::XMFLOAT4(rootFields.slot1Origin[0], rootFields.slot1Origin[1], rootFields.slot1Origin[2], 0.0f);
            reportStereoCaptureSuccess();
            return true;
        }

        bool initializeD3D(ID3D11Device* device)
        {
            if (!device) {
                return false;
            }

            D3DResources resources{};
            resources.device = device;

            Microsoft::WRL::ComPtr<ID3DBlob> vsBlob;
            Microsoft::WRL::ComPtr<ID3DBlob> psBlob;
            Microsoft::WRL::ComPtr<ID3DBlob> errorBlob;
            HRESULT hr = D3DCompile(
                debug_overlay_shaders::kStereoVertex,
                sizeof(debug_overlay_shaders::kStereoVertex) - 1,
                "ROCKDebugBodyVS",
                nullptr,
                nullptr,
                "main",
                "vs_5_0",
                D3DCOMPILE_ENABLE_STRICTNESS | D3DCOMPILE_PACK_MATRIX_COLUMN_MAJOR,
                0,
                vsBlob.GetAddressOf(),
                errorBlob.GetAddressOf());
            if (FAILED(hr)) {
                if (errorBlob) {
                    ROCK_LOG_ERROR(Hand, "Debug overlay vertex shader compile failed: {}", static_cast<const char*>(errorBlob->GetBufferPointer()));
                }
                return false;
            }

            hr = device->CreateVertexShader(vsBlob->GetBufferPointer(), vsBlob->GetBufferSize(), nullptr, resources.vertexShader.GetAddressOf());
            if (FAILED(hr)) {
                return false;
            }

            D3D11_INPUT_ELEMENT_DESC layoutDesc[] = { { "POS", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0 } };
            hr = device->CreateInputLayout(layoutDesc, 1, vsBlob->GetBufferPointer(), vsBlob->GetBufferSize(), resources.inputLayout.GetAddressOf());
            if (FAILED(hr)) {
                return false;
            }

            vsBlob.Reset();
            errorBlob.Reset();
            hr = D3DCompile(
                debug_overlay_shaders::kScreenTextVertex,
                sizeof(debug_overlay_shaders::kScreenTextVertex) - 1,
                "ROCKDebugTextVS",
                nullptr,
                nullptr,
                "main",
                "vs_5_0",
                D3DCOMPILE_ENABLE_STRICTNESS | D3DCOMPILE_PACK_MATRIX_COLUMN_MAJOR,
                0,
                vsBlob.GetAddressOf(),
                errorBlob.GetAddressOf());
            if (FAILED(hr)) {
                if (errorBlob) {
                    ROCK_LOG_ERROR(Hand, "Debug overlay text vertex shader compile failed: {}", static_cast<const char*>(errorBlob->GetBufferPointer()));
                }
                return false;
            }

            hr = device->CreateVertexShader(vsBlob->GetBufferPointer(), vsBlob->GetBufferSize(), nullptr, resources.screenTextVertexShader.GetAddressOf());
            if (FAILED(hr)) {
                return false;
            }

            errorBlob.Reset();
            hr = D3DCompile(
                debug_overlay_shaders::kPixel,
                sizeof(debug_overlay_shaders::kPixel) - 1,
                "ROCKDebugBodyPS",
                nullptr,
                nullptr,
                "main",
                "ps_5_0",
                D3DCOMPILE_ENABLE_STRICTNESS | D3DCOMPILE_PACK_MATRIX_COLUMN_MAJOR,
                0,
                psBlob.GetAddressOf(),
                errorBlob.GetAddressOf());
            if (FAILED(hr)) {
                if (errorBlob) {
                    ROCK_LOG_ERROR(Hand, "Debug overlay pixel shader compile failed: {}", static_cast<const char*>(errorBlob->GetBufferPointer()));
                }
                return false;
            }

            hr = device->CreatePixelShader(psBlob->GetBufferPointer(), psBlob->GetBufferSize(), nullptr, resources.pixelShader.GetAddressOf());
            if (FAILED(hr)) {
                return false;
            }

            D3D11_BUFFER_DESC cameraDesc{};
            cameraDesc.Usage = D3D11_USAGE_DYNAMIC;
            cameraDesc.ByteWidth = sizeof(PerFrameVSData);
            cameraDesc.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
            cameraDesc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
            if (FAILED(device->CreateBuffer(&cameraDesc, nullptr, resources.cameraCB.GetAddressOf()))) {
                return false;
            }

            D3D11_BUFFER_DESC modelDesc{};
            modelDesc.Usage = D3D11_USAGE_DYNAMIC;
            modelDesc.ByteWidth = sizeof(PerObjectVSData);
            modelDesc.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
            modelDesc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
            if (FAILED(device->CreateBuffer(&modelDesc, nullptr, resources.modelCB.GetAddressOf()))) {
                return false;
            }

            D3D11_BUFFER_DESC axisLineDesc{};
            axisLineDesc.Usage = D3D11_USAGE_DYNAMIC;
            axisLineDesc.ByteWidth = sizeof(Vertex) * debug_overlay_policy::kMaxLineVertexBudget;
            axisLineDesc.BindFlags = D3D11_BIND_VERTEX_BUFFER;
            axisLineDesc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
            if (FAILED(device->CreateBuffer(&axisLineDesc, nullptr, resources.axisLineVB.GetAddressOf()))) {
                return false;
            }

            D3D11_BUFFER_DESC textDesc{};
            textDesc.Usage = D3D11_USAGE_DYNAMIC;
            textDesc.ByteWidth = sizeof(Vertex) * kTextVertexCapacity;
            textDesc.BindFlags = D3D11_BIND_VERTEX_BUFFER;
            textDesc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
            if (FAILED(device->CreateBuffer(&textDesc, nullptr, resources.textVB.GetAddressOf()))) {
                return false;
            }

            D3D11_RASTERIZER_DESC rasterDesc{};
            rasterDesc.FillMode = D3D11_FILL_WIREFRAME;
            rasterDesc.CullMode = D3D11_CULL_NONE;
            rasterDesc.FrontCounterClockwise = TRUE;
            rasterDesc.DepthClipEnable = TRUE;
            if (FAILED(device->CreateRasterizerState(&rasterDesc, resources.wireRasterizer.GetAddressOf()))) {
                return false;
            }

            rasterDesc.FillMode = D3D11_FILL_SOLID;
            if (FAILED(device->CreateRasterizerState(&rasterDesc, resources.solidRasterizer.GetAddressOf()))) {
                return false;
            }

            D3D11_DEPTH_STENCIL_DESC depthDesc{};
            depthDesc.DepthEnable = FALSE;
            depthDesc.DepthWriteMask = D3D11_DEPTH_WRITE_MASK_ZERO;
            depthDesc.DepthFunc = D3D11_COMPARISON_ALWAYS;
            if (FAILED(device->CreateDepthStencilState(&depthDesc, resources.depthStencil.GetAddressOf()))) {
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
            if (FAILED(device->CreateBlendState(&blendDesc, resources.blendState.GetAddressOf()))) {
                return false;
            }

            if (!resources.ready()) {
                return false;
            }

            s_d3d = std::move(resources);
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

        bool sameSubmittedTextureDesc(const D3D11_TEXTURE2D_DESC& lhs, const D3D11_TEXTURE2D_DESC& rhs)
        {
            return lhs.Width == rhs.Width && lhs.Height == rhs.Height && lhs.MipLevels == rhs.MipLevels && lhs.ArraySize == rhs.ArraySize && lhs.Format == rhs.Format &&
                lhs.SampleDesc.Count == rhs.SampleDesc.Count && lhs.SampleDesc.Quality == rhs.SampleDesc.Quality;
        }

        void clearSubmittedTextureRtvCache()
        {
            s_submittedTextureRtv.rtv.Reset();
            s_submittedTextureRtv.texture = nullptr;
            s_submittedTextureRtv.desc = D3D11_TEXTURE2D_DESC{};
        }

        ID3D11RenderTargetView* getSubmittedTextureRtv(ID3D11Device* device, ID3D11Texture2D* texture, const D3D11_TEXTURE2D_DESC& desc, OverlayRuntimeStats& stats)
        {
            if (s_submittedTextureRtv.texture == texture && s_submittedTextureRtv.rtv && sameSubmittedTextureDesc(s_submittedTextureRtv.desc, desc)) {
                ++stats.rtvCacheHits;
                return s_submittedTextureRtv.rtv.Get();
            }

            clearSubmittedTextureRtvCache();
            Microsoft::WRL::ComPtr<ID3D11RenderTargetView> rtv;
            if (FAILED(device->CreateRenderTargetView(texture, nullptr, rtv.GetAddressOf())) || !rtv) {
                return nullptr;
            }

            ++stats.rtvCacheMisses;
            s_submittedTextureRtv.texture = texture;
            s_submittedTextureRtv.desc = desc;
            s_submittedTextureRtv.rtv = std::move(rtv);
            return s_submittedTextureRtv.rtv.Get();
        }

        RenderPassGuard::RenderPassGuard(ID3D11DeviceContext* context, ID3D11RenderTargetView* renderTarget, UINT width, UINT height) noexcept
            : _context(context)
        {
            if (!context || !renderTarget || width == 0 || height == 0 || !s_d3d.ready() || s_renderPassActive.test_and_set(std::memory_order_acquire)) {
                _context = nullptr;
                return;
            }

            _active = true;
            _saved.vsInstanceCount = kMaxShaderClassInstances;
            _saved.psInstanceCount = kMaxShaderClassInstances;
            _saved.gsInstanceCount = kMaxShaderClassInstances;
            _saved.hsInstanceCount = kMaxShaderClassInstances;
            _saved.dsInstanceCount = kMaxShaderClassInstances;
            context->VSGetShader(&_saved.vs, _saved.vsInstances, &_saved.vsInstanceCount);
            context->PSGetShader(&_saved.ps, _saved.psInstances, &_saved.psInstanceCount);
            context->GSGetShader(&_saved.gs, _saved.gsInstances, &_saved.gsInstanceCount);
            context->HSGetShader(&_saved.hs, _saved.hsInstances, &_saved.hsInstanceCount);
            context->DSGetShader(&_saved.ds, _saved.dsInstances, &_saved.dsInstanceCount);
            context->VSGetConstantBuffers(0, static_cast<UINT>(std::size(_saved.vsCBs)), _saved.vsCBs);
            context->IAGetInputLayout(&_saved.inputLayout);
            context->IAGetPrimitiveTopology(&_saved.topology);
            context->RSGetState(&_saved.rasterizerState);
            context->OMGetDepthStencilState(&_saved.depthStencilState, &_saved.stencilRef);
            context->OMGetBlendState(&_saved.blendState, _saved.blendFactor, &_saved.sampleMask);
            context->OMGetRenderTargets(D3D11_SIMULTANEOUS_RENDER_TARGET_COUNT, _saved.rtvs, &_saved.dsv);
            _saved.numViewports = D3D11_VIEWPORT_AND_SCISSORRECT_OBJECT_COUNT_PER_PIPELINE;
            context->RSGetViewports(&_saved.numViewports, _saved.viewports);
            context->IAGetVertexBuffers(0, 2, _saved.vertexBuffers, _saved.vbStrides, _saved.vbOffsets);
            context->IAGetIndexBuffer(&_saved.indexBuffer, &_saved.ibFormat, &_saved.ibOffset);

            context->OMSetRenderTargets(1, &renderTarget, nullptr);
            D3D11_VIEWPORT viewport{};
            viewport.Width = static_cast<float>(width);
            viewport.Height = static_cast<float>(height);
            viewport.MinDepth = 0.0f;
            viewport.MaxDepth = 1.0f;
            context->RSSetViewports(1, &viewport);
            context->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
            context->IASetInputLayout(s_d3d.inputLayout.Get());
            context->VSSetShader(s_d3d.vertexShader.Get(), nullptr, 0);
            context->PSSetShader(s_d3d.pixelShader.Get(), nullptr, 0);
            context->GSSetShader(nullptr, nullptr, 0);
            context->HSSetShader(nullptr, nullptr, 0);
            context->DSSetShader(nullptr, nullptr, 0);
            context->RSSetState(s_d3d.wireRasterizer.Get());
            FLOAT blendFactor[4] = {};
            context->OMSetBlendState(s_d3d.blendState.Get(), blendFactor, 0xFFFFFFFF);
            context->OMSetDepthStencilState(s_d3d.depthStencil.Get(), 0);
        }

        RenderPassGuard::~RenderPassGuard() noexcept
        {
            restore();
        }

        void RenderPassGuard::restore() noexcept
        {
            if (!_active || !_context) {
                return;
            }

            _context->VSSetShader(_saved.vs, _saved.vsInstances, _saved.vsInstanceCount);
            _context->PSSetShader(_saved.ps, _saved.psInstances, _saved.psInstanceCount);
            _context->GSSetShader(_saved.gs, _saved.gsInstances, _saved.gsInstanceCount);
            _context->HSSetShader(_saved.hs, _saved.hsInstances, _saved.hsInstanceCount);
            _context->DSSetShader(_saved.ds, _saved.dsInstances, _saved.dsInstanceCount);
            _context->VSSetConstantBuffers(0, static_cast<UINT>(std::size(_saved.vsCBs)), _saved.vsCBs);
            _context->IASetInputLayout(_saved.inputLayout);
            _context->IASetPrimitiveTopology(_saved.topology);
            _context->RSSetState(_saved.rasterizerState);
            _context->OMSetDepthStencilState(_saved.depthStencilState, _saved.stencilRef);
            _context->OMSetBlendState(_saved.blendState, _saved.blendFactor, _saved.sampleMask);
            _context->OMSetRenderTargets(D3D11_SIMULTANEOUS_RENDER_TARGET_COUNT, _saved.rtvs, _saved.dsv);
            _context->RSSetViewports(_saved.numViewports, _saved.viewports);
            _context->IASetVertexBuffers(0, 2, _saved.vertexBuffers, _saved.vbStrides, _saved.vbOffsets);
            _context->IASetIndexBuffer(_saved.indexBuffer, _saved.ibFormat, _saved.ibOffset);
            releaseSavedState(_saved);

            _active = false;
            _context = nullptr;
            s_renderPassActive.clear(std::memory_order_release);
        }

        bool uploadCamera(ID3D11DeviceContext* context, const DirectX::XMMATRIX& eye0, const DirectX::XMMATRIX& eye1, const DirectX::XMFLOAT4& adjust0,
            const DirectX::XMFLOAT4& adjust1)
        {
            if (!context || !s_d3d.cameraCB) {
                return false;
            }

            D3D11_MAPPED_SUBRESOURCE mapped{};
            if (FAILED(context->Map(s_d3d.cameraCB.Get(), 0, D3D11_MAP_WRITE_DISCARD, 0, &mapped)) || !mapped.pData) {
                if (!s_cameraUploadFailureReported.exchange(true, std::memory_order_relaxed)) {
                    ROCK_LOG_WARN(Hand, "Debug overlay: camera constant-buffer map failed; frame skipped");
                }
                return false;
            }

            auto* data = static_cast<PerFrameVSData*>(mapped.pData);
            data->matProjView[0] = eye0;
            data->matProjView[1] = eye1;
            data->posAdjust[0] = adjust0;
            data->posAdjust[1] = adjust1;
            context->Unmap(s_d3d.cameraCB.Get(), 0);
            ID3D11Buffer* cameraCB = s_d3d.cameraCB.Get();
            context->VSSetConstantBuffers(0, 1, &cameraCB);
            return true;
        }

        bool uploadColorModel(ID3D11DeviceContext* context, const DirectX::XMMATRIX& model, const float color[4])
        {
            if (!context || !s_d3d.modelCB || !color) {
                return false;
            }

            D3D11_MAPPED_SUBRESOURCE mapped{};
            if (FAILED(context->Map(s_d3d.modelCB.Get(), 0, D3D11_MAP_WRITE_DISCARD, 0, &mapped)) || !mapped.pData) {
                if (!s_modelUploadFailureReported.exchange(true, std::memory_order_relaxed)) {
                    ROCK_LOG_WARN(Hand, "Debug overlay: model constant-buffer map failed; affected draws skipped");
                }
                return false;
            }

            auto* data = static_cast<PerObjectVSData*>(mapped.pData);
            data->matModel = model;
            data->color[0] = color[0];
            data->color[1] = color[1];
            data->color[2] = color[2];
            data->color[3] = color[3];
            context->Unmap(s_d3d.modelCB.Get(), 0);
            ID3D11Buffer* modelCB = s_d3d.modelCB.Get();
            context->VSSetConstantBuffers(1, 1, &modelCB);
            return true;
        }

        bool uploadModel(ID3D11DeviceContext* context, const DirectX::XMMATRIX& model, BodyOverlayRole role, debug_overlay_policy::ShapeDecodeMode decodeMode)
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
            case BodyOverlayRole::BodyTorsoSegment:
                color[0] = 1.0f;
                color[1] = 0.58f;
                color[2] = 0.18f;
                color[3] = 0.48f;
                break;
            case BodyOverlayRole::BodyArmSegment:
                color[0] = 0.55f;
                color[1] = 0.95f;
                color[2] = 0.85f;
                color[3] = 0.48f;
                break;
            case BodyOverlayRole::BodyLegSegment:
                color[0] = 0.90f;
                color[1] = 0.95f;
                color[2] = 0.35f;
                color[3] = 0.48f;
                break;
            case BodyOverlayRole::BodyFootSegment:
                color[0] = 0.85f;
                color[1] = 0.70f;
                color[2] = 1.0f;
                color[3] = 0.48f;
                break;
            case BodyOverlayRole::Weapon:
                color[0] = 0.35f;
                color[1] = 1.0f;
                color[2] = 0.25f;
                break;
            case BodyOverlayRole::RightGrabAuthorityProxy:
                color[0] = 0.05f;
                color[1] = 0.95f;
                color[2] = 1.0f;
                color[3] = 0.92f;
                break;
            case BodyOverlayRole::LeftGrabAuthorityProxy:
                color[0] = 1.0f;
                color[1] = 0.20f;
                color[2] = 0.95f;
                color[3] = 0.92f;
                break;
            case BodyOverlayRole::RightGrabPivotSourceCollider:
                color[0] = 0.05f;
                color[1] = 0.95f;
                color[2] = 1.0f;
                color[3] = 0.42f;
                break;
            case BodyOverlayRole::LeftGrabPivotSourceCollider:
                color[0] = 1.0f;
                color[1] = 0.25f;
                color[2] = 0.95f;
                color[3] = 0.42f;
                break;
            case BodyOverlayRole::Target:
                color[0] = 1.0f;
                color[1] = 0.85f;
                color[2] = 0.05f;
                break;
            }

            if (decodeMode == debug_overlay_policy::ShapeDecodeMode::Proxy) {
                color[0] = 1.0f;
                color[1] = 0.62f;
                color[2] = 0.08f;
                color[3] = 0.82f;
            } else if (decodeMode == debug_overlay_policy::ShapeDecodeMode::Unsupported) {
                color[0] = 1.0f;
                color[1] = 0.10f;
                color[2] = 0.05f;
                color[3] = 0.90f;
            }

            return uploadColorModel(context, model, color);
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
            case AxisOverlayRole::RightGrabHeldRelativeHandTarget:
            case AxisOverlayRole::LeftGrabHeldRelativeHandTarget:
            case AxisOverlayRole::RightGrabRockVisualTarget:
            case AxisOverlayRole::LeftGrabRockVisualTarget:
            case AxisOverlayRole::RightGrabDesiredObject:
            case AxisOverlayRole::LeftGrabDesiredObject:
            case AxisOverlayRole::RightGrabHeldNode:
            case AxisOverlayRole::LeftGrabHeldNode:
                return kColliderAxisLength;
            case AxisOverlayRole::NativeScopeLiveCamera:
                return 14.0f;
            case AxisOverlayRole::NativeScopeRockTarget:
                return 10.0f;
            case AxisOverlayRole::RightGrabPalmGeneratedDirect:
            case AxisOverlayRole::LeftGrabPalmGeneratedDirect:
                return 7.0f;
            case AxisOverlayRole::RightGrabPalmAuthorityFrame:
            case AxisOverlayRole::LeftGrabPalmAuthorityFrame:
                return 11.0f;
            case AxisOverlayRole::RightGrabAuthorityProxyTarget:
            case AxisOverlayRole::LeftGrabAuthorityProxyTarget:
                return 13.0f;
            case AxisOverlayRole::RightGrabProxyReadback:
            case AxisOverlayRole::LeftGrabProxyReadback:
                return 15.0f;
            case AxisOverlayRole::RightGrabForceTorqueLiveBody:
            case AxisOverlayRole::LeftGrabForceTorqueLiveBody:
            case AxisOverlayRole::RightGrabForceTorqueDesiredBody:
            case AxisOverlayRole::LeftGrabForceTorqueDesiredBody:
                return 10.0f;
            case AxisOverlayRole::RightGrabMotorConstraintA:
            case AxisOverlayRole::LeftGrabMotorConstraintA:
            case AxisOverlayRole::RightGrabMotorConstraintB:
            case AxisOverlayRole::LeftGrabMotorConstraintB:
                return 8.0f;
            case AxisOverlayRole::RightGrabMotorAtomTargetBody:
            case AxisOverlayRole::LeftGrabMotorAtomTargetBody:
                return 12.0f;
            case AxisOverlayRole::RightGrabMotorColumnTargetBody:
            case AxisOverlayRole::LeftGrabMotorColumnTargetBody:
                return 9.0f;
            case AxisOverlayRole::RightGrabMotorRelationInputBody:
            case AxisOverlayRole::LeftGrabMotorRelationInputBody:
                return 7.0f;
            case AxisOverlayRole::RightGrabMotorRelationInverseBody:
            case AxisOverlayRole::LeftGrabMotorRelationInverseBody:
                return 13.0f;
            case AxisOverlayRole::RightGrabMotorSolverEffectiveBody:
            case AxisOverlayRole::LeftGrabMotorSolverEffectiveBody:
                return 16.0f;
            case AxisOverlayRole::RightCustomCalibrationOffset:
            case AxisOverlayRole::LeftCustomCalibrationOffset:
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
            case AxisOverlayRole::NativeScopeLiveCamera:
            case AxisOverlayRole::NativeScopeRockTarget:
            case AxisOverlayRole::RightWeaponPrimaryGrip:
            case AxisOverlayRole::LeftWeaponSupportGrip:
            case AxisOverlayRole::RightFrikAppliedHand:
            case AxisOverlayRole::LeftFrikAppliedHand:
            case AxisOverlayRole::RightGrabHeldRelativeHandTarget:
            case AxisOverlayRole::LeftGrabHeldRelativeHandTarget:
            case AxisOverlayRole::RightGrabRockVisualTarget:
            case AxisOverlayRole::LeftGrabRockVisualTarget:
            case AxisOverlayRole::RightGrabDesiredObject:
            case AxisOverlayRole::LeftGrabDesiredObject:
            case AxisOverlayRole::RightGrabHeldNode:
            case AxisOverlayRole::LeftGrabHeldNode:
                return 0.92f;
            case AxisOverlayRole::RightGrabPalmGeneratedDirect:
            case AxisOverlayRole::LeftGrabPalmGeneratedDirect:
                return 0.42f;
            case AxisOverlayRole::RightGrabPalmAuthorityFrame:
            case AxisOverlayRole::LeftGrabPalmAuthorityFrame:
                return 0.98f;
            case AxisOverlayRole::RightGrabAuthorityProxyTarget:
            case AxisOverlayRole::LeftGrabAuthorityProxyTarget:
                return 0.88f;
            case AxisOverlayRole::RightGrabProxyReadback:
            case AxisOverlayRole::LeftGrabProxyReadback:
                return 0.72f;
            case AxisOverlayRole::RightGrabForceTorqueLiveBody:
            case AxisOverlayRole::LeftGrabForceTorqueLiveBody:
                return 0.88f;
            case AxisOverlayRole::RightGrabForceTorqueDesiredBody:
            case AxisOverlayRole::LeftGrabForceTorqueDesiredBody:
                return 0.52f;
            case AxisOverlayRole::RightGrabMotorConstraintA:
            case AxisOverlayRole::LeftGrabMotorConstraintA:
                return 0.96f;
            case AxisOverlayRole::RightGrabMotorConstraintB:
            case AxisOverlayRole::LeftGrabMotorConstraintB:
                return 0.72f;
            case AxisOverlayRole::RightGrabMotorAtomTargetBody:
            case AxisOverlayRole::LeftGrabMotorAtomTargetBody:
                return 0.88f;
            case AxisOverlayRole::RightGrabMotorColumnTargetBody:
            case AxisOverlayRole::LeftGrabMotorColumnTargetBody:
                return 0.38f;
            case AxisOverlayRole::RightGrabMotorRelationInputBody:
            case AxisOverlayRole::LeftGrabMotorRelationInputBody:
                return 0.30f;
            case AxisOverlayRole::RightGrabMotorRelationInverseBody:
            case AxisOverlayRole::LeftGrabMotorRelationInverseBody:
                return 0.78f;
            case AxisOverlayRole::RightGrabMotorSolverEffectiveBody:
            case AxisOverlayRole::LeftGrabMotorSolverEffectiveBody:
                return 0.98f;
            case AxisOverlayRole::RightCustomCalibrationOffset:
            case AxisOverlayRole::LeftCustomCalibrationOffset:
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
            case MarkerOverlayRole::RightPalmPocketCenter:
            case MarkerOverlayRole::LeftPalmPocketCenter:
                color[0] = 1.0f;
                color[1] = 0.86f;
                color[2] = 0.08f;
                color[3] = 0.95f;
                break;
            case MarkerOverlayRole::RightPalmPocketRadius:
            case MarkerOverlayRole::LeftPalmPocketRadius:
                color[0] = 1.0f;
                color[1] = 0.72f;
                color[2] = 0.05f;
                color[3] = 0.72f;
                break;
            case MarkerOverlayRole::RightPinchPocketCenter:
                color[0] = 0.10f;
                color[1] = 1.0f;
                color[2] = 0.68f;
                color[3] = 0.98f;
                break;
            case MarkerOverlayRole::LeftPinchPocketCenter:
                color[0] = 1.0f;
                color[1] = 0.28f;
                color[2] = 0.75f;
                color[3] = 0.98f;
                break;
            case MarkerOverlayRole::RightPinchPocketAxis:
            case MarkerOverlayRole::LeftPinchPocketAxis:
                color[0] = 0.95f;
                color[1] = 0.95f;
                color[2] = 0.95f;
                color[3] = 0.82f;
                break;
            case MarkerOverlayRole::RightPinchDetectionDirection:
            case MarkerOverlayRole::LeftPinchDetectionDirection:
                color[0] = 0.25f;
                color[1] = 1.0f;
                color[2] = 0.18f;
                color[3] = 0.92f;
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
            case MarkerOverlayRole::RightGrabForceTorqueTargetPivot:
            case MarkerOverlayRole::LeftGrabForceTorqueTargetPivot:
                color[0] = 0.05f;
                color[1] = 1.0f;
                color[2] = 0.95f;
                color[3] = 1.0f;
                break;
            case MarkerOverlayRole::RightGrabForceTorqueLivePivot:
            case MarkerOverlayRole::LeftGrabForceTorqueLivePivot:
                color[0] = 1.0f;
                color[1] = 0.92f;
                color[2] = 0.05f;
                color[3] = 1.0f;
                break;
            case MarkerOverlayRole::RightGrabForceTorqueCorrection:
            case MarkerOverlayRole::LeftGrabForceTorqueCorrection:
                color[0] = 1.0f;
                color[1] = 0.08f;
                color[2] = 0.04f;
                color[3] = 0.96f;
                break;
            case MarkerOverlayRole::RightGrabForceTorqueLever:
            case MarkerOverlayRole::LeftGrabForceTorqueLever:
                color[0] = 0.92f;
                color[1] = 0.92f;
                color[2] = 0.92f;
                color[3] = 0.78f;
                break;
            case MarkerOverlayRole::RightGrabForceTorqueAxis:
            case MarkerOverlayRole::LeftGrabForceTorqueAxis:
                color[0] = 1.0f;
                color[1] = 0.15f;
                color[2] = 0.95f;
                color[3] = 0.96f;
                break;
            case MarkerOverlayRole::RightGrabMotorAnchorA:
            case MarkerOverlayRole::LeftGrabMotorAnchorA:
                color[0] = 0.10f;
                color[1] = 1.0f;
                color[2] = 0.95f;
                color[3] = 1.0f;
                break;
            case MarkerOverlayRole::RightGrabMotorAnchorB:
            case MarkerOverlayRole::LeftGrabMotorAnchorB:
                color[0] = 1.0f;
                color[1] = 0.85f;
                color[2] = 0.05f;
                color[3] = 1.0f;
                break;
            case MarkerOverlayRole::RightGrabMotorAtomTargetPivot:
            case MarkerOverlayRole::LeftGrabMotorAtomTargetPivot:
                color[0] = 0.25f;
                color[1] = 0.85f;
                color[2] = 1.0f;
                color[3] = 0.96f;
                break;
            case MarkerOverlayRole::RightGrabMotorAngularCommand:
            case MarkerOverlayRole::LeftGrabMotorAngularCommand:
                color[0] = 1.0f;
                color[1] = 0.25f;
                color[2] = 1.0f;
                color[3] = 0.96f;
                break;
            case MarkerOverlayRole::RightGrabMotorTargetBodyDelta:
            case MarkerOverlayRole::LeftGrabMotorTargetBodyDelta:
                color[0] = 1.0f;
                color[1] = 0.28f;
                color[2] = 0.05f;
                color[3] = 0.90f;
                break;
            case MarkerOverlayRole::RightGrabActivePivotBLiveBody:
            case MarkerOverlayRole::LeftGrabActivePivotBLiveBody:
                color[0] = 1.0f;
                color[1] = 0.72f;
                color[2] = 0.05f;
                color[3] = 1.0f;
                break;
            case MarkerOverlayRole::RightGrabActivePivotBDesiredBody:
            case MarkerOverlayRole::LeftGrabActivePivotBDesiredBody:
                color[0] = 0.05f;
                color[1] = 1.0f;
                color[2] = 0.95f;
                color[3] = 1.0f;
                break;
            case MarkerOverlayRole::RightGrabActivePivotBVisualNode:
            case MarkerOverlayRole::LeftGrabActivePivotBVisualNode:
                color[0] = 0.25f;
                color[1] = 0.45f;
                color[2] = 1.0f;
                color[3] = 1.0f;
                break;
            case MarkerOverlayRole::RightGrabActivePivotBVisualLock:
            case MarkerOverlayRole::LeftGrabActivePivotBVisualLock:
                color[0] = 1.0f;
                color[1] = 0.18f;
                color[2] = 0.02f;
                color[3] = 0.96f;
                break;
            case MarkerOverlayRole::RightGrabAuthorityProxyTarget:
                color[0] = 0.10f;
                color[1] = 0.95f;
                color[2] = 1.0f;
                color[3] = 0.98f;
                break;
            case MarkerOverlayRole::LeftGrabAuthorityProxyTarget:
                color[0] = 1.0f;
                color[1] = 0.20f;
                color[2] = 0.95f;
                color[3] = 0.98f;
                break;
            case MarkerOverlayRole::RightGrabAuthorityProxyOffset:
            case MarkerOverlayRole::LeftGrabAuthorityProxyOffset:
                color[0] = 1.0f;
                color[1] = 0.92f;
                color[2] = 0.10f;
                color[3] = 0.90f;
                break;
            case MarkerOverlayRole::RightGrabPivotSourceTriangle:
            case MarkerOverlayRole::LeftGrabPivotSourceTriangle:
                color[0] = 0.30f;
                color[1] = 0.90f;
                color[2] = 1.0f;
                color[3] = 0.78f;
                break;
            case MarkerOverlayRole::RightGrabPivotSourceMeshPoint:
            case MarkerOverlayRole::LeftGrabPivotSourceMeshPoint:
                color[0] = 0.12f;
                color[1] = 1.0f;
                color[2] = 0.45f;
                color[3] = 0.98f;
                break;
            case MarkerOverlayRole::RightGrabPivotSourceVisualMeshPoint:
            case MarkerOverlayRole::LeftGrabPivotSourceVisualMeshPoint:
                color[0] = 0.20f;
                color[1] = 0.55f;
                color[2] = 1.0f;
                color[3] = 0.98f;
                break;
            case MarkerOverlayRole::RightGrabPivotSourceCapturePoint:
            case MarkerOverlayRole::LeftGrabPivotSourceCapturePoint:
                color[0] = 0.98f;
                color[1] = 0.98f;
                color[2] = 0.98f;
                color[3] = 0.92f;
                break;
            case MarkerOverlayRole::RightGrabPivotSourceBodyVisualLock:
            case MarkerOverlayRole::LeftGrabPivotSourceBodyVisualLock:
                color[0] = 1.0f;
                color[1] = 0.38f;
                color[2] = 0.05f;
                color[3] = 0.92f;
                break;
            case MarkerOverlayRole::RightGrabPivotSourceCaptureMutation:
            case MarkerOverlayRole::LeftGrabPivotSourceCaptureMutation:
                color[0] = 0.75f;
                color[1] = 0.35f;
                color[2] = 1.0f;
                color[3] = 0.88f;
                break;
            case MarkerOverlayRole::RightGrabPivotSourceContactPoint:
            case MarkerOverlayRole::LeftGrabPivotSourceContactPoint:
                color[0] = 1.0f;
                color[1] = 0.55f;
                color[2] = 0.08f;
                color[3] = 0.95f;
                break;
            case MarkerOverlayRole::RightGrabSupportFramePivot:
            case MarkerOverlayRole::LeftGrabSupportFramePivot:
                color[0] = 0.98f;
                color[1] = 0.98f;
                color[2] = 0.98f;
                color[3] = 0.98f;
                break;
            case MarkerOverlayRole::RightGrabSupportFrameNormal:
            case MarkerOverlayRole::LeftGrabSupportFrameNormal:
                color[0] = 1.0f;
                color[1] = 0.90f;
                color[2] = 0.05f;
                color[3] = 0.98f;
                break;
            case MarkerOverlayRole::RightGrabSupportFrameAxis:
            case MarkerOverlayRole::LeftGrabSupportFrameAxis:
                color[0] = 1.0f;
                color[1] = 0.18f;
                color[2] = 0.95f;
                color[3] = 0.98f;
                break;
            case MarkerOverlayRole::RightGrabSupportFrameBinormal:
            case MarkerOverlayRole::LeftGrabSupportFrameBinormal:
                color[0] = 0.10f;
                color[1] = 0.95f;
                color[2] = 1.0f;
                color[3] = 0.94f;
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
            case MarkerOverlayRole::RightGrabFingerPadProbe:
            case MarkerOverlayRole::LeftGrabFingerPadProbe:
                color[0] = 1.0f;
                color[1] = 0.72f;
                color[2] = 0.08f;
                color[3] = 0.82f;
                break;
            case MarkerOverlayRole::RightGrabFingerSurfaceTarget:
            case MarkerOverlayRole::LeftGrabFingerSurfaceTarget:
                color[0] = 1.0f;
                color[1] = 0.24f;
                color[2] = 0.05f;
                color[3] = 0.95f;
                break;
            case MarkerOverlayRole::GrabFingerSweepTip:
                color[0] = 0.05f;
                color[1] = 0.95f;
                color[2] = 1.0f;
                color[3] = 0.96f;
                break;
            case MarkerOverlayRole::GrabFingerSweepOuter:
                color[0] = 0.25f;
                color[1] = 1.0f;
                color[2] = 0.12f;
                color[3] = 0.94f;
                break;
            case MarkerOverlayRole::GrabFingerSweepInner:
                color[0] = 0.92f;
                color[1] = 0.18f;
                color[2] = 1.0f;
                color[3] = 0.94f;
                break;
            case MarkerOverlayRole::GrabFingerSweepPivot:
                color[0] = 1.0f;
                color[1] = 1.0f;
                color[2] = 1.0f;
                color[3] = 1.0f;
                break;
            case MarkerOverlayRole::GrabFingerSweepAuthoredOpen:
                color[0] = 1.0f;
                color[1] = 0.72f;
                color[2] = 0.08f;
                color[3] = 1.0f;
                break;
            case MarkerOverlayRole::GrabFingerSweepContact:
                color[0] = 1.0f;
                color[1] = 0.90f;
                color[2] = 0.05f;
                color[3] = 1.0f;
                break;
            case MarkerOverlayRole::GrabFingerSweepHitNormal:
                color[0] = 1.0f;
                color[1] = 1.0f;
                color[2] = 1.0f;
                color[3] = 1.0f;
                break;
            case MarkerOverlayRole::GrabFingerSweepMiss:
                color[0] = 1.0f;
                color[1] = 0.05f;
                color[2] = 0.05f;
                color[3] = 1.0f;
                break;
            case MarkerOverlayRole::GrabFingerSweepOutOfReach:
                color[0] = 0.08f;
                color[1] = 0.35f;
                color[2] = 1.0f;
                color[3] = 1.0f;
                break;
            case MarkerOverlayRole::GrabFingerSweepOverOpen:
                color[0] = 1.0f;
                color[1] = 0.45f;
                color[2] = 0.02f;
                color[3] = 1.0f;
                break;
            case MarkerOverlayRole::GrabFingerSweepClosedLimit:
                color[0] = 1.0f;
                color[1] = 0.05f;
                color[2] = 0.42f;
                color[3] = 1.0f;
                break;
            case MarkerOverlayRole::GrabFingerSweepLiveSkeleton:
                color[0] = 0.92f;
                color[1] = 0.92f;
                color[2] = 0.92f;
                color[3] = 0.88f;
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
            case MarkerOverlayRole::NativeScopeLiveCamera:
                color[0] = 1.0f;
                color[1] = 0.08f;
                color[2] = 0.04f;
                color[3] = 1.0f;
                break;
            case MarkerOverlayRole::NativeScopeRockTarget:
                color[0] = 0.05f;
                color[1] = 1.0f;
                color[2] = 0.18f;
                color[3] = 1.0f;
                break;
            case MarkerOverlayRole::NativeScopeImmediateReadback:
                color[0] = 1.0f;
                color[1] = 0.52f;
                color[2] = 0.03f;
                color[3] = 1.0f;
                break;
            case MarkerOverlayRole::NativeScopePreWriteCamera:
                color[0] = 1.0f;
                color[1] = 0.90f;
                color[2] = 0.05f;
                color[3] = 0.92f;
                break;
            case MarkerOverlayRole::NativeScopeParentComposedCamera:
                color[0] = 0.20f;
                color[1] = 0.45f;
                color[2] = 1.0f;
                color[3] = 0.92f;
                break;
            case MarkerOverlayRole::NativeScopeHmd:
                color[0] = 0.05f;
                color[1] = 0.92f;
                color[2] = 1.0f;
                color[3] = 0.96f;
                break;
            case MarkerOverlayRole::NativeScopeCameraParent:
                color[0] = 0.82f;
                color[1] = 0.82f;
                color[2] = 0.82f;
                color[3] = 0.82f;
                break;
            case MarkerOverlayRole::NativeScopeSightBounds:
                color[0] = 0.25f;
                color[1] = 1.0f;
                color[2] = 0.48f;
                color[3] = 0.78f;
                break;
            case MarkerOverlayRole::NativeScopeMismatch:
                color[0] = 1.0f;
                color[1] = 0.08f;
                color[2] = 0.92f;
                color[3] = 0.98f;
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
            case MarkerOverlayRole::AuthoredSupportGripPalmSeat:
                color[0] = 1.0f;
                color[1] = 0.78f;
                color[2] = 0.05f;
                color[3] = 0.98f;
                break;
            case MarkerOverlayRole::AuthoredSupportGripLiveSample:
                color[0] = 0.05f;
                color[1] = 0.72f;
                color[2] = 1.0f;
                color[3] = 0.98f;
                break;
            case MarkerOverlayRole::AuthoredSupportGripError:
                color[0] = 1.0f;
                color[1] = 0.06f;
                color[2] = 0.03f;
                color[3] = 0.98f;
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
            case MarkerOverlayRole::RightGrabHeldRelativeHandTargetError:
            case MarkerOverlayRole::LeftGrabHeldRelativeHandTargetError:
                color[0] = 0.15f;
                color[1] = 1.0f;
                color[2] = 0.25f;
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
            case MarkerOverlayRole::RightDynamicHandRequestedDeviation:
            case MarkerOverlayRole::LeftDynamicHandRequestedDeviation:
                color[0] = 1.0f;
                color[1] = 0.82f;
                color[2] = 0.12f;
                color[3] = 0.98f;
                break;
            case MarkerOverlayRole::RightDynamicHandSolverResidual:
            case MarkerOverlayRole::LeftDynamicHandSolverResidual:
                color[0] = 1.0f;
                color[1] = 0.24f;
                color[2] = 0.16f;
                color[3] = 0.98f;
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

        debug_overlay_line_batch::Vec3 toLineVec(const Vertex& vertex)
        {
            return debug_overlay_line_batch::Vec3{ vertex.x, vertex.y, vertex.z };
        }

        debug_overlay_line_batch::Rgba toLineColor(const float color[4])
        {
            return debug_overlay_line_batch::Rgba{ color[0], color[1], color[2], color[3] };
        }

        std::uint32_t lineVertexBudget()
        {
            return debug_overlay_policy::clampLineVertexBudget(static_cast<int>(debug_overlay_policy::kDefaultLineVertexBudget));
        }

        void appendDebugLine(debug_overlay_line_batch::LineBatch& batch, const Vertex& start, const Vertex& end, const float color[4])
        {
            batch.addLine(toLineVec(start), toLineVec(end), toLineColor(color), lineVertexBudget());
        }

        void appendPointMarker(debug_overlay_line_batch::LineBatch& batch, const RE::NiPoint3& position, float size, const float color[4])
        {
            batch.addPointMarker(toLineVec(toVertex(position)), size, toLineColor(color), lineVertexBudget());
        }

        struct LineDrawRun
        {
            debug_overlay_line_batch::Rgba color{};
            UINT firstVertex = 0;
            UINT vertexCount = 0;
        };

        bool lineColorLess(const debug_overlay_line_batch::LineSegment& lhs, const debug_overlay_line_batch::LineSegment& rhs)
        {
            return std::tie(lhs.color.r, lhs.color.g, lhs.color.b, lhs.color.a) < std::tie(rhs.color.r, rhs.color.g, rhs.color.b, rhs.color.a);
        }

        void drawLineBatch(ID3D11DeviceContext* context, const debug_overlay_line_batch::LineBatch& batch, OverlayRuntimeStats& stats)
        {
            stats.lineVertices += static_cast<std::uint32_t>(batch.vertexCount());
            stats.lineLogicalLines += static_cast<std::uint32_t>(batch.lineCount());
            stats.lineBudgetRejects += static_cast<std::uint32_t>(batch.rejectedLineCount());

            if (batch.empty() || !s_d3d.axisLineVB) {
                return;
            }

            std::vector<debug_overlay_line_batch::LineSegment> ordered = batch.segments();
            std::sort(ordered.begin(), ordered.end(), lineColorLess);

            std::vector<Vertex> vertices;
            vertices.reserve(batch.vertexCount());
            std::vector<LineDrawRun> runs;
            for (const auto& segment : ordered) {
                if (runs.empty() || !(runs.back().color == segment.color)) {
                    runs.push_back(LineDrawRun{ segment.color, static_cast<UINT>(vertices.size()), 0 });
                }
                vertices.push_back(Vertex{ segment.start.x, segment.start.y, segment.start.z });
                vertices.push_back(Vertex{ segment.end.x, segment.end.y, segment.end.z });
                runs.back().vertexCount += 2;
            }

            D3D11_MAPPED_SUBRESOURCE mapped{};
            if (FAILED(context->Map(s_d3d.axisLineVB.Get(), 0, D3D11_MAP_WRITE_DISCARD, 0, &mapped))) {
                return;
            }

            std::memcpy(mapped.pData, vertices.data(), vertices.size() * sizeof(Vertex));
            context->Unmap(s_d3d.axisLineVB.Get(), 0);

            constexpr UINT stride = sizeof(Vertex);
            constexpr UINT offset = 0;
            ID3D11Buffer* vertexBuffer = s_d3d.axisLineVB.Get();
            context->IASetVertexBuffers(0, 1, &vertexBuffer, &stride, &offset);
            context->IASetIndexBuffer(nullptr, DXGI_FORMAT_UNKNOWN, 0);

            for (const auto& run : runs) {
                const float color[4] = { run.color.r, run.color.g, run.color.b, run.color.a };
                if (!uploadColorModel(context, DirectX::XMMatrixIdentity(), color)) {
                    continue;
                }
                context->DrawInstanced(run.vertexCount, 2, run.firstVertex, 0);
                ++stats.lineDrawCalls;
            }
        }

        void appendAxisTripod(debug_overlay_line_batch::LineBatch& batch, const Vertex& origin, const Vertex& xEnd, const Vertex& yEnd, const Vertex& zEnd, AxisOverlayRole role)
        {
            const float alpha = axisAlphaForRole(role);
            const float xColor[4] = { 1.0f, 0.05f, 0.05f, alpha };
            const float yColor[4] = { 0.05f, 1.0f, 0.10f, alpha };
            const float zColor[4] = { 0.10f, 0.35f, 1.0f, alpha };
            appendDebugLine(batch, origin, xEnd, xColor);
            appendDebugLine(batch, origin, yEnd, yColor);
            appendDebugLine(batch, origin, zEnd, zColor);
        }

        void collectTransformAxisEntry(debug_overlay_line_batch::LineBatch& batch, const AxisOverlayEntry& entry)
        {
            const float length = axisLengthForRole(entry.role);
            const RE::NiPoint3 origin = entry.transform.translate;
            RE::NiPoint3 xAxis{};
            RE::NiPoint3 yAxis{};
            RE::NiPoint3 zAxis{};
            if (entry.basis == AxisOverlayBasis::StoredColumns) {
                xAxis = normalizedNi(storedColumnAxis(entry.transform.rotate, 0));
                yAxis = normalizedNi(storedColumnAxis(entry.transform.rotate, 1));
                zAxis = normalizedNi(storedColumnAxis(entry.transform.rotate, 2));
            } else {
                xAxis = normalizedNi(debug_axis_math::rotateNiLocalToWorld(entry.transform.rotate, RE::NiPoint3(1.0f, 0.0f, 0.0f)));
                yAxis = normalizedNi(debug_axis_math::rotateNiLocalToWorld(entry.transform.rotate, RE::NiPoint3(0.0f, 1.0f, 0.0f)));
                zAxis = normalizedNi(debug_axis_math::rotateNiLocalToWorld(entry.transform.rotate, RE::NiPoint3(0.0f, 0.0f, 1.0f)));
            }

            appendAxisTripod(batch, toVertex(origin), toVertex(origin + xAxis * length), toVertex(origin + yAxis * length), toVertex(origin + zAxis * length), entry.role);

            if (entry.drawTranslationLine) {
                const float color[4] = { 1.0f, 0.86f, 0.05f, axisAlphaForRole(entry.role) };
                appendDebugLine(batch, toVertex(entry.translationStart), toVertex(origin), color);
            }
        }

        void collectBodyAxisEntry(debug_overlay_line_batch::LineBatch& batch, const PublishedAxisEntry& published)
        {
            const auto& entry = published.entry;

            const float length = axisLengthForRole(entry.role);
            const Vertex origin = transformPoint(published.bodyWorldMatrix, 0.0f, 0.0f, 0.0f);
            appendAxisTripod(batch, origin, transformPoint(published.bodyWorldMatrix, length, 0.0f, 0.0f),
                transformPoint(published.bodyWorldMatrix, 0.0f, length, 0.0f),
                transformPoint(published.bodyWorldMatrix, 0.0f, 0.0f, length), entry.role);

            if (entry.drawTranslationLine) {
                const float color[4] = { 1.0f, 0.86f, 0.05f, axisAlphaForRole(entry.role) };
                appendDebugLine(batch, toVertex(entry.translationStart), origin, color);
            }
        }

        void collectAxisOverlays(debug_overlay_line_batch::LineBatch& batch, const PublishedOverlayFrame& frame)
        {
            if (!frame.drawAxes || frame.axes.empty()) {
                return;
            }

            for (const auto& published : frame.axes) {
                if (published.entry.source == AxisOverlaySource::Body) {
                    collectBodyAxisEntry(batch, published);
                } else {
                    collectTransformAxisEntry(batch, published.entry);
                }
            }
        }

        void collectMarkerOverlays(debug_overlay_line_batch::LineBatch& batch, const PublishedOverlayFrame& frame)
        {
            if (!frame.drawMarkers || frame.markers.empty()) {
                return;
            }

            for (const auto& entry : frame.markers) {
                float color[4]{};
                markerColorForRole(entry.role, color);

                if (entry.drawLine) {
                    appendDebugLine(batch, toVertex(entry.position), toVertex(entry.lineEnd), color);
                }
                if (entry.drawPoint) {
                    appendPointMarker(batch, entry.position, entry.size, color);
                }
            }
        }

        void collectSkeletonOverlays(debug_overlay_line_batch::LineBatch& batch, const PublishedOverlayFrame& frame)
        {
            if (!frame.drawSkeleton || frame.skeleton.empty()) {
                return;
            }

            for (const auto& entry : frame.skeleton) {
                float color[4]{};
                skeletonColorForRole(entry.role, entry.inPowerArmor, color);

                if (entry.hasParent) {
                    appendDebugLine(batch, toVertex(entry.parentPosition), toVertex(entry.transform.translate), color);
                }
                if (entry.drawPoint) {
                    appendPointMarker(batch, entry.transform.translate, entry.pointSize, color);
                }
                if (entry.drawAxis) {
                    const auto endpoints = skeleton_bone_debug_math::computeAxisEndpoints(entry.transform, entry.axisLength);
                    const float axisAlpha = entry.inPowerArmor ? 0.95f : 0.70f;
                    const float xColor[4] = { 1.0f, 0.08f, 0.06f, axisAlpha };
                    const float yColor[4] = { 0.05f, 1.0f, 0.10f, axisAlpha };
                    const float zColor[4] = { 0.10f, 0.35f, 1.0f, axisAlpha };
                    appendDebugLine(batch, toVertex(entry.transform.translate), toVertex(endpoints.xEnd), xColor);
                    appendDebugLine(batch, toVertex(entry.transform.translate), toVertex(endpoints.yEnd), yColor);
                    appendDebugLine(batch, toVertex(entry.transform.translate), toVertex(endpoints.zEnd), zColor);
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
            float textureHeight,
            bool duplicatePerEye)
        {
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
            if (duplicatePerEye) {
                appendEye(1, eye1, adjust1);
            }
        }

        void drawTextOverlays(ID3D11DeviceContext* context,
            float textureWidth,
            float textureHeight,
            const PublishedOverlayFrame& frame,
            const DirectX::XMMATRIX& eye0,
            const DirectX::XMMATRIX& eye1,
            const DirectX::XMFLOAT4& adjust0,
            const DirectX::XMFLOAT4& adjust1,
            OverlayRuntimeStats& stats)
        {
            if (!frame.drawText || frame.text.empty() || !s_d3d.textVB || !s_d3d.screenTextVertexShader || textureWidth <= 0.0f || textureHeight <= 0.0f) {
                return;
            }

            context->IASetInputLayout(s_d3d.inputLayout.Get());
            context->VSSetShader(s_d3d.screenTextVertexShader.Get(), nullptr, 0);
            context->PSSetShader(s_d3d.pixelShader.Get(), nullptr, 0);
            context->RSSetState(s_d3d.solidRasterizer.Get());
            context->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

            constexpr UINT stride = sizeof(Vertex);
            constexpr UINT offset = 0;
            ID3D11Buffer* vertexBuffer = s_d3d.textVB.Get();
            context->IASetVertexBuffers(0, 1, &vertexBuffer, &stride, &offset);
            context->IASetIndexBuffer(nullptr, DXGI_FORMAT_UNKNOWN, 0);

            const bool duplicatePerEye = frame.settings.duplicateTextPerEye;
            const float eyeWidth = duplicatePerEye ? textureWidth * 0.5f : textureWidth;
            for (const auto& entry : frame.text) {
                std::vector<Vertex> vertices;
                vertices.reserve(4096);
                if (entry.worldAnchored) {
                    appendWorldAnchoredTextGlyphs(vertices, entry, eye0, eye1, adjust0, adjust1, textureWidth, textureHeight, duplicatePerEye);
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
                    ++stats.textVertexTruncations;
                    vertices.resize(kTextVertexCapacity);
                }

                D3D11_MAPPED_SUBRESOURCE mapped{};
                if (FAILED(context->Map(s_d3d.textVB.Get(), 0, D3D11_MAP_WRITE_DISCARD, 0, &mapped))) {
                    ++stats.textMapFailures;
                    continue;
                }
                std::memcpy(mapped.pData, vertices.data(), vertices.size() * sizeof(Vertex));
                context->Unmap(s_d3d.textVB.Get(), 0);
                if (!uploadColorModel(context, DirectX::XMMatrixIdentity(), entry.color)) {
                    continue;
                }
                context->Draw(static_cast<UINT>(vertices.size()), 0);
                stats.textVertices += static_cast<std::uint32_t>(vertices.size());
                ++stats.textDrawCalls;
            }
        }

        void drawOverlayToSubmittedTexture(const vr::Texture_t* texture)
        {
            performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::DebugOverlayRender);

            const auto frame = s_publishedFrame.load(std::memory_order_acquire);
            if (!frame) {
                return;
            }

            const bool hasBodiesToDraw = (frame->drawRockBodies || frame->drawTargetBodies) && !frame->bodies.empty();
            const bool hasAxesToDraw = frame->drawAxes && !frame->axes.empty();
            const bool hasMarkersToDraw = frame->drawMarkers && !frame->markers.empty();
            const bool hasSkeletonToDraw = frame->drawSkeleton && !frame->skeleton.empty();
            const bool hasTextToDraw = frame->drawText && !frame->text.empty();
            if ((!hasBodiesToDraw && !hasAxesToDraw && !hasMarkersToDraw && !hasSkeletonToDraw && !hasTextToDraw) || !frame->worldIdentity) {
                return;
            }

            auto* device = getDevice();
            auto* context = getContext();
            if (!device || !context || !s_initialized.load(std::memory_order_acquire) || s_d3d.device.Get() != device || !texture || !texture->handle ||
                texture->eType != vr::TextureType_DirectX) {
                return;
            }

            auto* submittedTexture = reinterpret_cast<ID3D11Texture2D*>(texture->handle);
            D3D11_TEXTURE2D_DESC textureDesc{};
            submittedTexture->GetDesc(&textureDesc);

            OverlayRuntimeStats stats{};
            stats.bodyExtractFailures = frame->bodyExtractFailures;
            stats.shapeGenerations = frame->shapeGenerations;
            stats.shapeGenerationDeferrals = frame->shapeGenerationDeferrals;
            ID3D11RenderTargetView* rtv = getSubmittedTextureRtv(device, submittedTexture, textureDesc, stats);
            if (!rtv) {
                return;
            }

            DirectX::XMMATRIX eye0;
            DirectX::XMMATRIX eye1;
            DirectX::XMFLOAT4 adjust0;
            DirectX::XMFLOAT4 adjust1;
            if (!getEyeViewProjMatrices(eye0, eye1, adjust0, adjust1)) {
                return;
            }

            RenderPassGuard renderPass(context, rtv, textureDesc.Width, textureDesc.Height);
            if (!renderPass.active() || !uploadCamera(context, eye0, eye1, adjust0, adjust1)) {
                return;
            }

            if (hasBodiesToDraw) {
                context->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
                std::shared_ptr<const GpuShape> lastBoundShape;
                for (const auto& entry : frame->bodies) {
                    ++stats.bodyEntries;
                    const ShapeDecodeResult* decoded = nullptr;
                    if (entry.capturedShapeIndex < frame->capturedShapes.size()) {
                        const auto& captured = frame->capturedShapes[entry.capturedShapeIndex];
                        if (captured.key == entry.shapeKey && captured.hasDecodedGeometry) {
                            decoded = &captured.decoded;
                        }
                    }

                    const auto gpuShape = getOrCreateShape(device, entry.shapeKey, decoded, stats);
                    if (!gpuShape) {
                        continue;
                    }

                    UINT stride = sizeof(Vertex);
                    UINT offset = 0;
                    if (gpuShape.get() != lastBoundShape.get()) {
                        ID3D11Buffer* vertexBuffer = gpuShape->vertexBuffer.Get();
                        context->IASetVertexBuffers(0, 1, &vertexBuffer, &stride, &offset);
                        context->IASetIndexBuffer(gpuShape->indexBuffer.Get(), DXGI_FORMAT_R16_UINT, 0);
                        lastBoundShape = gpuShape;
                        ++stats.bodyMeshBinds;
                    }
                    if (!uploadModel(context, entry.worldMatrix, entry.role, gpuShape->decodeMode)) {
                        continue;
                    }
                    context->DrawIndexedInstanced(gpuShape->indexCount, 2, 0, 0, 0);
                    ++stats.bodiesDrawn;
                    ++stats.bodyDrawCalls;
                }
            }

            debug_overlay_line_batch::LineBatch lineBatch;
            collectAxisOverlays(lineBatch, *frame);
            collectMarkerOverlays(lineBatch, *frame);
            collectSkeletonOverlays(lineBatch, *frame);
            context->VSSetShader(s_d3d.vertexShader.Get(), nullptr, 0);
            context->PSSetShader(s_d3d.pixelShader.Get(), nullptr, 0);
            context->RSSetState(s_d3d.wireRasterizer.Get());
            context->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_LINELIST);
            drawLineBatch(context, lineBatch, stats);
            drawTextOverlays(context, static_cast<float>(textureDesc.Width), static_cast<float>(textureDesc.Height), *frame, eye0, eye1, adjust0, adjust1, stats);

            if (frame->settings.verboseLogging && ++s_overlayStatsLogCounter >= 90) {
                s_overlayStatsLogCounter = 0;
                ROCK_LOG_DEBUG(Hand,
                    "Debug overlay frame: entries={} drawn={} bodyBinds={} bodyDraws={} axes={} markers={} skeleton={} text={} cacheHits={} cacheMisses={} shapeGenerations={} genDefers={} genCap={} cacheBudgetSkips={} proxies={} unsupportedProxy={} unsupportedSkip={} bodyReadFails={} lineVerts={} lineLines={} lineDraws={} lineRejects={} textVerts={} textDraws={} textTrunc={} textMapFails={} rtvHits={} rtvMisses={}",
                    stats.bodyEntries,
                    stats.bodiesDrawn,
                    stats.bodyMeshBinds,
                    stats.bodyDrawCalls,
                    frame->axes.size(),
                    frame->markers.size(),
                    frame->skeleton.size(),
                    frame->text.size(),
                    stats.shapeCacheHits,
                    stats.shapeCacheMisses,
                    stats.shapeGenerations,
                    stats.shapeGenerationDeferrals,
                    frame->settings.maxShapeGenerationsPerFrame,
                    stats.shapeCacheBudgetSkips,
                    stats.shapeProxyFallbacks,
                    stats.unsupportedShapeProxies,
                    stats.unsupportedShapeSkips,
                    stats.bodyExtractFailures,
                    stats.lineVertices,
                    stats.lineLogicalLines,
                    stats.lineDrawCalls,
                    stats.lineBudgetRejects,
                    stats.textVertices,
                    stats.textDrawCalls,
                    stats.textVertexTruncations,
                    stats.textMapFailures,
                    stats.rtvCacheHits,
                    stats.rtvCacheMisses);
            }
        }

        void reportOverlayExceptionOnce(const char* detail) noexcept
        {
            if (s_overlayExceptionReported.exchange(true, std::memory_order_relaxed)) {
                return;
            }

            try {
                ROCK_LOG_ERROR(Hand, "Debug overlay: compositor draw aborted by exception ({})", detail ? detail : "unknown");
            } catch (...) {
            }
        }

        void reportSubmitInstallFailureOnce(const char* detail) noexcept
        {
            if (s_submitInstallFailureReported.exchange(true, std::memory_order_relaxed)) {
                return;
            }

            ROCK_LOG_WARN(Hand, "Debug body overlay: {}; Submit hook installation will retry", detail);
        }

        vr::EVRCompositorError VRSubmitHook(vr::IVRCompositor* compositor, vr::EVREye eye, const vr::Texture_t* texture, const vr::VRTextureBounds_t* bounds,
            vr::EVRSubmitFlags flags) noexcept
        {
            if (eye == vr::Eye_Left && s_enabled.load(std::memory_order_acquire)) {
                auto admission = s_frameAdmission.tryAcquire();
                if (admission) {
                    try {
                        drawOverlayToSubmittedTexture(texture);
                    } catch (const std::exception& exception) {
                        reportOverlayExceptionOnce(exception.what());
                    } catch (...) {
                        reportOverlayExceptionOnce("non-standard exception");
                    }
                }
            }

            const auto originalSubmit = s_originalVRSubmit.load(std::memory_order_acquire);
            return originalSubmit ? originalSubmit(compositor, eye, texture, bounds, flags) : vr::VRCompositorError_RequestFailed;
        }

        bool installSubmitHook()
        {
            if (s_submitHookInstalled.load(std::memory_order_acquire)) {
                return true;
            }

            auto* compositor = vr::VRCompositor();
            if (!compositor) {
                reportSubmitInstallFailureOnce("VRCompositor unavailable");
                return false;
            }

            auto*** objectVTable = reinterpret_cast<void***>(compositor);
            if (!objectVTable || !*objectVTable) {
                reportSubmitInstallFailureOnce("VRCompositor vtable unavailable");
                return false;
            }
            s_vrCompositorVTable = *objectVTable;
            constexpr std::size_t kSubmitVTableIndex = 5;

            DWORD oldProtect = 0;
            if (!VirtualProtect(&s_vrCompositorVTable[kSubmitVTableIndex], sizeof(void*), kPageExecuteReadWrite, &oldProtect)) {
                reportSubmitInstallFailureOnce("VRCompositor vtable protection change failed");
                return false;
            }

            const auto originalSubmit = reinterpret_cast<VRSubmit_t>(s_vrCompositorVTable[kSubmitVTableIndex]);
            if (!originalSubmit || originalSubmit == &VRSubmitHook) {
                DWORD ignoredProtect = 0;
                VirtualProtect(&s_vrCompositorVTable[kSubmitVTableIndex], sizeof(void*), oldProtect, &ignoredProtect);
                reportSubmitInstallFailureOnce("original OpenVR Submit target is invalid");
                return false;
            }

            s_originalVRSubmit.store(originalSubmit, std::memory_order_release);
            s_vrCompositorVTable[kSubmitVTableIndex] = reinterpret_cast<void*>(&VRSubmitHook);
            DWORD ignoredProtect = 0;
            if (!VirtualProtect(&s_vrCompositorVTable[kSubmitVTableIndex], sizeof(void*), oldProtect, &ignoredProtect)) {
                ROCK_LOG_ERROR(Hand, "Debug body overlay: failed to restore OpenVR Submit vtable protection after hook install");
            }

            s_submitHookInstalled.store(true, std::memory_order_release);
            s_submitInstallFailureReported.store(false, std::memory_order_relaxed);
            ROCK_LOG_INFO(Hand, "Debug body overlay: OpenVR Submit hook installed");
            return true;
        }
    }

    void Install()
    {
        if (s_initialized.load(std::memory_order_acquire)) {
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

        s_initialized.store(true, std::memory_order_release);
        if (installSubmitHook()) {
            ROCK_LOG_INFO(Hand, "Debug body overlay installed");
        }
    }

    bool IsInstalled()
    {
        return s_initialized.load(std::memory_order_acquire) && s_submitHookInstalled.load(std::memory_order_acquire);
    }

    void PublishFrame(const BodyOverlayFrame& frame)
    {
        auto next = s_framePool.acquire();
        if (!next) {
            if (!s_snapshotPoolExhaustionReported.exchange(true, std::memory_order_relaxed)) {
                ROCK_LOG_WARN(Hand, "Debug body overlay: immutable snapshot pool exhausted; retaining the last safe publication");
            }
            return;
        }

        const bool enabled = buildPublishedFrame(frame, *next);
        std::shared_ptr<const PublishedOverlayFrame> immutable = std::move(next);
        s_publishedFrame.store(std::move(immutable), std::memory_order_release);
        s_snapshotPoolExhaustionReported.store(false, std::memory_order_relaxed);
        s_enabled.store(enabled, std::memory_order_release);
        (void)s_frameAdmission.publish();
    }

    void ClearFrame()
    {
        s_publishedFrame.store({}, std::memory_order_release);
        s_enabled.store(false, std::memory_order_release);
        (void)s_frameAdmission.publish();
    }

    void ClearShapeCache()
    {
        std::scoped_lock lock(s_shapeCacheMutex);
        s_shapeCache.clear();
    }

    bool TryGetCurrentStereoOrigin(RE::NiPoint3& outOrigin)
    {
        StereoCaptureStage deepestStage = StereoCaptureStage::None;
        DWORD readError = ERROR_SUCCESS;
        RootStereoFields rootFields{};
        if (!readRootStereoFields(rootFields, deepestStage, readError) || !validateStereoVector3(rootFields.slot0Origin)) {
            return false;
        }

        outOrigin.x = rootFields.slot0Origin[0];
        outOrigin.y = rootFields.slot0Origin[1];
        outOrigin.z = rootFields.slot0Origin[2];
        return true;
    }
}
