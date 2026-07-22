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
#include <tuple>
#include <utility>
#include <vector>

#include <d3d11.h>
#include <d3dcompiler.h>
#include <wrl/client.h>

#include "physics-interaction/native/HavokOffsets.h"
#include "physics-interaction/debug/DebugOverlayFrameAdmission.h"
#include "physics-interaction/debug/DebugOverlayGpuTimer.h"
#include "physics-interaction/debug/DebugOverlayLineBatch.h"
#include "physics-interaction/debug/DebugOverlayPolicy.h"
#include "physics-interaction/debug/DebugOverlayRuntimeSettings.h"
#include "physics-interaction/debug/DebugOverlayShapeGeometry.h"
#include "physics-interaction/debug/DebugOverlayShapePipeline.h"
#include "physics-interaction/debug/DebugOverlayShaders.h"
#include "physics-interaction/debug/DebugOverlaySnapshotPool.h"
#include "physics-interaction/debug/DebugOverlayStats.h"
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
#include "rock_support/VRControllers.h"
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
        // Concrete scaled/compound layouts are absent from CommonLibF4VR.
        // These FO4VR offsets were independently verified in the constructors,
        // alloc/copy helpers, key-mask code, and shape consumers recorded in
        // DEBUG_OVERLAY_MODERNIZATION_PROGRESS.md.
        constexpr std::uintptr_t kScaledConvexInnerShapeOffset = 0x30;
        constexpr std::uintptr_t kScaledConvexScaleOffset = 0x40;
        constexpr std::uintptr_t kScaledConvexTranslationOffset = 0x50;
        constexpr std::uintptr_t kCompoundSlotArrayOffset = 0x60;
        constexpr std::uintptr_t kCompoundSlotCountOffset = 0x68;
        constexpr std::uintptr_t kCompoundSlotStride = 0x80;
        constexpr std::uintptr_t kCompoundSlotTransformOffset = 0x00;
        constexpr std::uintptr_t kCompoundSlotScaleOffset = 0x40;
        constexpr std::uintptr_t kCompoundSlotShapeOffset = 0x50;
        constexpr std::uintptr_t kCompoundSlotActiveOffset = 0x60;
        constexpr std::uint32_t kInvalidBodyId = 0x7FFF'FFFF;
        constexpr std::uint32_t kFreeMotionIndex = 0x7FFF'FFFF;
        constexpr std::uint32_t kMaxBodyIndex = body_frame::kMaxReadableBodyIndex;
        constexpr std::uint32_t kMaxMotionIndex = 4096;
        constexpr float kRawAxisLength = 8.0f;
        constexpr float kColliderAxisLength = 12.0f;
        constexpr float kBodyAxisLength = 16.0f;
        constexpr float kTargetAxisLength = 20.0f;
        constexpr std::size_t kBodyInstanceCapacity = std::tuple_size_v<decltype(BodyOverlayFrame{}.entries)>;
        static_assert(kBodyInstanceCapacity == debug_overlay_runtime::kMaxBodyInstances);
        constexpr std::uint64_t kCanonicalSphereGeometryFingerprint = 0x5350'4845'5245'0001ull;
        constexpr DWORD kPageExecuteReadWrite = 0x00000040u;
        constexpr UINT kMaxShaderClassInstances = 256;
        constexpr UINT kSavedVertexBufferSlots = 2;

        using Vertex = debug_overlay_shape::Vertex;
        using MeshData = debug_overlay_shape::MeshData;
        using ShapeKey = debug_overlay_shape::ShapeKey;
        using GpuShape = debug_overlay_shape::GpuShape;

        struct ColoredVertex
        {
            float x{ 0.0f };
            float y{ 0.0f };
            float z{ 0.0f };
            float color[4]{ 1.0f, 1.0f, 1.0f, 1.0f };
        };
        static_assert(sizeof(ColoredVertex) == 28);

        struct alignas(16) BodyInstanceData
        {
            DirectX::XMFLOAT4X4 model{};
            float color[4]{ 1.0f, 1.0f, 1.0f, 0.85f };
        };
        static_assert(sizeof(BodyInstanceData) == 80);
        static_assert(alignof(BodyInstanceData) == 16);

        struct BodyDrawItem
        {
            std::shared_ptr<const GpuShape> shapeOwner;
            const GpuShape* shape{ nullptr };
            BodyInstanceData instance{};
        };

        struct RenderScratch
        {
            debug_overlay_line_batch::LineBatch lines;
            std::vector<BodyDrawItem> bodies;
            std::vector<ColoredVertex> textVertices;

            bool prepare()
            {
                try {
                    if (!lines.prepare(debug_overlay_policy::kMaxLineVertexBudget)) {
                        return false;
                    }
                    bodies.reserve(kBodyInstanceCapacity);
                    textVertices.reserve(debug_overlay_runtime::kMaxTextVertices);
                } catch (...) {
                    return false;
                }
                return true;
            }
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
            debug_overlay_runtime::Limits limits{};
            std::uint64_t shapeDecodeSettingsKey{ 0 };
            debug_overlay_shape::PipelineLimits pipelineLimits{};
            bool useBoundsForHeavyConvex{ false };
            bool duplicateTextPerEye{ true };
            bool verboseLogging{ false };
        };

        struct CapturedShapeIdentity
        {
            std::uintptr_t shapeAddress{ 0 };
            ShapeKey key{};
            float detailUniformScale{ 1.0f };
        };

        struct PublishedBodyEntry
        {
            ShapeKey shapeKey{};
            DirectX::XMMATRIX worldMatrix = DirectX::XMMatrixIdentity();
            DirectX::XMFLOAT3 worldAabbMin{};
            DirectX::XMFLOAT3 worldAabbMax{};
            BodyOverlayRole role{ BodyOverlayRole::Target };
            std::uint32_t bodyId{ kInvalidBodyId };
            float detailUniformScale{ 1.0f };
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
            std::vector<ColoredLineOverlayEntry> coloredLines;
            std::vector<TextOverlayEntry> text;
            std::vector<CapturedShapeIdentity> capturedShapeIdentities;
            OverlayRenderSettings settings{};
            std::uintptr_t worldIdentity{ 0 };
            std::uint32_t bodyExtractFailures{ 0 };
            std::uint32_t shapeCaptures{ 0 };
            std::uint32_t shapeCaptureDeferrals{ 0 };
            bool drawRockBodies{ false };
            bool drawTargetBodies{ false };
            bool drawAxes{ false };
            bool drawMarkers{ false };
            bool drawSkeleton{ false };
            bool drawColoredLines{ false };
            bool drawText{ false };
        };

        enum class BodyOverlayFrameSource : std::uint8_t
        {
            LiveMotionWhenAvailable,
            BodyArrayTransform
        };

        using OverlayRuntimeStats = debug_overlay_stats::RuntimeStats;

        struct alignas(16) PerFrameVSData
        {
            DirectX::XMMATRIX matProjView[2];
            DirectX::XMFLOAT4 posAdjust[2];
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
            Microsoft::WRL::ComPtr<ID3D11VertexShader> bodyVertexShader;
            Microsoft::WRL::ComPtr<ID3D11VertexShader> stereoColorVertexShader;
            Microsoft::WRL::ComPtr<ID3D11VertexShader> screenTextVertexShader;
            Microsoft::WRL::ComPtr<ID3D11PixelShader> pixelShader;
            Microsoft::WRL::ComPtr<ID3D11InputLayout> bodyInputLayout;
            Microsoft::WRL::ComPtr<ID3D11InputLayout> coloredInputLayout;
            Microsoft::WRL::ComPtr<ID3D11Buffer> cameraCB;
            Microsoft::WRL::ComPtr<ID3D11Buffer> bodyInstanceVB;
            Microsoft::WRL::ComPtr<ID3D11Buffer> axisLineVB;
            Microsoft::WRL::ComPtr<ID3D11Buffer> textVB;
            Microsoft::WRL::ComPtr<ID3D11RasterizerState> wireRasterizer;
            Microsoft::WRL::ComPtr<ID3D11RasterizerState> solidRasterizer;
            Microsoft::WRL::ComPtr<ID3D11DepthStencilState> depthStencil;
            Microsoft::WRL::ComPtr<ID3D11BlendState> blendState;
            GpuShape aabbProxy;
            std::unique_ptr<RenderScratch> scratch;
            debug_overlay_gpu_timer::TimestampQueryRing gpuTimer;

            [[nodiscard]] bool ready() const noexcept
            {
                return device && bodyVertexShader && stereoColorVertexShader && screenTextVertexShader && pixelShader && bodyInputLayout && coloredInputLayout &&
                       cameraCB && bodyInstanceVB && axisLineVB && textVB && scratch &&
                       wireRasterizer && solidRasterizer && depthStencil && blendState &&
                       aabbProxy.vertexBuffer && aabbProxy.indexBuffer && aabbProxy.indexCount > 0;
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
        static std::atomic<bool> s_bodyInstanceUploadFailureReported{ false };
        static std::atomic<bool> s_lineUploadFailureReported{ false };
        static std::atomic<bool> s_textUploadFailureReported{ false };
        static std::atomic<bool> s_submitInstallFailureReported{ false };
        static std::atomic<bool> s_snapshotPoolExhaustionReported{ false };
        static std::atomic<bool> s_shapeWorkerInitFailureReported{ false };
        static std::atomic<bool> s_gpuTimerInitFailureReported{ false };
        static CachedRenderTargetView s_submittedTextureRtv{};

        debug_overlay_shape::ShapePipeline& shapePipeline()
        {
            static debug_overlay_shape::ShapePipeline pipeline;
            return pipeline;
        }

        using VRSubmit_t = vr::EVRCompositorError(__thiscall*)(vr::IVRCompositor*, vr::EVREye, const vr::Texture_t*, const vr::VRTextureBounds_t*, vr::EVRSubmitFlags);
        static std::atomic<VRSubmit_t> s_originalVRSubmit{ nullptr };
        static void** s_vrCompositorVTable = nullptr;

        Vertex toVertex(const RE::NiPoint3& value) { return Vertex{ value.x, value.y, value.z }; }

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

        std::uint64_t computeShapeGeometryFingerprintUnsafe(
            std::uintptr_t shapeAddress,
            const OverlayRenderSettings& settings,
            int depth = 0,
            int* rootShapeType = nullptr,
            float* rootConvexRadius = nullptr)
        {
            /*
             * Debug collider meshes are cached on the VR submit path, but
             * generated ROCK bodies are destroyed and recreated when profiles
             * such as power armor change. Havok can reuse the same shape
             * address for different support geometry, so the cache key must
             * include a small geometry fingerprint instead of trusting the
             * pointer alone.
             */
            if (!shapeAddress || depth > static_cast<int>(settings.limits.maxCompoundDepth)) {
                return 0;
            }

            try {
                auto* shape = reinterpret_cast<RE::hknpShape*>(shapeAddress);
                const int shapeType = static_cast<int>(shape->GetType());
                if (depth == 0) {
                    if (rootShapeType) {
                        *rootShapeType = shapeType;
                    }
                    if (rootConvexRadius) {
                        *rootConvexRadius = shape->convexRadius;
                    }
                }
                std::uint64_t fingerprint = 0xcbf29ce484222325ull;
                fingerprint = mixShapeFingerprint(fingerprint, static_cast<std::uint64_t>(shapeType));
                fingerprint = mixShapeFloat(fingerprint, shape->convexRadius);

                switch (shapeType) {
                case 0:
                case 1:
                case 4: {
                    const int vertexCount = shape->GetNumberOfSupportVertices();
                    fingerprint = mixShapeFingerprint(fingerprint, static_cast<std::uint32_t>((std::max)(vertexCount, 0)));
                    if (vertexCount <= 0 || vertexCount > 256) {
                        return fingerprint;
                    }

                    std::array<RE::hkVector4f, 256> supportVertices{};
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
                case 7:
                case 8: {
                    const auto slotArray = *reinterpret_cast<const std::uintptr_t*>(shapeAddress + kCompoundSlotArrayOffset);
                    const auto slotCount = *reinterpret_cast<const std::int32_t*>(shapeAddress + kCompoundSlotCountOffset);
                    fingerprint = mixShapeFingerprint(fingerprint, static_cast<std::uint32_t>((std::max)(slotCount, 0)));
                    if (!slotArray || slotCount <= 0 ||
                        static_cast<std::uint32_t>(slotCount) > settings.limits.maxCompoundChildren) {
                        return fingerprint;
                    }

                    for (std::int32_t index = 0; index < slotCount; ++index) {
                        const auto slotOffset = static_cast<std::uintptr_t>(index) * kCompoundSlotStride;
                        if (slotArray > (std::numeric_limits<std::uintptr_t>::max)() - slotOffset) {
                            return 0;
                        }
                        const auto slot = slotArray + slotOffset;
                        const auto active = *reinterpret_cast<const std::uint8_t*>(slot + kCompoundSlotActiveOffset);
                        fingerprint = mixShapeFingerprint(fingerprint, static_cast<std::uint32_t>(index));
                        fingerprint = mixShapeFingerprint(fingerprint, active);
                        if (active != 0) {
                            continue;
                        }

                        const auto childShape = *reinterpret_cast<const std::uintptr_t*>(slot + kCompoundSlotShapeOffset);
                        fingerprint = mixShapeFingerprint(fingerprint, childShape);
                        const auto* transform = reinterpret_cast<const float*>(slot + kCompoundSlotTransformOffset);
                        const auto* childScale = reinterpret_cast<const float*>(slot + kCompoundSlotScaleOffset);
                        for (std::size_t component = 0; component < 16; ++component) {
                            fingerprint = mixShapeFloat(fingerprint, transform[component]);
                        }
                        for (std::size_t component = 0; component < 3; ++component) {
                            fingerprint = mixShapeFloat(fingerprint, childScale[component]);
                        }
                        fingerprint = mixShapeFingerprint(
                            fingerprint,
                            computeShapeGeometryFingerprintUnsafe(childShape, settings, depth + 1));
                    }
                    return fingerprint;
                }
                case 11: {
                    const auto innerShape = *reinterpret_cast<const std::uintptr_t*>(shapeAddress + kScaledConvexInnerShapeOffset);
                    fingerprint = mixShapeFingerprint(fingerprint, innerShape);
                    fingerprint = mixShapeFingerprint(
                        fingerprint,
                        computeShapeGeometryFingerprintUnsafe(innerShape, settings, depth + 1));
                    const auto* scaleVector = reinterpret_cast<const float*>(shapeAddress + kScaledConvexScaleOffset);
                    const auto* translationVector = reinterpret_cast<const float*>(shapeAddress + kScaledConvexTranslationOffset);
                    for (std::size_t component = 0; component < 3; ++component) {
                        fingerprint = mixShapeFloat(fingerprint, scaleVector[component]);
                        fingerprint = mixShapeFloat(fingerprint, translationVector[component]);
                    }
                    return fingerprint;
                }
                default:
                    return fingerprint;
                }
            } catch (...) {
                return 0;
            }
        }

        std::uint64_t computeShapeGeometryFingerprintSeh(
            std::uintptr_t shapeAddress,
            const OverlayRenderSettings* settings,
            int* rootShapeType,
            float* rootConvexRadius)
        {
            if (rootShapeType) {
                *rootShapeType = -1;
            }
            if (rootConvexRadius) {
                *rootConvexRadius = 0.0f;
            }
            if (!settings) {
                return 0;
            }
            __try {
                return computeShapeGeometryFingerprintUnsafe(shapeAddress, *settings, 0, rootShapeType, rootConvexRadius);
            } __except (EXCEPTION_EXECUTE_HANDLER) {
                return 0;
            }
        }

        ShapeKey makeShapeKey(
            std::uintptr_t shapeAddress,
            const OverlayRenderSettings& settings,
            int& shapeType,
            float& convexRadius)
        {
            return ShapeKey{
                shapeAddress,
                computeShapeGeometryFingerprintSeh(shapeAddress, &settings, &shapeType, &convexRadius)
            };
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

        DirectX::XMMATRIX worldAabbMatrix(const PublishedBodyEntry& body)
        {
            const float extentX = body.worldAabbMax.x - body.worldAabbMin.x;
            const float extentY = body.worldAabbMax.y - body.worldAabbMin.y;
            const float extentZ = body.worldAabbMax.z - body.worldAabbMin.z;
            const float centerX = (body.worldAabbMin.x + body.worldAabbMax.x) * 0.5f;
            const float centerY = (body.worldAabbMin.y + body.worldAabbMax.y) * 0.5f;
            const float centerZ = (body.worldAabbMin.z + body.worldAabbMax.z) * 0.5f;
            return DirectX::XMMatrixMultiply(
                DirectX::XMMatrixScaling(extentX, extentY, extentZ),
                DirectX::XMMatrixTranslation(centerX, centerY, centerZ));
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

        bool createStaticGpuShape(ID3D11Device* device, const MeshData& mesh, GpuShape& output)
        {
            if (!device || !mesh.valid || mesh.vertices.empty() || mesh.indices.empty()) {
                return false;
            }

            D3D11_BUFFER_DESC vertexDesc{};
            vertexDesc.Usage = D3D11_USAGE_DEFAULT;
            vertexDesc.ByteWidth = static_cast<UINT>(sizeof(Vertex) * mesh.vertices.size());
            vertexDesc.BindFlags = D3D11_BIND_VERTEX_BUFFER;
            D3D11_SUBRESOURCE_DATA vertexData{};
            vertexData.pSysMem = mesh.vertices.data();
            if (FAILED(device->CreateBuffer(&vertexDesc, &vertexData, output.vertexBuffer.GetAddressOf()))) {
                return false;
            }

            D3D11_BUFFER_DESC indexDesc{};
            indexDesc.Usage = D3D11_USAGE_DEFAULT;
            indexDesc.ByteWidth = static_cast<UINT>(sizeof(std::uint16_t) * mesh.indices.size());
            indexDesc.BindFlags = D3D11_BIND_INDEX_BUFFER;
            D3D11_SUBRESOURCE_DATA indexData{};
            indexData.pSysMem = mesh.indices.data();
            if (FAILED(device->CreateBuffer(&indexDesc, &indexData, output.indexBuffer.GetAddressOf()))) {
                output.vertexBuffer.Reset();
                return false;
            }

            output.indexCount = static_cast<std::uint32_t>(mesh.indices.size());
            output.decodeMode = debug_overlay_policy::ShapeDecodeMode::Proxy;
            output.approximateBytes = debug_overlay_shape::approximateGpuBytes(mesh);
            return true;
        }

        bool finiteShapeVector3(const float* values)
        {
            return values && std::isfinite(values[0]) && std::isfinite(values[1]) && std::isfinite(values[2]);
        }

        bool finiteShapeTransform(const float* values)
        {
            if (!values) {
                return false;
            }
            for (std::size_t index = 0; index < 16; ++index) {
                if (!std::isfinite(values[index])) {
                    return false;
                }
            }
            return true;
        }

        bool captureShapeRecipeUnsafe(
            std::uintptr_t shapeAddress,
            const OverlayRenderSettings& settings,
            debug_overlay_shape::ShapeRecipe& recipe,
            int depth)
        {
            recipe = {};
            recipe.settings.havokToGameScale = havokToGameScale();
            recipe.settings.maxConvexSupportVertices = settings.limits.maxConvexSupportVertices;
            recipe.settings.maxCompoundChildren = settings.limits.maxCompoundChildren;
            recipe.settings.maxCompoundDepth = settings.limits.maxCompoundDepth;
            recipe.settings.useBoundsForHeavyConvex = settings.useBoundsForHeavyConvex;
            if (!shapeAddress || depth > static_cast<int>(settings.limits.maxCompoundDepth) || !std::isfinite(recipe.settings.havokToGameScale) ||
                recipe.settings.havokToGameScale <= 0.0f) {
                return false;
            }

            auto* shape = reinterpret_cast<RE::hknpShape*>(shapeAddress);
            recipe.shapeType = static_cast<int>(shape->GetType());
            const float convexRadius = shape->convexRadius;
            if (!std::isfinite(convexRadius)) {
                return false;
            }
            recipe.convexRadius = convexRadius;

            switch (recipe.shapeType) {
            case 0:
            case 1:
            case 4: {
                const int vertexCount = shape->GetNumberOfSupportVertices();
                if (vertexCount < 3 || vertexCount > 256) {
                    return false;
                }

                std::array<RE::hkVector4f, 256> supportVertices{};
                auto* supportBuffer = reinterpret_cast<RE::hkcdVertex*>(supportVertices.data());
                const auto* resultRaw = shape->GetSupportVertices(supportBuffer, vertexCount);
                if (!resultRaw) {
                    return false;
                }

                const auto* resultVertices = reinterpret_cast<const RE::hkVector4f*>(resultRaw);
                recipe.vertices.reserve(static_cast<std::size_t>(vertexCount));
                for (int index = 0; index < vertexCount; ++index) {
                    const auto* values = reinterpret_cast<const float*>(&resultVertices[index]);
                    if (!finiteShapeVector3(values)) {
                        return false;
                    }
                    recipe.vertices.push_back(Vertex{
                        values[0] * recipe.settings.havokToGameScale,
                        values[1] * recipe.settings.havokToGameScale,
                        values[2] * recipe.settings.havokToGameScale
                    });
                }
                if (recipe.shapeType == 4) {
                    recipe.vertices.resize(3);
                    recipe.kind = debug_overlay_shape::ShapeRecipe::Kind::Triangle;
                } else {
                    recipe.kind = debug_overlay_shape::ShapeRecipe::Kind::ConvexVertices;
                }
                recipe.valid = true;
                return true;
            }
            case 2:
                recipe.kind = debug_overlay_shape::ShapeRecipe::Kind::Sphere;
                recipe.canonicalUnitSphere = depth == 0;
                recipe.valid = true;
                return true;
            case 3: {
                const auto* vertexA = reinterpret_cast<const float*>(shapeAddress + 0x50);
                const auto* vertexB = reinterpret_cast<const float*>(shapeAddress + 0x60);
                if (!finiteShapeVector3(vertexA) || !finiteShapeVector3(vertexB)) {
                    return false;
                }
                std::copy_n(vertexA, recipe.vertexA.size(), recipe.vertexA.begin());
                std::copy_n(vertexB, recipe.vertexB.size(), recipe.vertexB.begin());
                recipe.kind = debug_overlay_shape::ShapeRecipe::Kind::Capsule;
                recipe.valid = true;
                return true;
            }
            case 7:
            case 8: {
                const auto slotArray = *reinterpret_cast<const std::uintptr_t*>(shapeAddress + kCompoundSlotArrayOffset);
                const auto slotCount = *reinterpret_cast<const std::int32_t*>(shapeAddress + kCompoundSlotCountOffset);
                if (!slotArray || slotCount <= 0 ||
                    static_cast<std::uint32_t>(slotCount) > settings.limits.maxCompoundChildren) {
                    return false;
                }

                recipe.kind = debug_overlay_shape::ShapeRecipe::Kind::Compound;
                recipe.children.reserve(static_cast<std::size_t>(slotCount));
                for (std::int32_t index = 0; index < slotCount; ++index) {
                    const auto slotOffset = static_cast<std::uintptr_t>(index) * kCompoundSlotStride;
                    if (slotArray > (std::numeric_limits<std::uintptr_t>::max)() - slotOffset) {
                        return false;
                    }
                    const auto slot = slotArray + slotOffset;
                    if (*reinterpret_cast<const std::uint8_t*>(slot + kCompoundSlotActiveOffset) != 0) {
                        continue;
                    }

                    const auto childShapeAddress = *reinterpret_cast<const std::uintptr_t*>(slot + kCompoundSlotShapeOffset);
                    const auto* transform = reinterpret_cast<const float*>(slot + kCompoundSlotTransformOffset);
                    const auto* childScale = reinterpret_cast<const float*>(slot + kCompoundSlotScaleOffset);
                    if (!childShapeAddress || !finiteShapeTransform(transform) || !finiteShapeVector3(childScale)) {
                        return false;
                    }

                    recipe.children.emplace_back();
                    auto& child = recipe.children.back();
                    std::copy_n(transform, child.transform.size(), child.transform.begin());
                    std::copy_n(childScale, 3, child.scale.begin());
                    child.recipe = std::make_unique<debug_overlay_shape::ShapeRecipe>();
                    if (!captureShapeRecipeUnsafe(childShapeAddress, settings, *child.recipe, depth + 1)) {
                        return false;
                    }
                }
                recipe.valid = !recipe.children.empty();
                return recipe.valid;
            }
            case 11: {
                const auto innerShapeAddress = *reinterpret_cast<const std::uintptr_t*>(shapeAddress + kScaledConvexInnerShapeOffset);
                const auto* scaleVector = reinterpret_cast<const float*>(shapeAddress + kScaledConvexScaleOffset);
                const auto* translationVector = reinterpret_cast<const float*>(shapeAddress + kScaledConvexTranslationOffset);
                if (!innerShapeAddress || !finiteShapeVector3(scaleVector) || !finiteShapeVector3(translationVector)) {
                    return false;
                }

                recipe.inner = std::make_unique<debug_overlay_shape::ShapeRecipe>();
                if (!captureShapeRecipeUnsafe(innerShapeAddress, settings, *recipe.inner, depth + 1)) {
                    return false;
                }
                std::copy_n(scaleVector, 3, recipe.scale.begin());
                std::copy_n(translationVector, 3, recipe.translation.begin());
                recipe.kind = debug_overlay_shape::ShapeRecipe::Kind::ScaledConvex;
                recipe.valid = true;
                return true;
            }
            default:
                return false;
            }
        }

        bool captureShapeRecipeSeh(
            std::uintptr_t shapeAddress,
            const OverlayRenderSettings* settings,
            debug_overlay_shape::ShapeRecipe* recipe)
        {
            if (!settings || !recipe) {
                return false;
            }
            __try {
                return captureShapeRecipeUnsafe(shapeAddress, *settings, *recipe, 0);
            } __except (EXCEPTION_EXECUTE_HANDLER) {
                return false;
            }
        }

        bool captureShapeRecipeGuarded(
            std::uintptr_t shapeAddress,
            const OverlayRenderSettings& settings,
            debug_overlay_shape::ShapeRecipe& recipe)
        {
            recipe = {};
            if (!captureShapeRecipeSeh(shapeAddress, &settings, &recipe)) {
                recipe = {};
                return false;
            }
            return recipe.valid;
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
            debug_overlay_runtime::RequestedLimits requested{};
            requested.maxShapeCapturesPerFrame = g_rockConfig.rockDebugMaxShapeCapturesPerFrame;
            requested.maxConvexSupportVertices = g_rockConfig.rockDebugMaxConvexSupportVertices;
            requested.maxCompoundChildren = g_rockConfig.rockDebugMaxCompoundChildren;
            requested.maxCompoundDepth = g_rockConfig.rockDebugMaxCompoundDepth;
            requested.maxShapeQueuedJobs = g_rockConfig.rockDebugMaxShapeQueuedJobs;
            requested.maxShapeCompletedJobs = g_rockConfig.rockDebugMaxShapeCompletedJobs;
            requested.maxShapeUploadsPerFrame = g_rockConfig.rockDebugMaxShapeUploadsPerFrame;
            requested.maxShapeCacheEntries = g_rockConfig.rockDebugMaxShapeCacheEntries;
            requested.maxShapeCacheBytes = g_rockConfig.rockDebugMaxShapeCacheBytes;
            requested.maxBodyInstances = g_rockConfig.rockDebugMaxBodyInstances;
            requested.maxLineVertices = g_rockConfig.rockDebugMaxLineVertices;
            requested.maxTextVertices = g_rockConfig.rockDebugMaxTextVertices;
            settings.limits = debug_overlay_runtime::sanitize(requested);
            settings.useBoundsForHeavyConvex = g_rockConfig.rockDebugUseBoundsForHeavyConvex;
            settings.shapeDecodeSettingsKey = debug_overlay_policy::makeShapeDecodeSettingsKey(
                static_cast<int>(settings.limits.maxConvexSupportVertices),
                settings.useBoundsForHeavyConvex,
                static_cast<int>(settings.limits.maxCompoundChildren),
                static_cast<int>(settings.limits.maxCompoundDepth));
            settings.pipelineLimits.maxQueuedJobs = settings.limits.maxShapeQueuedJobs;
            settings.pipelineLimits.maxCompletedJobs = settings.limits.maxShapeCompletedJobs;
            settings.pipelineLimits.maxCacheEntries = settings.limits.maxShapeCacheEntries;
            settings.pipelineLimits.maxGpuBytes = settings.limits.maxShapeCacheBytes;
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
            frame.coloredLines.clear();
            frame.text.clear();
            frame.capturedShapeIdentities.clear();
            frame.settings = {};
            frame.worldIdentity = 0;
            frame.bodyExtractFailures = 0;
            frame.shapeCaptures = 0;
            frame.shapeCaptureDeferrals = 0;
            frame.drawRockBodies = false;
            frame.drawTargetBodies = false;
            frame.drawAxes = false;
            frame.drawMarkers = false;
            frame.drawSkeleton = false;
            frame.drawColoredLines = false;
            frame.drawText = false;
        }

        CapturedShapeIdentity captureShapeIdentityForFrame(PublishedOverlayFrame& frame, std::uintptr_t shapeAddress)
        {
            for (const auto& captured : frame.capturedShapeIdentities) {
                if (captured.shapeAddress == shapeAddress) {
                    return captured;
                }
            }

            int shapeType = -1;
            float convexRadius = 0.0f;
            ShapeKey key = makeShapeKey(shapeAddress, frame.settings, shapeType, convexRadius);
            float detailUniformScale = 1.0f;
            if (shapeType == 2 && std::isfinite(convexRadius) && convexRadius > 0.0f) {
                key = ShapeKey{ 0, kCanonicalSphereGeometryFingerprint };
                detailUniformScale = convexRadius * havokToGameScale();
            }

            const CapturedShapeIdentity captured{ shapeAddress, key, detailUniformScale };
            frame.capturedShapeIdentities.push_back(captured);
            return captured;
        }

        void requestShapeBuildForFrame(
            PublishedOverlayFrame& frame, const ShapeKey& key, std::uintptr_t shapeAddress)
        {
            if (shapePipeline().lookup(key).state != debug_overlay_shape::CacheState::Missing) {
                return;
            }
            if (frame.shapeCaptures >= frame.settings.limits.maxShapeCapturesPerFrame) {
                ++frame.shapeCaptureDeferrals;
                return;
            }

            const auto reservation = shapePipeline().reserve(key, frame.settings.pipelineLimits);
            if (reservation.status != debug_overlay_shape::ReserveStatus::Reserved) {
                if (reservation.status == debug_overlay_shape::ReserveStatus::QueueFull ||
                    reservation.status == debug_overlay_shape::ReserveStatus::NotRunning) {
                    ++frame.shapeCaptureDeferrals;
                } else if (reservation.status == debug_overlay_shape::ReserveStatus::CacheFull) {
                    ++frame.shapeCaptureDeferrals;
                }
                return;
            }

            ++frame.shapeCaptures;
            debug_overlay_shape::ShapeRecipe recipe{};
            bool captured = false;
            try {
                captured = captureShapeRecipeGuarded(shapeAddress, frame.settings, recipe);
            } catch (...) {
                captured = false;
            }
            if (!captured) {
                shapePipeline().markUnsupported(reservation.reservation);
                return;
            }
            if (!shapePipeline().submit(reservation.reservation, std::move(recipe))) {
                ++frame.shapeCaptureDeferrals;
            }
        }

        bool buildPublishedFrame(const BodyOverlayFrame& source, PublishedOverlayFrame& destination)
        {
            resetPublishedFrame(destination);
            destination.settings = captureOverlayRenderSettings();
            destination.worldIdentity = reinterpret_cast<std::uintptr_t>(source.world);
            if (!destination.worldIdentity) {
                return false;
            }

            if (!shapePipeline().initialize()) {
                if (!s_shapeWorkerInitFailureReported.exchange(true, std::memory_order_relaxed)) {
                    ROCK_LOG_ERROR(Hand, "Debug body overlay: failed to initialize the owned shape worker");
                }
            } else {
                s_shapeWorkerInitFailureReported.store(false, std::memory_order_relaxed);
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
            destination.drawColoredLines = source.drawColoredLines;
            destination.drawText = source.drawText;

            const auto bodyCount = (std::min)(source.count, static_cast<std::uint32_t>(source.entries.size()));
            destination.bodies.reserve(source.entries.size());
            destination.capturedShapeIdentities.reserve(source.entries.size());
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
                const auto shapeIdentity = captureShapeIdentityForFrame(destination, body.shapeAddress);
                published.shapeKey = shapeIdentity.key;
                published.worldMatrix = body.worldMatrix;
                published.role = entry.role;
                published.bodyId = body.bodyId;
                published.detailUniformScale = shapeIdentity.detailUniformScale;
                published.hasValidWorldAabb =
                    captureBodyWorldAabb(source.world, entry.bodyId, published.worldAabbMin, published.worldAabbMax);
                requestShapeBuildForFrame(destination, published.shapeKey, body.shapeAddress);
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
            if (source.drawColoredLines) {
                const auto count = (std::min)(
                    source.coloredLineCount,
                    provider::ROCK_PROVIDER_MAX_DEBUG_OVERLAY_LINES_V1);
                if (source.coloredLineEntries && count > 0) {
                    destination.coloredLines.resize(count);
                    for (std::uint32_t index = 0; index < count; ++index) {
                        const auto& providerLine = source.coloredLineEntries[index];
                        auto& line = destination.coloredLines[index];
                        line.start = RE::NiPoint3(
                            providerLine.startGame[0],
                            providerLine.startGame[1],
                            providerLine.startGame[2]);
                        line.end = RE::NiPoint3(
                            providerLine.endGame[0],
                            providerLine.endGame[1],
                            providerLine.endGame[2]);
                        std::copy_n(providerLine.color, 4, line.color);
                    }
                }
            }
            if (source.drawText) {
                const auto count = (std::min)(source.textCount, static_cast<std::uint32_t>(source.textEntries.size()));
                destination.text.assign(source.textEntries.begin(), source.textEntries.begin() + count);
            }

            return !destination.bodies.empty() || !destination.axes.empty() || !destination.markers.empty() ||
                   !destination.skeleton.empty() || !destination.coloredLines.empty() || !destination.text.empty();
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
                debug_overlay_shaders::kInstancedBodyVertex,
                sizeof(debug_overlay_shaders::kInstancedBodyVertex) - 1,
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

            hr = device->CreateVertexShader(vsBlob->GetBufferPointer(), vsBlob->GetBufferSize(), nullptr, resources.bodyVertexShader.GetAddressOf());
            if (FAILED(hr)) {
                return false;
            }

            const D3D11_INPUT_ELEMENT_DESC bodyLayoutDesc[] = {
                { "POS", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0 },
                { "IROW", 0, DXGI_FORMAT_R32G32B32A32_FLOAT, 1, 0, D3D11_INPUT_PER_INSTANCE_DATA, 2 },
                { "IROW", 1, DXGI_FORMAT_R32G32B32A32_FLOAT, 1, 16, D3D11_INPUT_PER_INSTANCE_DATA, 2 },
                { "IROW", 2, DXGI_FORMAT_R32G32B32A32_FLOAT, 1, 32, D3D11_INPUT_PER_INSTANCE_DATA, 2 },
                { "IROW", 3, DXGI_FORMAT_R32G32B32A32_FLOAT, 1, 48, D3D11_INPUT_PER_INSTANCE_DATA, 2 },
                { "ICOLOR", 0, DXGI_FORMAT_R32G32B32A32_FLOAT, 1, 64, D3D11_INPUT_PER_INSTANCE_DATA, 2 },
            };
            hr = device->CreateInputLayout(bodyLayoutDesc, static_cast<UINT>(std::size(bodyLayoutDesc)), vsBlob->GetBufferPointer(), vsBlob->GetBufferSize(), resources.bodyInputLayout.GetAddressOf());
            if (FAILED(hr)) {
                return false;
            }

            vsBlob.Reset();
            errorBlob.Reset();
            hr = D3DCompile(
                debug_overlay_shaders::kStereoColorVertex,
                sizeof(debug_overlay_shaders::kStereoColorVertex) - 1,
                "ROCKDebugColorVS",
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
                    ROCK_LOG_ERROR(Hand, "Debug overlay color vertex shader compile failed: {}", static_cast<const char*>(errorBlob->GetBufferPointer()));
                }
                return false;
            }

            hr = device->CreateVertexShader(vsBlob->GetBufferPointer(), vsBlob->GetBufferSize(), nullptr, resources.stereoColorVertexShader.GetAddressOf());
            if (FAILED(hr)) {
                return false;
            }

            const D3D11_INPUT_ELEMENT_DESC coloredLayoutDesc[] = {
                { "POS", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0 },
                { "COLOR", 0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, 12, D3D11_INPUT_PER_VERTEX_DATA, 0 },
            };
            hr = device->CreateInputLayout(coloredLayoutDesc, static_cast<UINT>(std::size(coloredLayoutDesc)), vsBlob->GetBufferPointer(), vsBlob->GetBufferSize(), resources.coloredInputLayout.GetAddressOf());
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

            D3D11_BUFFER_DESC instanceDesc{};
            instanceDesc.Usage = D3D11_USAGE_DYNAMIC;
            instanceDesc.ByteWidth = static_cast<UINT>(sizeof(BodyInstanceData) * kBodyInstanceCapacity);
            instanceDesc.BindFlags = D3D11_BIND_VERTEX_BUFFER;
            instanceDesc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
            if (FAILED(device->CreateBuffer(&instanceDesc, nullptr, resources.bodyInstanceVB.GetAddressOf()))) {
                return false;
            }

            D3D11_BUFFER_DESC axisLineDesc{};
            axisLineDesc.Usage = D3D11_USAGE_DYNAMIC;
            axisLineDesc.ByteWidth = sizeof(ColoredVertex) * debug_overlay_policy::kMaxLineVertexBudget;
            axisLineDesc.BindFlags = D3D11_BIND_VERTEX_BUFFER;
            axisLineDesc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
            if (FAILED(device->CreateBuffer(&axisLineDesc, nullptr, resources.axisLineVB.GetAddressOf()))) {
                return false;
            }

            D3D11_BUFFER_DESC textDesc{};
            textDesc.Usage = D3D11_USAGE_DYNAMIC;
            textDesc.ByteWidth = sizeof(ColoredVertex) * debug_overlay_runtime::kMaxTextVertices;
            textDesc.BindFlags = D3D11_BIND_VERTEX_BUFFER;
            textDesc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
            if (FAILED(device->CreateBuffer(&textDesc, nullptr, resources.textVB.GetAddressOf()))) {
                return false;
            }

            try {
                resources.scratch = std::make_unique<RenderScratch>();
            } catch (...) {
                return false;
            }
            if (!resources.scratch->prepare()) {
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

            if (!createStaticGpuShape(device, debug_overlay_shape::makeUnitBoxMesh(), resources.aabbProxy)) {
                return false;
            }

            if (!resources.ready()) {
                return false;
            }

            if (resources.gpuTimer.initialize(device)) {
                s_gpuTimerInitFailureReported.store(false, std::memory_order_relaxed);
            } else if (!s_gpuTimerInitFailureReported.exchange(true, std::memory_order_relaxed)) {
                ROCK_LOG_WARN(Hand, "Debug overlay: GPU timestamp queries unavailable; rendering will continue without GPU timing");
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
            context->IASetInputLayout(s_d3d.bodyInputLayout.Get());
            context->VSSetShader(s_d3d.bodyVertexShader.Get(), nullptr, 0);
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

        void bodyColor(BodyOverlayRole role, debug_overlay_policy::ShapeDecodeMode decodeMode, float color[4])
        {
            color[0] = 1.0f;
            color[1] = 1.0f;
            color[2] = 1.0f;
            color[3] = 0.85f;
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

        void appendDebugLine(debug_overlay_line_batch::LineBatch& batch, const Vertex& start, const Vertex& end, const float color[4])
        {
            batch.addLine(toLineVec(start), toLineVec(end), toLineColor(color));
        }

        void appendPointMarker(debug_overlay_line_batch::LineBatch& batch, const RE::NiPoint3& position, float size, const float color[4])
        {
            batch.addPointMarker(toLineVec(toVertex(position)), size, toLineColor(color));
        }

        void drawLineBatch(ID3D11DeviceContext* context, const debug_overlay_line_batch::LineBatch& batch, OverlayRuntimeStats& stats)
        {
            stats.lineVertices += static_cast<std::uint32_t>(batch.vertexCount());
            stats.lineLogicalLines += static_cast<std::uint32_t>(batch.lineCount());
            stats.lineBudgetRejects += static_cast<std::uint32_t>(batch.rejectedLineCount());

            if (batch.empty() || !s_d3d.axisLineVB || !s_d3d.stereoColorVertexShader || !s_d3d.coloredInputLayout) {
                return;
            }

            D3D11_MAPPED_SUBRESOURCE mapped{};
            if (FAILED(context->Map(s_d3d.axisLineVB.Get(), 0, D3D11_MAP_WRITE_DISCARD, 0, &mapped)) || !mapped.pData) {
                ++stats.lineMapFailures;
                if (!s_lineUploadFailureReported.exchange(true, std::memory_order_relaxed)) {
                    ROCK_LOG_WARN(Hand, "Debug overlay: colored line vertex-buffer map failed; line batch skipped");
                }
                return;
            }

            auto* vertices = static_cast<ColoredVertex*>(mapped.pData);
            std::size_t vertexIndex = 0;
            for (const auto& segment : batch.segments()) {
                const ColoredVertex start{
                    segment.start.x,
                    segment.start.y,
                    segment.start.z,
                    { segment.color.r, segment.color.g, segment.color.b, segment.color.a }
                };
                const ColoredVertex end{
                    segment.end.x,
                    segment.end.y,
                    segment.end.z,
                    { segment.color.r, segment.color.g, segment.color.b, segment.color.a }
                };
                vertices[vertexIndex++] = start;
                vertices[vertexIndex++] = end;
            }
            context->Unmap(s_d3d.axisLineVB.Get(), 0);

            context->IASetInputLayout(s_d3d.coloredInputLayout.Get());
            context->VSSetShader(s_d3d.stereoColorVertexShader.Get(), nullptr, 0);
            context->PSSetShader(s_d3d.pixelShader.Get(), nullptr, 0);
            context->RSSetState(s_d3d.wireRasterizer.Get());
            context->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_LINELIST);

            constexpr UINT stride = sizeof(ColoredVertex);
            constexpr UINT offset = 0;
            ID3D11Buffer* vertexBuffer = s_d3d.axisLineVB.Get();
            context->IASetVertexBuffers(0, 1, &vertexBuffer, &stride, &offset);
            context->IASetIndexBuffer(nullptr, DXGI_FORMAT_UNKNOWN, 0);
            context->DrawInstanced(static_cast<UINT>(batch.vertexCount()), 2, 0, 0);
            ++stats.lineDrawCalls;
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

        bool appendTextQuad(
            std::vector<ColoredVertex>& vertices,
            float x,
            float y,
            float size,
            float textureWidth,
            float textureHeight,
            const float color[4],
            std::uint32_t maxVertices,
            std::uint32_t& rejectedVertices)
        {
            if (vertices.size() + 6 > maxVertices) {
                rejectedVertices += 6;
                return false;
            }

            const auto toClip = [&](float px, float py) {
                return Vertex{ (px / textureWidth) * 2.0f - 1.0f, 1.0f - (py / textureHeight) * 2.0f, 0.0f };
            };

            const auto colored = [&](const Vertex& position) {
                return ColoredVertex{ position.x, position.y, position.z, { color[0], color[1], color[2], color[3] } };
            };
            const ColoredVertex a = colored(toClip(x, y));
            const ColoredVertex b = colored(toClip(x + size, y));
            const ColoredVertex c = colored(toClip(x + size, y + size));
            const ColoredVertex d = colored(toClip(x, y + size));
            vertices.push_back(a);
            vertices.push_back(b);
            vertices.push_back(c);
            vertices.push_back(a);
            vertices.push_back(c);
            vertices.push_back(d);
            return true;
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

        void appendTextGlyphs(
            std::vector<ColoredVertex>& vertices,
            const TextOverlayEntry& entry,
            float baseX,
            float baseY,
            float maxX,
            float textureWidth,
            float textureHeight,
            std::uint32_t maxVertices,
            std::uint32_t& rejectedVertices)
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
                            (void)appendTextQuad(
                                vertices,
                                cursorX + static_cast<float>(col) * pixel,
                                cursorY + static_cast<float>(row) * pixel,
                                pixel,
                                textureWidth,
                                textureHeight,
                                entry.color,
                                maxVertices,
                                rejectedVertices);
                        }
                    }
                }
                cursorX += kGlyphAdvanceColumns * pixel;
                if (cursorX >= maxX || cursorX >= textureWidth - 8.0f) {
                    break;
                }
            }
        }

        void appendWorldAnchoredTextGlyphs(std::vector<ColoredVertex>& vertices,
            const TextOverlayEntry& entry,
            const DirectX::XMMATRIX& eye0,
            const DirectX::XMMATRIX& eye1,
            const DirectX::XMFLOAT4& adjust0,
            const DirectX::XMFLOAT4& adjust1,
            float textureWidth,
            float textureHeight,
            bool duplicatePerEye,
            std::uint32_t maxVertices,
            std::uint32_t& rejectedVertices)
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
                appendTextGlyphs(vertices, entry, baseX, baseY, eyeMaxX - 8.0f, textureWidth, textureHeight, maxVertices, rejectedVertices);
            };

            appendEye(0, eye0, adjust0);
            if (duplicatePerEye) {
                appendEye(1, eye1, adjust1);
            }
        }

        void collectColoredLineOverlays(debug_overlay_line_batch::LineBatch& batch, const PublishedOverlayFrame& frame)
        {
            if (!frame.drawColoredLines || frame.coloredLines.empty()) {
                return;
            }

            for (const auto& entry : frame.coloredLines) {
                appendDebugLine(batch, toVertex(entry.start), toVertex(entry.end), entry.color);
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
            if (!frame.drawText || frame.text.empty() || !s_d3d.textVB || !s_d3d.screenTextVertexShader || !s_d3d.coloredInputLayout || !s_d3d.scratch ||
                textureWidth <= 0.0f || textureHeight <= 0.0f) {
                return;
            }

            const bool duplicatePerEye = frame.settings.duplicateTextPerEye;
            const float eyeWidth = duplicatePerEye ? textureWidth * 0.5f : textureWidth;
            auto& vertices = s_d3d.scratch->textVertices;
            vertices.clear();
            std::uint32_t rejectedVertices = 0;
            const auto maxVertices = frame.settings.limits.maxTextVertices;
            for (const auto& entry : frame.text) {
                const std::uint32_t rejectedBefore = rejectedVertices;
                if (entry.worldAnchored) {
                    appendWorldAnchoredTextGlyphs(
                        vertices, entry, eye0, eye1, adjust0, adjust1, textureWidth, textureHeight, duplicatePerEye, maxVertices, rejectedVertices);
                } else {
                    appendTextGlyphs(vertices, entry, entry.x, entry.y, eyeWidth - 8.0f, textureWidth, textureHeight, maxVertices, rejectedVertices);
                    if (duplicatePerEye) {
                        appendTextGlyphs(
                            vertices, entry, entry.x + eyeWidth, entry.y, textureWidth - 8.0f, textureWidth, textureHeight, maxVertices, rejectedVertices);
                    }
                }
                if (rejectedVertices != rejectedBefore) {
                    ++stats.textVertexTruncations;
                }
            }
            stats.textRejectedVertices += rejectedVertices;
            if (vertices.empty()) {
                return;
            }

            D3D11_MAPPED_SUBRESOURCE mapped{};
            if (FAILED(context->Map(s_d3d.textVB.Get(), 0, D3D11_MAP_WRITE_DISCARD, 0, &mapped)) || !mapped.pData) {
                ++stats.textMapFailures;
                if (!s_textUploadFailureReported.exchange(true, std::memory_order_relaxed)) {
                    ROCK_LOG_WARN(Hand, "Debug overlay: aggregate colored text vertex-buffer map failed; text batch skipped");
                }
                return;
            }
            std::memcpy(mapped.pData, vertices.data(), vertices.size() * sizeof(ColoredVertex));
            context->Unmap(s_d3d.textVB.Get(), 0);

            context->IASetInputLayout(s_d3d.coloredInputLayout.Get());
            context->VSSetShader(s_d3d.screenTextVertexShader.Get(), nullptr, 0);
            context->PSSetShader(s_d3d.pixelShader.Get(), nullptr, 0);
            context->RSSetState(s_d3d.solidRasterizer.Get());
            context->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

            constexpr UINT stride = sizeof(ColoredVertex);
            constexpr UINT offset = 0;
            ID3D11Buffer* vertexBuffer = s_d3d.textVB.Get();
            context->IASetVertexBuffers(0, 1, &vertexBuffer, &stride, &offset);
            context->IASetIndexBuffer(nullptr, DXGI_FORMAT_UNKNOWN, 0);
            context->Draw(static_cast<UINT>(vertices.size()), 0);
            stats.textVertices += static_cast<std::uint32_t>(vertices.size());
            ++stats.textDrawCalls;
        }

        void drawBodyBatch(ID3D11DeviceContext* context, const PublishedOverlayFrame& frame, OverlayRuntimeStats& stats)
        {
            if (!context || !s_d3d.bodyInstanceVB || !s_d3d.bodyInputLayout || !s_d3d.bodyVertexShader || !s_d3d.scratch) {
                return;
            }

            auto& draws = s_d3d.scratch->bodies;
            draws.clear();
            for (const auto& entry : frame.bodies) {
                ++stats.bodyEntries;
                const auto cached = shapePipeline().lookup(entry.shapeKey);
                std::shared_ptr<const GpuShape> shapeOwner;
                const GpuShape* gpuShape = nullptr;
                DirectX::XMMATRIX model = entry.worldMatrix;
                if (cached.state == debug_overlay_shape::CacheState::Ready && cached.shape && cached.shape->indexCount > 0) {
                    shapeOwner = cached.shape;
                    gpuShape = shapeOwner.get();
                    if (entry.detailUniformScale != 1.0f) {
                        model = DirectX::XMMatrixScaling(
                            entry.detailUniformScale, entry.detailUniformScale, entry.detailUniformScale) * model;
                    }
                    ++stats.shapeCacheHits;
                    if (gpuShape->decodeMode == debug_overlay_policy::ShapeDecodeMode::Proxy) {
                        ++stats.shapeProxyFallbacks;
                    }
                } else {
                    ++stats.shapeCacheMisses;
                    if (!entry.hasValidWorldAabb) {
                        if (cached.state == debug_overlay_shape::CacheState::Unsupported) {
                            ++stats.unsupportedShapeSkips;
                        }
                        continue;
                    }
                    gpuShape = &s_d3d.aabbProxy;
                    model = worldAabbMatrix(entry);
                    ++stats.shapeProxyFallbacks;
                    if (cached.state == debug_overlay_shape::CacheState::Unsupported) {
                        ++stats.unsupportedShapeProxies;
                    } else {
                        ++stats.shapePendingProxies;
                    }
                }
                if (!gpuShape || !gpuShape->vertexBuffer || !gpuShape->indexBuffer || gpuShape->indexCount == 0) {
                    continue;
                }
                if (draws.size() >= frame.settings.limits.maxBodyInstances) {
                    ++stats.bodyInstanceRejects;
                    continue;
                }

                BodyDrawItem draw{};
                draw.shapeOwner = std::move(shapeOwner);
                draw.shape = gpuShape;
                DirectX::XMStoreFloat4x4(&draw.instance.model, model);
                bodyColor(entry.role, gpuShape->decodeMode, draw.instance.color);
                draws.push_back(std::move(draw));
            }
            if (draws.empty()) {
                return;
            }

            D3D11_MAPPED_SUBRESOURCE mapped{};
            if (FAILED(context->Map(s_d3d.bodyInstanceVB.Get(), 0, D3D11_MAP_WRITE_DISCARD, 0, &mapped)) || !mapped.pData) {
                if (!s_bodyInstanceUploadFailureReported.exchange(true, std::memory_order_relaxed)) {
                    ROCK_LOG_WARN(Hand, "Debug overlay: body instance-buffer map failed; body batch skipped");
                }
                return;
            }
            auto* instances = static_cast<BodyInstanceData*>(mapped.pData);
            for (std::size_t index = 0; index < draws.size(); ++index) {
                instances[index] = draws[index].instance;
            }
            context->Unmap(s_d3d.bodyInstanceVB.Get(), 0);
            ++stats.bodyInstanceMaps;

            context->IASetInputLayout(s_d3d.bodyInputLayout.Get());
            context->VSSetShader(s_d3d.bodyVertexShader.Get(), nullptr, 0);
            context->PSSetShader(s_d3d.pixelShader.Get(), nullptr, 0);
            context->RSSetState(s_d3d.wireRasterizer.Get());
            context->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

            std::size_t runStart = 0;
            while (runStart < draws.size()) {
                std::size_t runEnd = runStart + 1;
                while (runEnd < draws.size() && draws[runEnd].shape == draws[runStart].shape) {
                    ++runEnd;
                }

                const auto* shape = draws[runStart].shape;
                ID3D11Buffer* vertexBuffers[2] = { shape->vertexBuffer.Get(), s_d3d.bodyInstanceVB.Get() };
                const UINT strides[2] = { sizeof(Vertex), sizeof(BodyInstanceData) };
                const UINT offsets[2] = { 0, static_cast<UINT>(runStart * sizeof(BodyInstanceData)) };
                context->IASetVertexBuffers(0, 2, vertexBuffers, strides, offsets);
                context->IASetIndexBuffer(shape->indexBuffer.Get(), DXGI_FORMAT_R16_UINT, 0);

                const UINT runLength = static_cast<UINT>(runEnd - runStart);
                context->DrawIndexedInstanced(shape->indexCount, runLength * 2, 0, 0, 0);
                stats.bodiesDrawn += runLength;
                ++stats.bodyMeshBinds;
                ++stats.bodyDrawCalls;
                runStart = runEnd;
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
            const bool hasColoredLinesToDraw = frame->drawColoredLines && !frame->coloredLines.empty();
            const bool hasTextToDraw = frame->drawText && !frame->text.empty();
            if ((!hasBodiesToDraw && !hasAxesToDraw && !hasMarkersToDraw && !hasSkeletonToDraw && !hasColoredLinesToDraw && !hasTextToDraw) || !frame->worldIdentity) {
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
            stats.shapeCaptures = frame->shapeCaptures;
            stats.shapeCaptureDeferrals = frame->shapeCaptureDeferrals;
            const auto uploadResult = shapePipeline().processCompletedUploads(
                device, frame->settings.limits.maxShapeUploadsPerFrame, frame->settings.pipelineLimits);
            stats.shapeUploadsProcessed = uploadResult.processed;
            stats.shapeUploadsCompleted = uploadResult.uploaded;
            stats.shapeUploadFailures = uploadResult.failed;
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

            {
                [[maybe_unused]] auto gpuTimerScope = s_d3d.gpuTimer.begin(context);
                if (hasBodiesToDraw) {
                    drawBodyBatch(context, *frame, stats);
                }

                auto& lineBatch = s_d3d.scratch->lines;
                lineBatch.beginFrame(frame->settings.limits.maxLineVertices);
                // Owner-published diagnostics are already hard-bounded by the
                // provider API. Admit them first so an enabled addon view is
                // not silently starved by unrelated high-cardinality probes.
                collectColoredLineOverlays(lineBatch, *frame);
                collectAxisOverlays(lineBatch, *frame);
                collectMarkerOverlays(lineBatch, *frame);
                collectSkeletonOverlays(lineBatch, *frame);
                drawLineBatch(context, lineBatch, stats);
                drawTextOverlays(context, static_cast<float>(textureDesc.Width), static_cast<float>(textureDesc.Height), *frame, eye0, eye1, adjust0, adjust1, stats);
            }

            if (frame->settings.verboseLogging && ++s_overlayStatsLogCounter >= 90) {
                s_overlayStatsLogCounter = 0;
                const auto pipelineStats = shapePipeline().stats();
                const auto gpuStats = s_d3d.gpuTimer.stats();
                const auto admissionStats = s_frameAdmission.stats();
                ROCK_LOG_DEBUG(Hand,
                    "Debug overlay frame: entries={} drawn={} bodyBinds={} bodyDraws={} bodyMaps={} bodyRejects={} axes={} markers={} skeleton={} text={} cacheHits={} cacheMisses={} shapeCaptures={} captureDefers={} captureCap={} uploads={}/{} uploadFails={} proxies={} pendingProxy={} unsupportedProxy={} unsupportedSkip={} cache(ready/pending/unsupported/total/bytes)={}/{}/{}/{}/{} jobs(reserved/queued/active/completed)={}/{}/{}/{} evictions={} staleDrops={} completedDrops={} bodyReadFails={} lineVerts={} lineLines={} lineDraws={} lineRejects={} lineMapFails={} textVerts={} textDraws={} textTrunc={} textRejectVerts={} textMapFails={} rtvHits={} rtvMisses={} limits(convex/compound/depth/queue/completed/uploads/cacheEntries/cacheBytes/bodies/lines/text)={}/{}/{}/{}/{}/{}/{}/{}/{}/{}/{}",
                    stats.bodyEntries,
                    stats.bodiesDrawn,
                    stats.bodyMeshBinds,
                    stats.bodyDrawCalls,
                    stats.bodyInstanceMaps,
                    stats.bodyInstanceRejects,
                    frame->axes.size(),
                    frame->markers.size(),
                    frame->skeleton.size(),
                    frame->text.size(),
                    stats.shapeCacheHits,
                    stats.shapeCacheMisses,
                    stats.shapeCaptures,
                    stats.shapeCaptureDeferrals,
                    frame->settings.limits.maxShapeCapturesPerFrame,
                    stats.shapeUploadsCompleted,
                    stats.shapeUploadsProcessed,
                    stats.shapeUploadFailures,
                    stats.shapeProxyFallbacks,
                    stats.shapePendingProxies,
                    stats.unsupportedShapeProxies,
                    stats.unsupportedShapeSkips,
                    pipelineStats.ready,
                    pipelineStats.pending,
                    pipelineStats.unsupported,
                    pipelineStats.entries,
                    pipelineStats.approximateGpuBytes,
                    pipelineStats.reservedJobs,
                    pipelineStats.queuedJobs,
                    pipelineStats.activeJobs,
                    pipelineStats.completedJobs,
                    pipelineStats.evictions,
                    pipelineStats.staleResultDrops,
                    pipelineStats.completedQueueDrops,
                    stats.bodyExtractFailures,
                    stats.lineVertices,
                    stats.lineLogicalLines,
                    stats.lineDrawCalls,
                    stats.lineBudgetRejects,
                    stats.lineMapFailures,
                    stats.textVertices,
                    stats.textDrawCalls,
                    stats.textVertexTruncations,
                    stats.textRejectedVertices,
                    stats.textMapFailures,
                    stats.rtvCacheHits,
                    stats.rtvCacheMisses,
                    frame->settings.limits.maxConvexSupportVertices,
                    frame->settings.limits.maxCompoundChildren,
                    frame->settings.limits.maxCompoundDepth,
                    frame->settings.limits.maxShapeQueuedJobs,
                    frame->settings.limits.maxShapeCompletedJobs,
                    frame->settings.limits.maxShapeUploadsPerFrame,
                    frame->settings.limits.maxShapeCacheEntries,
                    frame->settings.limits.maxShapeCacheBytes,
                    frame->settings.limits.maxBodyInstances,
                    frame->settings.limits.maxLineVertices,
                    frame->settings.limits.maxTextVertices);
                ROCK_LOG_DEBUG(Hand,
                    "Debug overlay timing/admission: gpuUs(latest/average)={:.2f}/{:.2f} gpu(issued/completed/pendingPolls/skipped/disjoint/failed)={}/{}/{}/{}/{}/{} admission(published/acquired/active/noPublication/duplicate/serialRace)={}/{}/{}/{}/{}/{}",
                    gpuStats.latestMicroseconds,
                    gpuStats.averageMicroseconds,
                    gpuStats.issuedSamples,
                    gpuStats.completedSamples,
                    gpuStats.pendingPolls,
                    gpuStats.skippedBegins,
                    gpuStats.disjointSamples,
                    gpuStats.failedPolls,
                    admissionStats.publishedSerial,
                    admissionStats.acquiredFrames,
                    admissionStats.activeSkips,
                    admissionStats.noPublicationSkips,
                    admissionStats.duplicateSkips,
                    admissionStats.serialRaceSkips);
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
        if (!shapePipeline().initialize()) {
            ROCK_LOG_ERROR(Hand, "Debug body overlay: shape worker initialization failed");
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
        shapePipeline().invalidate();
    }

    void ShutdownShapePipeline()
    {
        ClearFrame();
        shapePipeline().shutdown();
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
