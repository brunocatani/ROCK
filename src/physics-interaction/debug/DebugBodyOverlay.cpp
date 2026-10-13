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

#include "physics-interaction/native/HavokOffsets.h"
#include "physics-interaction/native/HavokPhysicsTiming.h"
#include "physics-interaction/native/HavokWorldLock.h"
#include "physics-interaction/debug/DebugOverlayFrameAdmission.h"
#include "physics-interaction/native/HeldScenePresentation.h"
#include "physics-interaction/debug/DebugOverlayLineBatch.h"
#include "physics-interaction/debug/DebugOverlayPolicy.h"
#include "physics-interaction/debug/DebugOverlayRuntimeSettings.h"
#include "physics-interaction/debug/DebugOverlayShapeGeometry.h"
#include "physics-interaction/debug/DebugOverlayShapePipeline.h"
#include "rock_support/ImmutableSnapshotPool.h"
#include "physics-interaction/debug/DebugOverlayStats.h"
#include "physics-interaction/PhysicsBodyFrame.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/native/PhysicsUtils.h"
#include "physics-interaction/performance/PerformanceProfiler.h"
#include "physics-interaction/visual/GripZoneIndicators.h"
#include "physics-interaction/visual/WorldDrawClient.h"
#include "physics-interaction/visual/WorldOverlayRenderer.h"
#include "RockConfig.h"
#include "RPSUIWorldDrawApi.h"

#include "RE/Havok/hknpMotion.h"
#include "RE/Havok/hknpShape.h"
#include "RE/Havok/hknpWorld.h"

#include <F4SE/F4SE.h>
#include <REL/Relocation.h>
#include <windows.h>

#include "physics-interaction/debug/DebugMath.h"

namespace rock::debug
{
    namespace
    {
        constexpr std::uintptr_t kBodyMotionPropertiesOffset = 0x72;
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
        constexpr float kRawAxisLength = 8.0f;
        constexpr float kColliderAxisLength = 12.0f;
        constexpr float kBodyAxisLength = 16.0f;
        constexpr float kTargetAxisLength = 20.0f;
        constexpr std::size_t kBodySourceCapacity = std::tuple_size_v<decltype(BodyOverlayFrame{}.entries)>;
        constexpr std::size_t kBodyInstanceCapacity = kBodySourceCapacity;
        static_assert(kBodySourceCapacity == debug_overlay_runtime::kMaxBodyInstances);
        static_assert(grip_zone_indicators::Frame::kCapacity <= rpsui::sdk::RPSUI_WORLD_DRAW_MAX_SPHERES);
        static_assert(kBodyInstanceCapacity <= rpsui::sdk::RPSUI_WORLD_DRAW_MAX_MESH_INSTANCES);
        static_assert(debug_overlay_policy::kMaxLineVertexBudget / 2 <= rpsui::sdk::RPSUI_WORLD_DRAW_MAX_LINES);
        constexpr std::uint64_t kCanonicalSphereGeometryFingerprint = 0x5350'4845'5245'0001ull;
        // Host mesh ids below the shape pipeline's range belong to the overlay.
        constexpr std::uint64_t kAabbProxyMeshId = 1;
        static_assert(kAabbProxyMeshId < debug_overlay_shape::kFirstPipelineMeshId);

        using Vertex = debug_overlay_shape::Vertex;
        using MeshData = debug_overlay_shape::MeshData;
        using ShapeKey = debug_overlay_shape::ShapeKey;
        using RegisteredShape = debug_overlay_shape::RegisteredShape;

        // Frame-callback scratch for one composed publication; capacity is kept
        // between frames.
        struct WorldDrawScratch
        {
            debug_overlay_line_batch::LineBatch lines;
            std::vector<rpsui::sdk::WorldSphereV1> spheres;
            std::vector<rpsui::sdk::WorldLineV1> worldLines;
            std::vector<rpsui::sdk::WorldTextV1> texts;
            std::vector<rpsui::sdk::WorldMeshInstanceV1> instances;
            // Keeps each drawn cache entry, and so its host mesh, alive until
            // the publication naming it has been submitted.
            std::vector<std::shared_ptr<const RegisteredShape>> heldShapes;
            std::vector<float> meshPositions;
            std::vector<std::uint32_t> meshIndices;
            bool prepared{ false };

            bool prepare()
            {
                if (prepared) {
                    return true;
                }
                try {
                    if (!lines.prepare(debug_overlay_policy::kMaxLineVertexBudget)) {
                        return false;
                    }
                    spheres.reserve(rpsui::sdk::RPSUI_WORLD_DRAW_MAX_SPHERES);
                    worldLines.reserve(rpsui::sdk::RPSUI_WORLD_DRAW_MAX_LINES);
                    texts.reserve(rpsui::sdk::RPSUI_WORLD_DRAW_MAX_TEXTS);
                    instances.reserve(kBodyInstanceCapacity);
                    heldShapes.reserve(kBodyInstanceCapacity);
                } catch (...) {
                    return false;
                }
                prepared = true;
                return true;
            }

            void clear() noexcept
            {
                spheres.clear();
                worldLines.clear();
                texts.clear();
                instances.clear();
                heldShapes.clear();
            }
        };

        WorldDrawScratch& worldDrawScratch() noexcept;

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
            bool verboseLogging{ false };
        };

        struct CapturedShapeIdentity
        {
            std::uintptr_t shapeAddress{ 0 };
            ShapeKey key{};
            float detailUniformScale{ 1.0f };
            int shapeType{ -1 };
        };

        struct PublishedBodyEntry
        {
            ShapeKey shapeKey{};
            DirectX::XMMATRIX worldMatrix = DirectX::XMMatrixIdentity();
            DirectX::XMMATRIX physicsWorldMatrix = DirectX::XMMatrixIdentity();
            DirectX::XMMATRIX childLocalMatrix =
                DirectX::XMMatrixIdentity();
            DirectX::XMFLOAT3 worldAabbMin{};
            DirectX::XMFLOAT3 worldAabbMax{};
            BodyOverlayRole role{ BodyOverlayRole::Target };
            std::uint32_t bodyId{ kInvalidBodyId };
            std::uintptr_t expectedCollisionObject{ 0 };
            std::uintptr_t expectedShape{ 0 };
            float externalColor[4]{1,1,1,1};
            float detailUniformScale{ 1.0f };
            bool hasValidWorldAabb{ false };
            bool hasChildLocalMatrix{ false };
            bool hasHeldPresentation{ false };
        };

        struct PhysicsPhaseCaptureRequestEntry
        {
            RE::hknpBodyId bodyId{ kInvalidBodyId };
            BodyOverlayRole role{ BodyOverlayRole::Target };
            bool useBodyArrayTransform{ false };
            std::uintptr_t expectedCollisionObject{ 0 };
            std::uintptr_t expectedShape{ 0 };
        };

        struct PhysicsPhaseCaptureRequest
        {
            std::array<PhysicsPhaseCaptureRequestEntry, kBodySourceCapacity> entries{};
            std::uintptr_t worldIdentity{ 0 };
            std::uint64_t gameFrameIndex{ 0 };
            std::uint32_t count{ 0 };
        };

        struct CompletedBodyPhaseEntry
        {
            DirectX::XMFLOAT4X4 postSolveWorldMatrix{};
            RE::hknpBodyId bodyId{ kInvalidBodyId };
            BodyOverlayRole role{ BodyOverlayRole::Target };
            bool postSolveValid{ false };
        };

        struct CompletedBodyPhaseFrame
        {
            std::array<CompletedBodyPhaseEntry, kBodySourceCapacity> entries{};
            std::uintptr_t worldIdentity{ 0 };
            std::uint64_t gameFrameIndex{ 0 };
            std::uint64_t solveSequence{ 0 };
            std::uint32_t substepIndex{ 0 };
            std::uint32_t substepCount{ 0 };
            std::uint32_t count{ 0 };
            std::uint32_t validCount{ 0 };
        };

        class AtomicFlagLease
        {
        public:
            explicit AtomicFlagLease(std::atomic_flag& flag) noexcept :
                _flag(flag.test_and_set(std::memory_order_acquire) ? nullptr : &flag)
            {}

            ~AtomicFlagLease()
            {
                if (_flag) {
                    _flag->clear(std::memory_order_release);
                }
            }

            AtomicFlagLease(const AtomicFlagLease&) = delete;
            AtomicFlagLease& operator=(const AtomicFlagLease&) = delete;

            [[nodiscard]] explicit operator bool() const noexcept { return _flag != nullptr; }

        private:
            std::atomic_flag* _flag{ nullptr };
        };

        struct PublishedAxisEntry
        {
            AxisOverlayEntry entry{};
            DirectX::XMMATRIX bodyWorldMatrix = DirectX::XMMatrixIdentity();
            bool hasHeldPresentation{ false };
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
            std::uint64_t gameFrameIndex{ 0 };
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

        constexpr std::size_t kPublishedFramePoolCapacity = 4;
        static immutable_snapshot::SnapshotPool<PublishedOverlayFrame, kPublishedFramePoolCapacity> s_framePool{};
        static std::atomic<std::shared_ptr<const PublishedOverlayFrame>> s_publishedFrame{};
        static std::atomic<bool> s_standardFrameEnabled{ false };
        static std::uintptr_t s_previousWorld = 0;
        static std::uint64_t s_previousShapeDecodeSettingsKey = 0;
        static std::uint32_t s_overlayStatsLogCounter = 0;
        static std::atomic_flag s_physicsPhaseCaptureGate = ATOMIC_FLAG_INIT;
        static std::atomic<bool> s_physicsPhaseCaptureEnabled{ false };
        static PhysicsPhaseCaptureRequest s_physicsPhaseCaptureRequest{};
        static CompletedBodyPhaseFrame s_completedBodyPhaseFrame{};

        /*
         * Post-solve phase-capture diagnostics. The capture chain crosses
         * three threads (game publish, physics capture, render consume) and
         * previously failed silently; these counters attribute a body drawn
         * at its pre-solve pose to the exact gate that refused. Read in a
         * rate-limited game-thread log line.
         */
        struct PhaseCaptureDiagnostics
        {
            std::atomic<std::uint32_t> storedFrames{ 0 };
            std::atomic<std::uint32_t> invalidInputs{ 0 };
            std::atomic<std::uint32_t> gateMisses{ 0 };
            std::atomic<std::uint32_t> notStaged{ 0 };
            std::atomic<std::uint32_t> worldMismatches{ 0 };
            std::atomic<std::uint32_t> frameMismatches{ 0 };
            std::atomic<std::uint32_t> extractFailures{ 0 };
            std::atomic<std::uint32_t> consumeHits{ 0 };
            std::atomic<std::uint32_t> consumeMisses{ 0 };
            std::atomic<std::uint64_t> lastRequestFrame{ 0 };
            std::atomic<std::uint64_t> lastCaptureFrame{ 0 };
        };
        static PhaseCaptureDiagnostics s_phaseCaptureDiagnostics{};

        static debug_overlay_frame_admission::FrameAdmission s_frameAdmission{};
        static std::atomic<bool> s_overlayExceptionReported{ false };
        static std::atomic<bool> s_snapshotPoolExhaustionReported{ false };
        static std::atomic<bool> s_shapeWorkerInitFailureReported{ false };
        // Frame-callback thread only.
        static bool s_scratchFailureReported = false;
        static bool s_submitFailureReported = false;
        static bool s_screenTextSkipReported = false;
        static bool s_aabbProxyRegistered = false;

        debug_overlay_shape::ShapePipeline& shapePipeline()
        {
            static debug_overlay_shape::ShapePipeline pipeline;
            return pipeline;
        }

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

        bool isFiniteTransform(const RE::NiTransform& transform)
        {
            if (!std::isfinite(transform.translate.x) ||
                !std::isfinite(transform.translate.y) ||
                !std::isfinite(transform.translate.z) ||
                !std::isfinite(transform.scale) ||
                transform.scale <= 0.0f) {
                return false;
            }
            for (std::uint32_t row = 0; row < 3; ++row) {
                for (std::uint32_t column = 0; column < 3; ++column) {
                    if (!std::isfinite(transform.rotate.entry[row][column])) {
                        return false;
                    }
                }
            }
            return true;
        }

        DirectX::XMMATRIX niStoredBodyAxesToWorldMatrix(const RE::NiTransform& transform)
        {
            /*
             * Grab BODY targets use the same stored-axis contract as hknp BODY
             * readback: each NiMatrix row is one physical local axis. Do not
             * transpose these targets through the generated-collider write
             * convention. bodyToWorldMatrix copies the corresponding native
             * BODY axis blocks into these same D3D matrix rows.
             */
            const DirectX::XMMATRIX rotation = DirectX::XMMatrixSet(
                transform.rotate.entry[0][0], transform.rotate.entry[0][1], transform.rotate.entry[0][2], 0.0f,
                transform.rotate.entry[1][0], transform.rotate.entry[1][1], transform.rotate.entry[1][2], 0.0f,
                transform.rotate.entry[2][0], transform.rotate.entry[2][1], transform.rotate.entry[2][2], 0.0f,
                0.0f, 0.0f, 0.0f, 1.0f);
            const DirectX::XMMATRIX translation = DirectX::XMMatrixTranslation(
                transform.translate.x,
                transform.translate.y,
                transform.translate.z);
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
            return role == BodyOverlayRole::Target || isExternalBodyRole(role) ?
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
            const auto* body = havok_runtime::getBody(world, bodyId);
            if (!body || !body->shape) {
                return false;
            }

            const auto bodyAddress = reinterpret_cast<std::uintptr_t>(body);
            out.bodyAddress = bodyAddress;
            out.shapeAddress = reinterpret_cast<std::uintptr_t>(body->shape);
            out.bodyId = bodyId.value;
            out.motionIndex = body->motionIndex;
            out.flags = body->flags;
            out.filterInfo = body->collisionFilterInfo;
            out.motionPropertiesId = *reinterpret_cast<std::uint16_t*>(bodyAddress + kBodyMotionPropertiesOffset);

            if (frameSource == BodyOverlayFrameSource::BodyArrayTransform || body->motionIndex == 0) {
                const auto* transform = reinterpret_cast<const float*>(bodyAddress);
                out.worldMatrix = bodyToWorldMatrix(transform);
            } else {
                const auto* motion = havok_runtime::getMotion(world, body->motionIndex);
                if (!motion) {
                    return false;
                }
                out.worldMatrix = motionToWorldMatrix(&motion->position.x, &motion->orientation.x);
            }

            return true;
        }

        bool matchesBodyWitness(RE::hknpWorld* world, RE::hknpBodyId id, std::uintptr_t collisionObject, std::uintptr_t shape)
        {
            if (!collisionObject && !shape) return true;
            const auto live = havok_runtime::snapshotBodyIdentity(world, id);
            return live.valid && reinterpret_cast<std::uintptr_t>(live.collisionObject) == collisionObject &&
                reinterpret_cast<std::uintptr_t>(live.body->shape) == shape;
        }

        // Reads the body's motion-record world-space linear velocity (Havok
        // units/s). Same guarded walk as extractBody; fails closed on any
        // missing array, freed motion, or non-finite component.
        void stagePhysicsPhaseCapture(const PublishedOverlayFrame& frame) noexcept
        {
            s_physicsPhaseCaptureEnabled.store(
                false,
                std::memory_order_release);
            AtomicFlagLease lease(s_physicsPhaseCaptureGate);
            if (!lease) {
                return;
            }

            s_physicsPhaseCaptureRequest = {};
            if (!frame.worldIdentity || frame.gameFrameIndex == 0 ||
                (!frame.drawRockBodies && !frame.drawTargetBodies && !frame.drawAxes)) {
                s_physicsPhaseCaptureEnabled.store(
                    false,
                    std::memory_order_release);
                s_completedBodyPhaseFrame = {};
                return;
            }

            if (s_completedBodyPhaseFrame.worldIdentity != 0 &&
                s_completedBodyPhaseFrame.worldIdentity != frame.worldIdentity) {
                s_completedBodyPhaseFrame = {};
            }

            s_physicsPhaseCaptureRequest.worldIdentity = frame.worldIdentity;
            s_physicsPhaseCaptureRequest.gameFrameIndex = frame.gameFrameIndex;
            for (const auto& source : frame.bodies) {
                if (source.bodyId == kInvalidBodyId) {
                    continue;
                }

                bool alreadyStaged = false;
                for (std::uint32_t index = 0;
                     index < s_physicsPhaseCaptureRequest.count;
                     ++index) {
                    const auto& staged = s_physicsPhaseCaptureRequest.entries[index];
                    if (staged.bodyId.value == source.bodyId &&
                        staged.role == source.role) {
                        alreadyStaged = true;
                        break;
                    }
                }
                if (alreadyStaged) {
                    continue;
                }

                if (s_physicsPhaseCaptureRequest.count >=
                    s_physicsPhaseCaptureRequest.entries.size()) {
                    break;
                }

                auto& destination = s_physicsPhaseCaptureRequest.entries[
                    s_physicsPhaseCaptureRequest.count++];
                destination.bodyId = RE::hknpBodyId{ source.bodyId };
                destination.role = source.role;
                destination.useBodyArrayTransform =
                    source.role == BodyOverlayRole::Target || isExternalBodyRole(source.role);
                destination.expectedCollisionObject = source.expectedCollisionObject;
                destination.expectedShape = source.expectedShape;
            }
            // Body axes can be enabled without collider shells. Capture their
            // physical frame too, sharing a request when the shell is present.
            for (const auto& axis : frame.axes) {
                if (axis.entry.source != AxisOverlaySource::Body || axis.entry.bodyId.value == kInvalidBodyId) continue;
                const auto role = axis.entry.role == AxisOverlayRole::TargetBody ? BodyOverlayRole::Target :
                    axis.entry.role == AxisOverlayRole::LeftHandBody ? BodyOverlayRole::LeftHand : BodyOverlayRole::RightHand;
                bool alreadyStaged = false;
                for (std::uint32_t index = 0; index < s_physicsPhaseCaptureRequest.count; ++index) {
                    const auto& entry = s_physicsPhaseCaptureRequest.entries[index];
                    alreadyStaged |= entry.bodyId.value == axis.entry.bodyId.value && entry.role == role;
                }
                if (alreadyStaged) continue;
                if (s_physicsPhaseCaptureRequest.count >= s_physicsPhaseCaptureRequest.entries.size()) break;
                auto& request = s_physicsPhaseCaptureRequest.entries[s_physicsPhaseCaptureRequest.count++];
                request.bodyId = axis.entry.bodyId;
                request.role = role;
                request.useBodyArrayTransform = role == BodyOverlayRole::Target;
            }
            s_physicsPhaseCaptureEnabled.store(
                s_physicsPhaseCaptureRequest.count != 0,
                std::memory_order_release);
        }

        void clearPhysicsPhaseCapture() noexcept
        {
            s_physicsPhaseCaptureEnabled.store(
                false,
                std::memory_order_release);
            AtomicFlagLease lease(s_physicsPhaseCaptureGate);
            if (!lease) {
                return;
            }
            s_physicsPhaseCaptureRequest = {};
            s_completedBodyPhaseFrame = {};
        }

        bool tryCopyCompletedBodyPhaseFrame(
            const PublishedOverlayFrame& published,
            CompletedBodyPhaseFrame& outFrame) noexcept
        {
            constexpr std::uint64_t kMaxCompletedPhaseAgeFrames = 3;
            AtomicFlagLease lease(s_physicsPhaseCaptureGate);
            if (!lease ||
                s_completedBodyPhaseFrame.worldIdentity != published.worldIdentity ||
                s_completedBodyPhaseFrame.gameFrameIndex > published.gameFrameIndex ||
                published.gameFrameIndex - s_completedBodyPhaseFrame.gameFrameIndex >
                    kMaxCompletedPhaseAgeFrames ||
                s_completedBodyPhaseFrame.count == 0) {
                s_phaseCaptureDiagnostics.consumeMisses.fetch_add(
                    1, std::memory_order_relaxed);
                return false;
            }

            s_phaseCaptureDiagnostics.consumeHits.fetch_add(
                1, std::memory_order_relaxed);
            outFrame = s_completedBodyPhaseFrame;
            return true;
        }

        const CompletedBodyPhaseEntry* findCompletedBodyPhaseEntry(
            const CompletedBodyPhaseFrame& frame,
            std::uint32_t bodyId, BodyOverlayRole role)
        {
            for (std::uint32_t index = 0;
                 index < frame.count && index < frame.entries.size();
                 ++index) {
                const auto& candidate = frame.entries[index];
                if (candidate.postSolveValid &&
                    candidate.bodyId.value == bodyId &&
                    candidate.role == role) {
                    return &candidate;
                }
            }
            return nullptr;
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

        struct CapturedCompoundChild
        {
            std::uintptr_t shapeAddress{ 0 };
            std::array<float, 16> transform{};
            std::array<float, 3> scale{};
        };

        inline constexpr std::size_t kMaxExpandedCompoundChildren = 64;

        std::int32_t captureCompoundChildSlotsUnsafe(
            const std::uintptr_t shapeAddress,
            const OverlayRenderSettings& settings,
            std::array<CapturedCompoundChild,
                kMaxExpandedCompoundChildren>& outChildren)
        {
            try {
                const auto slotArray =
                    *reinterpret_cast<const std::uintptr_t*>(
                        shapeAddress + kCompoundSlotArrayOffset);
                const auto slotCount =
                    *reinterpret_cast<const std::int32_t*>(
                        shapeAddress + kCompoundSlotCountOffset);
                if (!slotArray || slotCount <= 0 ||
                    static_cast<std::uint32_t>(slotCount) >
                        settings.limits.maxCompoundChildren ||
                    static_cast<std::size_t>(slotCount) >
                        outChildren.size()) {
                    return -1;
                }

                std::int32_t captured = 0;
                for (std::int32_t index = 0;
                     index < slotCount;
                     ++index) {
                    const auto slotOffset =
                        static_cast<std::uintptr_t>(index) *
                        kCompoundSlotStride;
                    if (slotArray >
                        (std::numeric_limits<std::uintptr_t>::max)() -
                            slotOffset) {
                        return -1;
                    }
                    const auto slot = slotArray + slotOffset;
                    if (*reinterpret_cast<const std::uint8_t*>(
                            slot + kCompoundSlotActiveOffset) != 0) {
                        continue;
                    }

                    const auto childShapeAddress =
                        *reinterpret_cast<const std::uintptr_t*>(
                            slot + kCompoundSlotShapeOffset);
                    const auto* transform =
                        reinterpret_cast<const float*>(
                            slot + kCompoundSlotTransformOffset);
                    const auto* childScale =
                        reinterpret_cast<const float*>(
                            slot + kCompoundSlotScaleOffset);
                    if (!childShapeAddress ||
                        !finiteShapeTransform(transform) ||
                        !finiteShapeVector3(childScale)) {
                        return -1;
                    }

                    auto& child = outChildren[
                        static_cast<std::size_t>(captured++)];
                    child.shapeAddress = childShapeAddress;
                    std::copy_n(
                        transform,
                        child.transform.size(),
                        child.transform.begin());
                    std::copy_n(
                        childScale,
                        child.scale.size(),
                        child.scale.begin());
                }
                return captured;
            } catch (...) {
                return -1;
            }
        }

        std::int32_t captureCompoundChildSlotsSeh(
            const std::uintptr_t shapeAddress,
            const OverlayRenderSettings* settings,
            std::array<CapturedCompoundChild,
                kMaxExpandedCompoundChildren>* outChildren)
        {
            if (!settings || !outChildren) {
                return -1;
            }
            __try {
                return captureCompoundChildSlotsUnsafe(
                    shapeAddress,
                    *settings,
                    *outChildren);
            } __except (EXCEPTION_EXECUTE_HANDLER) {
                return -1;
            }
        }

        DirectX::XMMATRIX compoundChildLocalMatrix(
            const CapturedCompoundChild& child)
        {
            return DirectX::XMMatrixMultiply(
                DirectX::XMMatrixScaling(
                    child.scale[0],
                    child.scale[1],
                    child.scale[2]),
                bodyToWorldMatrix(child.transform.data()));
        }

        bool isRockBodyRole(BodyOverlayRole role)
        {
            return role == BodyOverlayRole::RightHand || role == BodyOverlayRole::LeftHand ||
                   role == BodyOverlayRole::RightHandSegment || role == BodyOverlayRole::LeftHandSegment ||
                   role == BodyOverlayRole::BodyTorsoSegment || role == BodyOverlayRole::BodyArmSegment ||
                   role == BodyOverlayRole::BodyLegSegment || role == BodyOverlayRole::BodyFootSegment ||
                   role == BodyOverlayRole::Weapon ||
                   role == BodyOverlayRole::DynamicWeaponProxy ||
                   role == BodyOverlayRole::FocusedWeaponPart ||
                   role == BodyOverlayRole::RightGrabAuthorityProxy ||
                   role == BodyOverlayRole::LeftGrabAuthorityProxy ||
                   role == BodyOverlayRole::RightGrabPivotSourceCollider ||
                   role == BodyOverlayRole::LeftGrabPivotSourceCollider || isExternalBodyRole(role);
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
            settings.pipelineLimits.maxBytes = settings.limits.maxShapeCacheBytes;
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
            frame.gameFrameIndex = 0;
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

            const CapturedShapeIdentity captured{
                shapeAddress,
                key,
                detailUniformScale,
                shapeType,
            };
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

        bool readPrimitiveDimensions(std::uintptr_t shapeAddress, int& type, float& radius, float& length)
        {
            // Same primitive interpretation as captureShapeRecipeUnsafe; no
            // authoring guesses, scene bounds or heap allocation for labels.
            __try {
                const auto* shape = reinterpret_cast<const RE::hknpShape*>(shapeAddress);
                type = static_cast<int>(shape->GetType());
                radius = shape->convexRadius * havokToGameScale();
                if (!std::isfinite(radius) || radius < 0.0f) return false;
                if (type == 3) {
                    const auto* a = reinterpret_cast<const float*>(shapeAddress + 0x50);
                    const auto* b = reinterpret_cast<const float*>(shapeAddress + 0x60);
                    if (!finiteShapeVector3(a) || !finiteShapeVector3(b)) return false;
                    const float x = b[0] - a[0], y = b[1] - a[1], z = b[2] - a[2];
                    length = std::sqrt(x*x + y*y + z*z) * havokToGameScale() + 2.0f * radius;
                    return std::isfinite(length);
                }
                return true;
            } __except (EXCEPTION_EXECUTE_HANDLER) {
                return false;
            }
        }

        void appendExternalShapeLabel(const BodyOverlayFrame& source, const BodyOverlayEntry& entry,
            const BodyRenderInfo& body, PublishedOverlayFrame& destination)
        {
            if (!source.drawText || !isExternalBodyRole(entry.role) || !entry.label[0]) return;
            TextOverlayEntry text{};
            std::copy_n(entry.color, 4, text.color);
            text.color[3] = 1.0f;
            text.worldAnchored = true;
            DirectX::XMFLOAT4X4 matrix{};
            DirectX::XMStoreFloat4x4(&matrix, body.worldMatrix);
            text.worldAnchor = {matrix._41, matrix._42, matrix._43};
            text.x = 12.0f;
            text.y = entry.labelOffsetY;
            text.size = 1.0f;
            int type = -1;
            float radius = 0.0f, length = 0.0f;
            if (!readPrimitiveDimensions(body.shapeAddress, type, radius, length)) {
                std::snprintf(text.text, sizeof(text.text), "%s | shape unreadable", entry.label);
            } else if (type == 2) {
                std::snprintf(text.text, sizeof(text.text), "%s | sphere R %.2f", entry.label, radius);
            } else if (type == 3) {
                std::snprintf(text.text, sizeof(text.text), "%s | capsule R %.2f L %.2f", entry.label, radius, length);
            } else {
                DirectX::XMFLOAT3 low{}, high{};
                if ((entry.visualFlags & static_cast<std::uint32_t>(api::native_interaction::VisualFlag::InactiveSource)) == 0 && captureBodyWorldAabb(source.world, entry.bodyId, low, high)) {
                    std::snprintf(text.text, sizeof(text.text), "%s | type %d AABB %.1fx%.1fx%.1f", entry.label, type,
                        high.x - low.x, high.y - low.y, high.z - low.z);
                } else {
                    std::snprintf(text.text, sizeof(text.text), "%s | type %d convex R %.2f (no active AABB)", entry.label, type, radius);
                }
            }
            destination.text.push_back(text);
        }

        bool buildPublishedFrame(const BodyOverlayFrame& source, PublishedOverlayFrame& destination)
        {
            resetPublishedFrame(destination);
            destination.settings = captureOverlayRenderSettings();
            destination.worldIdentity = reinterpret_cast<std::uintptr_t>(source.world);
            destination.gameFrameIndex = source.gameFrameIndex;
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
            if (source.drawText) {
                const auto count = (std::min)(source.textCount, static_cast<std::uint32_t>(source.textEntries.size()));
                destination.text.reserve(count + bodyCount);
                destination.text.assign(source.textEntries.begin(), source.textEntries.begin() + count);
            }
            destination.bodies.reserve(source.entries.size());
            destination.capturedShapeIdentities.reserve(source.entries.size());
            for (std::uint32_t index = 0; index < bodyCount; ++index) {
                const auto& entry = source.entries[index];
                const bool rockRole = isRockBodyRole(entry.role);
                if ((rockRole && !source.drawRockBodies) || (!rockRole && !source.drawTargetBodies)) {
                    continue;
                }

                BodyRenderInfo body{};
                if (!matchesBodyWitness(source.world, entry.bodyId, entry.expectedCollisionObject, entry.expectedShape) ||
                    !extractBody(source.world, entry.bodyId, targetBodyOverlayFrameSource(entry.role), body)) {
                    ++destination.bodyExtractFailures;
                    continue;
                }
                appendExternalShapeLabel(source, entry, body, destination);
                const bool canReadWorldAabb = (entry.visualFlags & static_cast<std::uint32_t>(api::native_interaction::VisualFlag::InactiveSource)) == 0;

                // Held-object presentation advances the complete solved pose.
                // Use that same immutable frame for the ordinary collider shell,
                // while phase diagnostics retain the unmodified physics sample.
                RE::NiTransform presentedBody{};
                const bool hasPresentation = !isExternalBodyRole(entry.role) && held_scene_presentation::tryGetPresentedBodyWorld(
                    source.world, entry.bodyId.value, source.gameFrameIndex, presentedBody);
                const auto displayedWorldMatrix = hasPresentation ?
                    niStoredBodyAxesToWorldMatrix(presentedBody) : body.worldMatrix;

                const auto shapeIdentity = captureShapeIdentityForFrame(destination, body.shapeAddress);
                if (rockRole &&
                    (shapeIdentity.shapeType == 7 ||
                        shapeIdentity.shapeType == 8)) {
                    std::array<CapturedCompoundChild,
                        kMaxExpandedCompoundChildren> children{};
                    const auto childCount = captureCompoundChildSlotsSeh(
                        body.shapeAddress,
                        &destination.settings,
                        &children);
                    if (childCount > 0) {
                        DirectX::XMFLOAT3 aabbMin{};
                        DirectX::XMFLOAT3 aabbMax{};
                        const bool aabbValid = canReadWorldAabb && captureBodyWorldAabb(
                            source.world,
                            entry.bodyId,
                            aabbMin,
                            aabbMax);
                        for (std::int32_t childIndex = 0;
                             childIndex < childCount;
                             ++childIndex) {
                            const auto& child = children[
                                static_cast<std::size_t>(childIndex)];
                            const auto childIdentity =
                                captureShapeIdentityForFrame(
                                    destination,
                                    child.shapeAddress);
                            PublishedBodyEntry published{};
                            published.shapeKey = childIdentity.key;
                            published.worldMatrix = displayedWorldMatrix;
                            published.physicsWorldMatrix = body.worldMatrix;
                            published.hasHeldPresentation = hasPresentation;
                            published.childLocalMatrix =
                                compoundChildLocalMatrix(child);
                            published.hasChildLocalMatrix = true;
                            published.role = entry.role;
                            // Keep the array-index id, not the +0x6C
                            // self-reported id: the post-solve phase capture
                            // feeds this value back into extractBody, whose
                            // index guard rejects the serial bits.
                            published.bodyId = entry.bodyId.value;
                            published.expectedCollisionObject = entry.expectedCollisionObject;
                            published.expectedShape = entry.expectedShape;
                            std::copy_n(entry.color,4,published.externalColor);
                            published.detailUniformScale =
                                childIdentity.detailUniformScale;
                            published.hasValidWorldAabb = aabbValid;
                            published.worldAabbMin = aabbMin;
                            published.worldAabbMax = aabbMax;
                            requestShapeBuildForFrame(
                                destination,
                                published.shapeKey,
                                child.shapeAddress);
                            destination.bodies.push_back(
                                std::move(published));
                        }
                        continue;
                    }
                }

                PublishedBodyEntry published{};
                published.shapeKey = shapeIdentity.key;
                published.worldMatrix = displayedWorldMatrix;
                published.physicsWorldMatrix = body.worldMatrix;
                published.hasHeldPresentation = hasPresentation;
                published.role = entry.role;
                // Keep the array-index id, not the +0x6C self-reported id:
                // the post-solve phase capture feeds this value back into
                // extractBody, whose index guard rejects the serial bits.
                published.bodyId = entry.bodyId.value;
                published.expectedCollisionObject = entry.expectedCollisionObject;
                published.expectedShape = entry.expectedShape;
                std::copy_n(entry.color,4,published.externalColor);
                published.detailUniformScale = shapeIdentity.detailUniformScale;
                published.hasValidWorldAabb =
                    canReadWorldAabb && captureBodyWorldAabb(source.world, entry.bodyId, published.worldAabbMin, published.worldAabbMax);
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
                        RE::NiTransform presentedBody{};
                        if (held_scene_presentation::tryGetPresentedBodyWorld(source.world,
                                published.entry.bodyId.value, source.gameFrameIndex, presentedBody)) {
                            published.bodyWorldMatrix = niStoredBodyAxesToWorldMatrix(presentedBody);
                            published.hasHeldPresentation = true;
                        }
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
            return !destination.bodies.empty() || !destination.axes.empty() || !destination.markers.empty() ||
                   !destination.skeleton.empty() || !destination.coloredLines.empty() || !destination.text.empty();
        }

        void bodyColor(
            BodyOverlayRole role,
            debug_overlay_policy::ShapeDecodeMode decodeMode,
            float color[4])
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
            case BodyOverlayRole::DynamicWeaponProxy:
                color[0] = 1.0f;
                color[1] = 0.24f;
                color[2] = 0.08f;
                color[3] = 0.92f;
                break;
            case BodyOverlayRole::FocusedWeaponPart:
                color[0] = 1.0f;
                color[1] = 0.72f;
                color[2] = 0.05f;
                color[3] = 0.95f;
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
            case BodyOverlayRole::External: break;
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
            case AxisOverlayRole::RightGrabAuthorityProxyAppliedTarget:
            case AxisOverlayRole::LeftGrabAuthorityProxyAppliedTarget:
                return 9.0f;
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
            case AxisOverlayRole::RightGrabAuthorityProxyAppliedTarget:
            case AxisOverlayRole::LeftGrabAuthorityProxyAppliedTarget:
                return 0.98f;
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
            case MarkerOverlayRole::LeftGrabAuthorityProxyTarget:
                color[0] = 1.0f;
                color[1] = 0.88f;
                color[2] = 0.05f;
                color[3] = 0.98f;
                break;
            case MarkerOverlayRole::RightGrabAuthorityProxyAppliedTarget:
            case MarkerOverlayRole::LeftGrabAuthorityProxyAppliedTarget:
                color[0] = 1.0f;
                color[1] = 0.42f;
                color[2] = 0.02f;
                color[3] = 1.0f;
                break;
            case MarkerOverlayRole::RightGrabAuthorityProxyClockDelta:
            case MarkerOverlayRole::LeftGrabAuthorityProxyClockDelta:
                color[0] = 1.0f;
                color[1] = 1.0f;
                color[2] = 1.0f;
                color[3] = 0.96f;
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
            case MarkerOverlayRole::GunstockBaseline:
                color[0] = 0.10f;
                color[1] = 0.95f;
                color[2] = 1.0f;
                break;
            case MarkerOverlayRole::LeftWeaponSupportGrip:
                color[0] = 1.0f;
                color[1] = 0.40f;
                color[2] = 0.85f;
                break;
            case MarkerOverlayRole::GunstockWeapon:
                color[0] = 0.2f;
                color[1] = 1.0f;
                color[2] = 0.2f;
                break;
            case MarkerOverlayRole::GunstockSeat:
                color[0] = 1.0f;
                color[1] = 0.8f;
                color[2] = 0.1f;
                break;
            case MarkerOverlayRole::GunstockAim:
                color[0] = 0.9f;
                color[1] = 0.25f;
                color[2] = 1.0f;
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
            case MarkerOverlayRole::AuthoredGripActivationSupportSideAxis:
                color[0] = 0.05f;
                color[1] = 0.9f;
                color[2] = 1.0f;
                color[3] = 0.98f;
                break;
            case MarkerOverlayRole::AuthoredGripActivationDownAxis:
                color[0] = 1.0f;
                color[1] = 0.15f;
                color[2] = 0.85f;
                color[3] = 0.98f;
                break;
            case MarkerOverlayRole::AuthoredGripActivationReferenceAxis:
                color[0] = 0.68f;
                color[1] = 0.68f;
                color[2] = 0.72f;
                color[3] = 0.9f;
                break;
            case MarkerOverlayRole::AuthoredGripActivationAllowedRegion:
            case MarkerOverlayRole::AuthoredGripActivationPass:
                color[0] = 0.25f;
                color[1] = 1.0f;
                color[2] = 0.12f;
                color[3] = 0.98f;
                break;
            case MarkerOverlayRole::AuthoredGripActivationFail:
                color[0] = 1.0f;
                color[1] = 0.12f;
                color[2] = 0.05f;
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

        bool finiteValues(const float* values, std::size_t count) noexcept
        {
            for (std::size_t index = 0; index < count; ++index) {
                if (!std::isfinite(values[index])) {
                    return false;
                }
            }
            return true;
        }

        // RPS UI rejects a whole publication for one non-finite record, so each
        // record is checked here and skipped alone.
        void emitLineBatch(const debug_overlay_line_batch::LineBatch& batch, WorldDrawScratch& out, OverlayRuntimeStats& stats)
        {
            stats.lineVertices += static_cast<std::uint32_t>(batch.vertexCount());
            stats.lineLogicalLines += static_cast<std::uint32_t>(batch.lineCount());
            stats.lineBudgetRejects += static_cast<std::uint32_t>(batch.rejectedLineCount());
            for (const auto& segment : batch.segments()) {
                rpsui::sdk::WorldLineV1 line{};
                line.start[0] = segment.start.x;
                line.start[1] = segment.start.y;
                line.start[2] = segment.start.z;
                line.end[0] = segment.end.x;
                line.end[1] = segment.end.y;
                line.end[2] = segment.end.z;
                line.color[0] = segment.color.r;
                line.color[1] = segment.color.g;
                line.color[2] = segment.color.b;
                line.color[3] = segment.color.a;
                if (!finiteValues(line.start, 3) || !finiteValues(line.end, 3) || !finiteValues(line.color, 4)) {
                    ++stats.lineInvalidRejects;
                    continue;
                }
                if (out.worldLines.size() >= rpsui::sdk::RPSUI_WORLD_DRAW_MAX_LINES) {
                    ++stats.lineBudgetRejects;
                    continue;
                }
                out.worldLines.push_back(line);
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

        void collectAxisOverlays(debug_overlay_line_batch::LineBatch& batch, const PublishedOverlayFrame& frame,
            const CompletedBodyPhaseFrame* completedFrame)
        {
            if (!frame.drawAxes || frame.axes.empty()) {
                return;
            }

            for (const auto& published : frame.axes) {
                if (published.entry.source == AxisOverlaySource::Body) {
                    auto displayed = published;
                    if (!published.hasHeldPresentation && completedFrame && completedFrame->gameFrameIndex == frame.gameFrameIndex) {
                        const auto role = published.entry.role == AxisOverlayRole::TargetBody ? BodyOverlayRole::Target :
                            published.entry.role == AxisOverlayRole::LeftHandBody ? BodyOverlayRole::LeftHand : BodyOverlayRole::RightHand;
                        if (const auto* completed = findCompletedBodyPhaseEntry(*completedFrame, published.entry.bodyId.value, role)) {
                            displayed.bodyWorldMatrix = DirectX::XMLoadFloat4x4(&completed->postSolveWorldMatrix);
                        }
                    }
                    collectBodyAxisEntry(batch, displayed);
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

        void collectColoredLineOverlays(debug_overlay_line_batch::LineBatch& batch, const PublishedOverlayFrame& frame)
        {
            if (!frame.drawColoredLines || frame.coloredLines.empty()) {
                return;
            }

            for (const auto& entry : frame.coloredLines) {
                appendDebugLine(batch, toVertex(entry.start), toVertex(entry.end), entry.color);
            }
        }

        // Text is drawn by RPS UI next to its world anchor in both eyes.
        // Screen-positioned text has no VR placement and is not drawn.
        void appendTexts(const PublishedOverlayFrame& frame, WorldDrawScratch& out, OverlayRuntimeStats& stats)
        {
            if (!frame.drawText) {
                return;
            }
            for (const auto& entry : frame.text) {
                if (!entry.worldAnchored) {
                    ++stats.screenTextSkips;
                    continue;
                }
                rpsui::sdk::WorldTextV1 text{};
                static_assert(sizeof(text.text) == sizeof(entry.text));
                std::memcpy(text.text, entry.text, sizeof(text.text));
                text.text[sizeof(text.text) - 1] = '\0';
                text.anchor[0] = entry.worldAnchor.x;
                text.anchor[1] = entry.worldAnchor.y;
                text.anchor[2] = entry.worldAnchor.z;
                text.offsetPixels[0] = entry.x;
                text.offsetPixels[1] = entry.y;
                text.pixelSize = (std::max)(1.0f, entry.size);
                std::copy_n(entry.color, 4, text.color);
                if (!finiteValues(text.anchor, 3) || !finiteValues(text.offsetPixels, 2) || !finiteValues(text.color, 4)) {
                    ++stats.textInvalidRejects;
                    continue;
                }
                if (out.texts.size() >= rpsui::sdk::RPSUI_WORLD_DRAW_MAX_TEXTS) {
                    ++stats.textTruncations;
                    continue;
                }
                out.texts.push_back(text);
            }
        }

        // Gameplay markers: solid white unit spheres scaled to the marker size.
        void appendGripZoneIndicators(const grip_zone_indicators::Frame& frame, WorldDrawScratch& out)
        {
            const std::uint32_t count = (std::min)(
                frame.count,
                static_cast<std::uint32_t>(frame.positions.size()));
            if (count == 0 || !std::isfinite(frame.diameterGameUnits) || frame.diameterGameUnits <= 0.0f) {
                return;
            }
            const float radius = frame.diameterGameUnits * 0.5f;
            for (std::uint32_t index = 0; index < count; ++index) {
                const auto& position = frame.positions[index];
                rpsui::sdk::WorldSphereV1 sphere{};
                sphere.center[0] = position.x;
                sphere.center[1] = position.y;
                sphere.center[2] = position.z;
                sphere.radius = radius;
                std::fill_n(sphere.color, 4, 1.0f);
                sphere.fill = rpsui::sdk::WorldFillV1::Solid;
                if (finiteValues(sphere.center, 3)) {
                    out.spheres.push_back(sphere);
                }
            }
        }

        // Frame-callback thread. Copies one finished mesh into RPS UI.
        bool registerMeshData(std::uint64_t meshId, const MeshData& mesh) noexcept
        {
            auto& scratch = worldDrawScratch();
            try {
                scratch.meshPositions.clear();
                scratch.meshPositions.reserve(mesh.vertices.size() * 3);
                for (const auto& vertex : mesh.vertices) {
                    scratch.meshPositions.insert(scratch.meshPositions.end(), { vertex.x, vertex.y, vertex.z });
                }
                scratch.meshIndices.assign(mesh.indices.begin(), mesh.indices.end());
            } catch (...) {
                return false;
            }
            return world_draw_client::registerMesh(meshId, scratch.meshPositions.data(),
                static_cast<std::uint32_t>(mesh.vertices.size()), scratch.meshIndices.data(),
                static_cast<std::uint32_t>(scratch.meshIndices.size()));
        }

        // Any thread: the shape cache drops entries on the game thread too.
        void releaseMeshData(std::uint64_t meshId) noexcept
        {
            world_draw_client::releaseMesh(meshId);
        }

        constexpr debug_overlay_shape::MeshSink kMeshSink{ &registerMeshData, &releaseMeshData };

        void ensureAabbProxyRegistered()
        {
            if (s_aabbProxyRegistered) {
                return;
            }
            static const MeshData box = debug_overlay_shape::makeUnitBoxMesh();
            s_aabbProxyRegistered = registerMeshData(kAabbProxyMeshId, box);
        }

        void appendBodies(
            const PublishedOverlayFrame& frame,
            const CompletedBodyPhaseFrame* completedPhaseFrame,
            WorldDrawScratch& out,
            OverlayRuntimeStats& stats)
        {
            const std::size_t baseEntryLimit = frame.settings.limits.maxBodyInstances;
            const std::size_t renderInstanceLimit = (std::min)(
                kBodyInstanceCapacity,
                baseEntryLimit);
            std::size_t admittedBaseEntries = 0;
            for (const auto& entry : frame.bodies) {
                ++stats.bodyEntries;
                if (admittedBaseEntries >= baseEntryLimit) {
                    ++stats.bodyInstanceRejects;
                    continue;
                }
                ++admittedBaseEntries;

                const auto cached = shapePipeline().lookup(entry.shapeKey);
                const RegisteredShape* shape = nullptr;
                bool shapeReady = false;
                if (cached.state == debug_overlay_shape::CacheState::Ready && cached.shape && cached.shape->indexCount > 0) {
                    shape = cached.shape.get();
                    shapeReady = true;
                    ++stats.shapeCacheHits;
                    if (shape->decodeMode == debug_overlay_policy::ShapeDecodeMode::Proxy) {
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
                    if (!s_aabbProxyRegistered) {
                        continue;
                    }
                    ++stats.shapeProxyFallbacks;
                    if (cached.state == debug_overlay_shape::CacheState::Unsupported) {
                        ++stats.unsupportedShapeProxies;
                    } else {
                        ++stats.shapePendingProxies;
                    }
                }

                const auto appendInstance = [&](DirectX::XMMATRIX model) {
                    if (out.instances.size() >= renderInstanceLimit) {
                        ++stats.bodyInstanceRejects;
                        return;
                    }

                    if (shapeReady) {
                        if (entry.hasChildLocalMatrix) {
                            model = DirectX::XMMatrixMultiply(
                                entry.childLocalMatrix,
                                model);
                        }
                        if (entry.detailUniformScale != 1.0f) {
                            model = DirectX::XMMatrixScaling(
                                        entry.detailUniformScale,
                                        entry.detailUniformScale,
                                        entry.detailUniformScale) *
                                model;
                        }
                    } else {
                        // The cached world AABB belongs to the physics sample.
                        // Carry its proxy by the same pose delta as the detailed
                        // shape, including while that shape is still building.
                        DirectX::XMVECTOR determinant{};
                        const auto inversePhysics = DirectX::XMMatrixInverse(&determinant, entry.physicsWorldMatrix);
                        const float determinantValue = DirectX::XMVectorGetX(determinant);
                        if (!std::isfinite(determinantValue) || std::abs(determinantValue) < 0.000001f) return;
                        model = worldAabbMatrix(entry) * inversePhysics * model;
                    }

                    rpsui::sdk::WorldMeshInstanceV1 instance{};
                    instance.meshId = shapeReady ? shape->meshId : kAabbProxyMeshId;
                    DirectX::XMFLOAT4X4 stored{};
                    DirectX::XMStoreFloat4x4(&stored, model);
                    static_assert(sizeof(stored) == sizeof(instance.transform));
                    std::memcpy(instance.transform, &stored, sizeof(instance.transform));
                    const auto drawnMode = shapeReady ? shape->decodeMode : debug_overlay_policy::ShapeDecodeMode::Proxy;
                    bodyColor(
                        entry.role,
                        isExternalBodyRole(entry.role) && !shapeReady ?
                            (cached.state == debug_overlay_shape::CacheState::Unsupported ?
                                debug_overlay_policy::ShapeDecodeMode::Unsupported : debug_overlay_policy::ShapeDecodeMode::Proxy) :
                            drawnMode,
                        instance.color);
                    if (isExternalBodyRole(entry.role) && shapeReady && drawnMode == debug_overlay_policy::ShapeDecodeMode::Detailed)
                        std::copy_n(entry.externalColor, 4, instance.color);
                    instance.fill = rpsui::sdk::WorldFillV1::Wire;
                    if (!finiteValues(instance.transform, 16) || !finiteValues(instance.color, 4)) {
                        ++stats.bodyInvalidRejects;
                        return;
                    }
                    if (shapeReady) {
                        out.heldShapes.push_back(cached.shape);
                    }
                    out.instances.push_back(instance);
                    ++stats.bodiesDrawn;
                };

                auto displayedWorld = entry.worldMatrix;
                if (!entry.hasHeldPresentation && completedPhaseFrame && completedPhaseFrame->gameFrameIndex == frame.gameFrameIndex) {
                    if (const auto* completed = findCompletedBodyPhaseEntry(*completedPhaseFrame, entry.bodyId, entry.role)) {
                        displayedWorld = DirectX::XMLoadFloat4x4(&completed->postSolveWorldMatrix);
                    }
                }
                appendInstance(displayedWorld);
            }
        }

        void composeWorldDrawFrameUnsafe()
        {
            performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::DebugOverlayRender);

            auto frame = s_publishedFrame.load(std::memory_order_acquire);
            // Invalid diagnostic worlds must not suppress gameplay markers.
            if (frame && !frame->worldIdentity) frame.reset();
            const auto gripZoneIndicatorFrame = grip_zone_indicators::Snapshot();
            const bool hasGripZoneIndicatorsToDraw =
                gripZoneIndicatorFrame && gripZoneIndicatorFrame->count > 0;
            if (!frame && !hasGripZoneIndicatorsToDraw) {
                return;
            }

            // Frame-callback thread only; too large for the host's render stack.
            static CompletedBodyPhaseFrame completedPhaseFrame{};
            const bool hasCompletedPhaseFrame =
                frame &&
                tryCopyCompletedBodyPhaseFrame(
                    *frame,
                    completedPhaseFrame);

            const bool hasBodiesToDraw = frame &&
                (frame->drawRockBodies || frame->drawTargetBodies) &&
                !frame->bodies.empty();
            const bool hasAxesToDraw =
                frame && frame->drawAxes && !frame->axes.empty();
            const bool hasMarkersToDraw =
                frame && frame->drawMarkers && !frame->markers.empty();
            const bool hasSkeletonToDraw =
                frame && frame->drawSkeleton && !frame->skeleton.empty();
            const bool hasColoredLinesToDraw = frame &&
                frame->drawColoredLines && !frame->coloredLines.empty();
            const bool hasTextToDraw =
                frame && frame->drawText && !frame->text.empty();
            const bool hasStandardOverlayToDraw =
                hasBodiesToDraw || hasAxesToDraw || hasMarkersToDraw ||
                hasSkeletonToDraw || hasColoredLinesToDraw || hasTextToDraw;
            if (!hasStandardOverlayToDraw && !hasGripZoneIndicatorsToDraw) {
                return;
            }

            auto& scratch = worldDrawScratch();
            if (!scratch.prepare()) {
                if (!s_scratchFailureReported) {
                    s_scratchFailureReported = true;
                    ROCK_LOG_WARN(Hand, "Debug overlay: world-draw scratch allocation failed; drawing skipped");
                }
                return;
            }
            scratch.clear();

            OverlayRuntimeStats stats{};
            if (frame) {
                stats.bodyExtractFailures = frame->bodyExtractFailures;
                stats.shapeCaptures = frame->shapeCaptures;
                stats.shapeCaptureDeferrals = frame->shapeCaptureDeferrals;
                const auto uploadResult =
                    shapePipeline().processCompletedUploads(
                        kMeshSink,
                        frame->settings.limits.maxShapeUploadsPerFrame,
                        frame->settings.pipelineLimits);
                stats.shapeUploadsProcessed = uploadResult.processed;
                stats.shapeUploadsCompleted = uploadResult.uploaded;
                stats.shapeUploadFailures = uploadResult.failed;
            }

            if (hasBodiesToDraw) {
                ensureAabbProxyRegistered();
                appendBodies(
                    *frame,
                    hasCompletedPhaseFrame ?
                        &completedPhaseFrame :
                        nullptr,
                    scratch,
                    stats);
            }

            if (hasGripZoneIndicatorsToDraw) {
                appendGripZoneIndicators(*gripZoneIndicatorFrame, scratch);
            }

            if (frame) {
                auto& lineBatch = scratch.lines;
                lineBatch.beginFrame(
                    frame->settings.limits.maxLineVertices);
                // Owner-published diagnostics are already hard-bounded by
                // the provider API. Admit them first so an enabled addon
                // view is not silently starved by unrelated
                // high-cardinality probes.
                collectColoredLineOverlays(lineBatch, *frame);
                collectAxisOverlays(lineBatch, *frame, hasCompletedPhaseFrame ? &completedPhaseFrame : nullptr);
                collectMarkerOverlays(lineBatch, *frame);
                collectSkeletonOverlays(lineBatch, *frame);
                emitLineBatch(lineBatch, scratch, stats);
                appendTexts(*frame, scratch, stats);
            }

            // One frame: the next drawn frame either receives a fresh
            // composition or shows nothing, as when ROCK drew directly.
            rpsui::sdk::WorldDrawPublicationV1 publication{};
            publication.leaseFrames = 1;
            publication.sphereCount = static_cast<std::uint32_t>(scratch.spheres.size());
            publication.lineCount = static_cast<std::uint32_t>(scratch.worldLines.size());
            publication.textCount = static_cast<std::uint32_t>(scratch.texts.size());
            publication.meshInstanceCount = static_cast<std::uint32_t>(scratch.instances.size());
            publication.spheres = scratch.spheres.data();
            publication.lines = scratch.worldLines.data();
            publication.texts = scratch.texts.data();
            publication.meshInstances = scratch.instances.data();
            const bool submitted = world_draw_client::submit(publication);
            scratch.heldShapes.clear();
            if (!submitted) {
                if (!s_submitFailureReported) {
                    s_submitFailureReported = true;
                    ROCK_LOG_WARN(Hand, "Debug overlay: RPS UI refused the world-draw publication; frame not drawn");
                }
            } else {
                s_submitFailureReported = false;
            }
            if (stats.screenTextSkips > 0 && !s_screenTextSkipReported) {
                s_screenTextSkipReported = true;
                ROCK_LOG_WARN(Hand, "Debug overlay: text without a world anchor has no VR placement and is not drawn");
            }

            if (frame && frame->settings.verboseLogging &&
                ++s_overlayStatsLogCounter >= 90) {
                s_overlayStatsLogCounter = 0;
                const auto pipelineStats = shapePipeline().stats();
                const auto admissionStats = s_frameAdmission.stats();
                ROCK_LOG_DEBUG(Hand,
                    "Debug overlay frame: entries={} drawn={} bodyRejects={} bodyInvalid={} axes={} markers={} skeleton={} text={}/{} textTrunc={} textInvalid={} screenTextSkips={} cacheHits={} cacheMisses={} shapeCaptures={} captureDefers={} captureCap={} registrations={}/{} registrationFails={} proxies={} pendingProxy={} unsupportedProxy={} unsupportedSkip={} cache(ready/pending/unsupported/total/bytes)={}/{}/{}/{}/{} jobs(reserved/queued/active/completed)={}/{}/{}/{} evictions={} staleDrops={} completedDrops={} bodyReadFails={} lineVerts={} lineLines={} lineRejects={} lineInvalid={} limits(convex/compound/depth/queue/completed/uploads/cacheEntries/cacheBytes/bodies/lines)={}/{}/{}/{}/{}/{}/{}/{}/{}/{}",
                    stats.bodyEntries,
                    stats.bodiesDrawn,
                    stats.bodyInstanceRejects,
                    stats.bodyInvalidRejects,
                    frame->axes.size(),
                    frame->markers.size(),
                    frame->skeleton.size(),
                    scratch.texts.size(),
                    frame->text.size(),
                    stats.textTruncations,
                    stats.textInvalidRejects,
                    stats.screenTextSkips,
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
                    pipelineStats.approximateBytes,
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
                    stats.lineBudgetRejects,
                    stats.lineInvalidRejects,
                    frame->settings.limits.maxConvexSupportVertices,
                    frame->settings.limits.maxCompoundChildren,
                    frame->settings.limits.maxCompoundDepth,
                    frame->settings.limits.maxShapeQueuedJobs,
                    frame->settings.limits.maxShapeCompletedJobs,
                    frame->settings.limits.maxShapeUploadsPerFrame,
                    frame->settings.limits.maxShapeCacheEntries,
                    frame->settings.limits.maxShapeCacheBytes,
                    frame->settings.limits.maxBodyInstances,
                    frame->settings.limits.maxLineVertices);
                ROCK_LOG_DEBUG(Hand,
                    "Debug overlay admission: published={} acquired={} active={} noPublication={} duplicate={} serialRace={}",
                    admissionStats.publishedSerial,
                    admissionStats.acquiredFrames,
                    admissionStats.activeSkips,
                    admissionStats.noPublicationSkips,
                    admissionStats.duplicateSkips,
                    admissionStats.serialRaceSkips);
                ROCK_LOG_DEBUG(Hand,
                    "Debug overlay phase: completed={} source={} display={} age={} solve={} substep={}/{} bodies={}/{}",
                    hasCompletedPhaseFrame ? "yes" : "no",
                    hasCompletedPhaseFrame ?
                        completedPhaseFrame.gameFrameIndex :
                        0,
                    frame->gameFrameIndex,
                    hasCompletedPhaseFrame ?
                        frame->gameFrameIndex -
                            completedPhaseFrame.gameFrameIndex :
                        0,
                    hasCompletedPhaseFrame ?
                        completedPhaseFrame.solveSequence :
                        0,
                    hasCompletedPhaseFrame ?
                        completedPhaseFrame.substepIndex + 1 :
                        0,
                    hasCompletedPhaseFrame ?
                        completedPhaseFrame.substepCount :
                        0,
                    hasCompletedPhaseFrame ?
                        completedPhaseFrame.validCount :
                        0,
                    hasCompletedPhaseFrame ?
                        completedPhaseFrame.count :
                        0);
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

        // RPS UI frame callback (render thread). Draws at most once per ROCK
        // publication, like the former direct renderer: a frame with no fresh
        // publication shows nothing rather than repeating stale poses.
        void composeWorldDrawFrame() noexcept
        {
            if (!s_standardFrameEnabled.load(std::memory_order_acquire) && !grip_zone_indicators::Snapshot()) {
                return;
            }
            auto admission = s_frameAdmission.tryAcquire();
            if (!admission) {
                return;
            }
            try {
                composeWorldDrawFrameUnsafe();
            } catch (const std::exception& exception) {
                reportOverlayExceptionOnce(exception.what());
            } catch (...) {
                reportOverlayExceptionOnce("non-standard exception");
            }
        }

        WorldDrawScratch& worldDrawScratch() noexcept
        {
            static auto* scratch = new WorldDrawScratch();
            return *scratch;
        }
    }

    bool EnsureWorldOverlayInstalled()
    {
        return world_draw_client::ensureBound(&composeWorldDrawFrame);
    }

    void Install()
    {
        (void)world_overlay_renderer::EnsureInstalled();
    }

    void PublishFrame(const BodyOverlayFrame& frame)
    {
        auto next = s_framePool.acquire();
        if (!next) {
            if (!s_snapshotPoolExhaustionReported.exchange(true, std::memory_order_relaxed)) {
                ROCK_LOG_WARN(Hand, "Debug body overlay: immutable snapshot pool exhausted; retaining the last safe publication");
            }
            (void)s_frameAdmission.publish();
            return;
        }

        bool enabled = false;
        {
            // All native shape data is copied into recipes before this lock
            // ends. The worker and renderer retain no borrowed engine pointers.
            havok_world_lock::ScopedWorldReadLock readLock(frame.world);
            enabled = buildPublishedFrame(frame, *next);
        }
        if (enabled && (!next->bodies.empty() || !next->axes.empty())) {
            stagePhysicsPhaseCapture(*next);

            // Publish-thread-only counter; ~7 s cadence at 90 Hz. Leaves
            // log evidence of which phase-capture gate refused, so a body
            // drawn without its post-solve pose is attributable from the
            // session log.
            static std::uint32_t s_phaseDiagPublishCounter = 0;
            if (++s_phaseDiagPublishCounter >= 600) {
                s_phaseDiagPublishCounter = 0;
                const auto& diag = s_phaseCaptureDiagnostics;
                ROCK_LOG_INFO(
                    Hand,
                    "PHASE_CAPTURE store={} invalid={} gate={} notStaged={} worldMis={} frameMis={} extractFail={} consume={}/{} reqFrame={} capFrame={}",
                    diag.storedFrames.load(std::memory_order_relaxed),
                    diag.invalidInputs.load(std::memory_order_relaxed),
                    diag.gateMisses.load(std::memory_order_relaxed),
                    diag.notStaged.load(std::memory_order_relaxed),
                    diag.worldMismatches.load(std::memory_order_relaxed),
                    diag.frameMismatches.load(std::memory_order_relaxed),
                    diag.extractFailures.load(std::memory_order_relaxed),
                    diag.consumeHits.load(std::memory_order_relaxed),
                    diag.consumeMisses.load(std::memory_order_relaxed),
                    diag.lastRequestFrame.load(std::memory_order_relaxed),
                    diag.lastCaptureFrame.load(std::memory_order_relaxed));
            }
        } else {
            clearPhysicsPhaseCapture();
        }
        std::shared_ptr<const PublishedOverlayFrame> immutable = std::move(next);
        s_publishedFrame.store(std::move(immutable), std::memory_order_release);
        s_snapshotPoolExhaustionReported.store(false, std::memory_order_relaxed);
        s_standardFrameEnabled.store(enabled, std::memory_order_release);
        (void)s_frameAdmission.publish();
    }

    void CapturePostSolveBodyPhases(
        RE::hknpWorld* world,
        const havok_physics_timing::PhysicsTimingSample& timing,
        const std::uint64_t gameFrameIndex,
        const std::uint64_t solveSequence) noexcept
    {
        if (!s_physicsPhaseCaptureEnabled.load(
                std::memory_order_acquire)) {
            return;
        }
        if (!world || gameFrameIndex == 0 || solveSequence == 0 ||
            !timing.valid) {
            s_phaseCaptureDiagnostics.invalidInputs.fetch_add(
                1, std::memory_order_relaxed);
            return;
        }

        const std::uint32_t substepCount = (std::max)(timing.substepCount, 1u);
        const bool finalSubstep =
            timing.substepIndex + 1 >= substepCount ||
            timing.substepProgress >= 0.999f;
        if (!finalSubstep) {
            return;
        }

        AtomicFlagLease lease(s_physicsPhaseCaptureGate);
        if (!lease) {
            s_phaseCaptureDiagnostics.gateMisses.fetch_add(
                1, std::memory_order_relaxed);
            return;
        }
        s_phaseCaptureDiagnostics.lastRequestFrame.store(
            s_physicsPhaseCaptureRequest.gameFrameIndex,
            std::memory_order_relaxed);
        s_phaseCaptureDiagnostics.lastCaptureFrame.store(
            gameFrameIndex, std::memory_order_relaxed);
        if (s_physicsPhaseCaptureRequest.count == 0) {
            s_phaseCaptureDiagnostics.notStaged.fetch_add(
                1, std::memory_order_relaxed);
            return;
        }
        if (s_physicsPhaseCaptureRequest.worldIdentity !=
            reinterpret_cast<std::uintptr_t>(world)) {
            s_phaseCaptureDiagnostics.worldMismatches.fetch_add(
                1, std::memory_order_relaxed);
            return;
        }
        if (s_physicsPhaseCaptureRequest.gameFrameIndex != gameFrameIndex) {
            s_phaseCaptureDiagnostics.frameMismatches.fetch_add(
                1, std::memory_order_relaxed);
            return;
        }

        CompletedBodyPhaseFrame captured{};
        captured.worldIdentity = reinterpret_cast<std::uintptr_t>(world);
        captured.gameFrameIndex = gameFrameIndex;
        captured.solveSequence = solveSequence;
        captured.substepIndex = timing.substepIndex;
        captured.substepCount = substepCount;
        for (std::uint32_t index = 0;
             index < s_physicsPhaseCaptureRequest.count &&
             index < s_physicsPhaseCaptureRequest.entries.size();
             ++index) {
            const auto& request = s_physicsPhaseCaptureRequest.entries[index];
            auto& destination = captured.entries[captured.count++];
            destination.bodyId = request.bodyId;
            destination.role = request.role;

            BodyRenderInfo body{};
            if (!matchesBodyWitness(world, request.bodyId, request.expectedCollisionObject, request.expectedShape) || !extractBody(
                    world,
                    request.bodyId,
                    request.useBodyArrayTransform ?
                        BodyOverlayFrameSource::BodyArrayTransform :
                        BodyOverlayFrameSource::LiveMotionWhenAvailable,
                    body)) {
                s_phaseCaptureDiagnostics.extractFailures.fetch_add(
                    1, std::memory_order_relaxed);
                continue;
            }

            DirectX::XMStoreFloat4x4(
                &destination.postSolveWorldMatrix,
                body.worldMatrix);
            destination.postSolveValid = true;
            ++captured.validCount;
        }

        s_completedBodyPhaseFrame = captured;
        s_phaseCaptureDiagnostics.storedFrames.fetch_add(
            1, std::memory_order_relaxed);
    }

    void ClearFrame()
    {
        clearPhysicsPhaseCapture();
        s_publishedFrame.store({}, std::memory_order_release);
        s_standardFrameEnabled.store(false, std::memory_order_release);
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

}

namespace rock::world_overlay_renderer
{
    bool EnsureInstalled()
    {
        return debug::EnsureWorldOverlayInstalled();
    }

    void NotifyPublication() noexcept
    {
        (void)debug::s_frameAdmission.publish();
    }
}
