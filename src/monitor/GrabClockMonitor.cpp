#include "monitor/GrabClockMonitor.h"

#include "monitor/GrabClockMonitorConfig.h"
#include "monitor/PrismaUI_F4_API.h"
#include "monitor/PrismaUI_F4VR_API.h"

#include "api/ROCKProviderApi.h"
#include "physics-interaction/debug/GrabClockDebugFeed.h"
#include "rock_support/Fo4VrRuntime.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <limits>

namespace rock::monitor
{
    namespace
    {
        using ::rock::provider::RockProviderFrameSnapshot;
        using ::rock::provider::RockProviderLifecycleFlag;

        constexpr MonitorSettings kDefaultMonitorSettings{};
        constexpr std::uint32_t kPanelWidthPixels = 1400;
        constexpr std::uint32_t kPanelHeightPixels = 1120;
        // Same physical footprint as ROCK Prober so the shared [PanelPose]
        // numbers place both panels identically.
        constexpr float kPanelPhysicalWidthGameUnits = 24.0f;
        constexpr float kPanelPhysicalHeightGameUnits =
            kPanelPhysicalWidthGameUnits * static_cast<float>(kPanelHeightPixels) /
            static_cast<float>(kPanelWidthPixels);
        constexpr std::uint32_t kF4VrApiFlavor = 0x52563446u;
        constexpr std::uint32_t kMinimumSpatialRevision = 1;
        // Display-only panel: the pointer/central-routing features required by
        // the interactive Prober are intentionally absent here.
        constexpr std::uint64_t kRequiredSpatialFeatureBits =
            PRISMA_UI_VR_API::SpatialFeature_FullPose |
            PRISMA_UI_VR_API::SpatialFeature_IndependentDimensions |
            PRISMA_UI_VR_API::SpatialFeature_LatestOnlyUpdates |
            PRISMA_UI_VR_API::SpatialFeature_AppliedSequenceQuery |
            PRISMA_UI_VR_API::SpatialFeature_GpuRendering |
            PRISMA_UI_VR_API::SpatialFeature_NativeNetworkPolicy |
            PRISMA_UI_VR_API::SpatialFeature_SceneDepthOcclusion;
        constexpr auto kSpatialFailureLogInterval = std::chrono::seconds(5);
        constexpr auto kHeadLockedResetRetryInterval = std::chrono::seconds(1);

        struct PanelPose
        {
            RE::NiPoint3 position{};
            std::array<float, 4> orientation{ 0.0f, 0.0f, 0.0f, 1.0f };
        };

        struct Vec3Sample
        {
            RE::NiPoint3 value{};
            bool valid{ false };
        };

        struct RoomSample
        {
            RE::NiPoint3 position{};
            float yawDegrees{ 0.0f };
            bool valid{ false };
        };

        struct MonitorFrameData
        {
            bool providerReady{ false };
            bool configuredEnabled{ true };
            std::uint64_t frameIndex{ 0 };
            float deltaSeconds{ 0.0f };
            std::uint32_t menuBlocking{ 0 };
            std::uint32_t configBlocking{ 0 };
            std::optional<PanelPose> panelPose{};
            RoomSample room{};
            Vec3Sample handRight{};
            Vec3Sample handLeft{};
            Vec3Sample hmd{};
            Vec3Sample weapon{};
            ::rock::debug::RockGrabClockDebugHandV1 grabRight{};
            ::rock::debug::RockGrabClockDebugHandV1 grabLeft{};
        };

        PRISMA_UI_API::IVPrismaUI4* s_prisma = nullptr;
        PRISMA_UI_VR_API::IVPrismaUIVR1* s_prismaVR = nullptr;
        PRISMA_UI_VR_API::SpatialCapabilitiesV1 s_spatialCapabilities{};
        std::atomic_bool s_spatialCapabilitiesReady = false;
        PrismaView s_view = 0;
        std::atomic_bool s_domReady = false;
        std::atomic_bool s_viewRequested = false;
        std::unique_ptr<MonitorConfig> s_config;
        std::atomic_bool s_initialized = false;

        std::atomic_int s_lastConfiguredEnabledStatus{ -1 };
        std::atomic_bool s_lastProviderReadyLogged{ false };
        std::atomic_int s_lastPanelPoseStatus{ -1 };

        std::mutex s_stateMutex;
        MonitorFrameData s_latestFrameData{};

        std::atomic_bool s_pushScheduled{ false };
        std::atomic_bool s_panelVisible{ false };
        std::atomic_bool s_worldPresentationActive{ false };
        std::atomic_uint64_t s_nextSpatialSequence{ 1 };
        std::atomic_bool s_sequenceExhaustionLogged{ false };
        std::chrono::steady_clock::time_point s_nextHeadLockedResetAttempt{};

        struct SpatialResultLogState
        {
            bool failing{ false };
            PRISMA_UI_VR_API::SpatialResult lastResult{ PRISMA_UI_VR_API::SpatialResult::Ok };
            std::chrono::steady_clock::time_point nextLog{};
        };

        SpatialResultLogState s_worldSubmitLogState{};
        SpatialResultLogState s_headLockedSubmitLogState{};

        struct SpatialApplyHealth
        {
            bool ready{ false };
            bool hasObservedReady{ false };
            bool failureLogged{ false };
            std::uint64_t lastAppliedSequence{ 0 };
            std::chrono::steady_clock::time_point failureSince{};
            std::chrono::steady_clock::time_point nextLog{};
        };

        std::mutex s_spatialApplyHealthMutex;
        SpatialApplyHealth s_spatialApplyHealth{};

        void pushLatestSnapshot();

        [[nodiscard]] bool isFinitePoint(const RE::NiPoint3& point)
        {
            return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
        }

        [[nodiscard]] std::optional<RE::NiPoint3> normalize(const RE::NiPoint3& value)
        {
            if (!isFinitePoint(value)) {
                return std::nullopt;
            }
            const float length = value.Length();
            if (!std::isfinite(length) || length < 0.0001f) {
                return std::nullopt;
            }
            return value / length;
        }

        [[nodiscard]] float dot(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs)
        {
            return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
        }

        [[nodiscard]] RE::NiPoint3 cross(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs)
        {
            return {
                lhs.y * rhs.z - lhs.z * rhs.y,
                lhs.z * rhs.x - lhs.x * rhs.z,
                lhs.x * rhs.y - lhs.y * rhs.x,
            };
        }

        [[nodiscard]] std::optional<std::array<float, 4>> normalizedQuaternion(std::array<float, 4> quaternion)
        {
            float normSquared = 0.0f;
            for (const float component : quaternion) {
                if (!std::isfinite(component)) {
                    return std::nullopt;
                }
                normSquared += component * component;
            }
            if (!std::isfinite(normSquared) || normSquared < 0.000001f) {
                return std::nullopt;
            }
            const float inverseNorm = 1.0f / std::sqrt(normSquared);
            for (float& component : quaternion) {
                component *= inverseNorm;
            }
            return quaternion;
        }

        [[nodiscard]] std::optional<std::array<float, 4>> quaternionFromBasis(
            const RE::NiPoint3& right, const RE::NiPoint3& up, const RE::NiPoint3& front)
        {
            // Conventional column-vector rotation matrix. Its columns are the world-space
            // images of Prisma's panel-local +X/+Y/+Z axes.
            const float m00 = right.x;
            const float m01 = up.x;
            const float m02 = front.x;
            const float m10 = right.y;
            const float m11 = up.y;
            const float m12 = front.y;
            const float m20 = right.z;
            const float m21 = up.z;
            const float m22 = front.z;

            std::array<float, 4> quaternion{};
            const float trace = m00 + m11 + m22;
            if (trace > 0.0f) {
                const float scale = std::sqrt(trace + 1.0f) * 2.0f;
                if (!std::isfinite(scale) || scale < 0.0001f) {
                    return std::nullopt;
                }
                quaternion[3] = 0.25f * scale;
                quaternion[0] = (m21 - m12) / scale;
                quaternion[1] = (m02 - m20) / scale;
                quaternion[2] = (m10 - m01) / scale;
            } else if (m00 > m11 && m00 > m22) {
                const float scale = std::sqrt(std::max(0.0f, 1.0f + m00 - m11 - m22)) * 2.0f;
                if (!std::isfinite(scale) || scale < 0.0001f) {
                    return std::nullopt;
                }
                quaternion[3] = (m21 - m12) / scale;
                quaternion[0] = 0.25f * scale;
                quaternion[1] = (m01 + m10) / scale;
                quaternion[2] = (m02 + m20) / scale;
            } else if (m11 > m22) {
                const float scale = std::sqrt(std::max(0.0f, 1.0f + m11 - m00 - m22)) * 2.0f;
                if (!std::isfinite(scale) || scale < 0.0001f) {
                    return std::nullopt;
                }
                quaternion[3] = (m02 - m20) / scale;
                quaternion[0] = (m01 + m10) / scale;
                quaternion[1] = 0.25f * scale;
                quaternion[2] = (m12 + m21) / scale;
            } else {
                const float scale = std::sqrt(std::max(0.0f, 1.0f + m22 - m00 - m11)) * 2.0f;
                if (!std::isfinite(scale) || scale < 0.0001f) {
                    return std::nullopt;
                }
                quaternion[3] = (m10 - m01) / scale;
                quaternion[0] = (m02 + m20) / scale;
                quaternion[1] = (m12 + m21) / scale;
                quaternion[2] = 0.25f * scale;
            }
            return normalizedQuaternion(quaternion);
        }

        [[nodiscard]] std::optional<std::array<float, 4>> multiplyQuaternions(
            const std::array<float, 4>& left,
            const std::array<float, 4>& right)
        {
            return normalizedQuaternion({
                left[3] * right[0] + left[0] * right[3] + left[1] * right[2] - left[2] * right[1],
                left[3] * right[1] - left[0] * right[2] + left[1] * right[3] + left[2] * right[0],
                left[3] * right[2] + left[0] * right[1] - left[1] * right[0] + left[2] * right[3],
                left[3] * right[3] - left[0] * right[0] - left[1] * right[1] - left[2] * right[2],
            });
        }

        [[nodiscard]] bool snapshotAllowsDisplay(const RockProviderFrameSnapshot& snapshot)
        {
            return snapshot.providerReady != 0 &&
                   snapshot.menuBlocking == 0 &&
                   snapshot.configBlocking == 0 &&
                   ::rock::provider::hasLifecycleFlag(snapshot.lifecycleFlags, RockProviderLifecycleFlag::WorldAvailable) &&
                   ::rock::provider::hasLifecycleFlag(snapshot.lifecycleFlags, RockProviderLifecycleFlag::SkeletonReady) &&
                   ::rock::provider::hasLifecycleFlag(snapshot.lifecycleFlags, RockProviderLifecycleFlag::ProviderReady);
        }

        [[nodiscard]] std::optional<PanelPose> computeRightHandPanelPose(
            const RockProviderFrameSnapshot& snapshot,
            const MonitorSettings& settings)
        {
            const auto& transform = snapshot.rightHandTransform;
            const RE::NiPoint3 origin{ transform.translate[0], transform.translate[1], transform.translate[2] };
            if (!isFinitePoint(origin)) {
                return std::nullopt;
            }

            const RE::NiPoint3 rawForward{ transform.rotate[0], transform.rotate[1], transform.rotate[2] };
            const RE::NiPoint3 rawLateral{ transform.rotate[3], transform.rotate[4], transform.rotate[5] };
            const RE::NiPoint3 rawUp{ transform.rotate[6], transform.rotate[7], transform.rotate[8] };
            const auto forward = normalize(rawForward);
            if (!forward) {
                return std::nullopt;
            }

            // Gram-Schmidt keeps pose stable if an engine transform contains small
            // numerical drift. Prefer the hand's local +Z as panel up; reconstruct it
            // from local +Y only if +Z is degenerate.
            auto up = normalize(rawUp - (*forward * dot(rawUp, *forward)));
            if (!up) {
                const auto lateral = normalize(rawLateral - (*forward * dot(rawLateral, *forward)));
                if (!lateral) {
                    return std::nullopt;
                }
                up = normalize(cross(*forward, *lateral));
                if (!up) {
                    return std::nullopt;
                }
            }

            const auto handLateral = normalize(cross(*up, *forward));
            if (!handLateral) {
                return std::nullopt;
            }
            up = normalize(cross(*forward, *handLateral));
            if (!up) {
                return std::nullopt;
            }

            // The panel sits in front of the hand and faces back toward the wearer.
            const RE::NiPoint3 panelFront = *forward * -1.0f;
            const RE::NiPoint3 panelRight = *handLateral * -1.0f;

            const auto baseOrientation = quaternionFromBasis(panelRight, *up, panelFront);
            if (!baseOrientation) {
                return std::nullopt;
            }
            const auto orientation = multiplyQuaternions(*baseOrientation, settings.localOrientation);
            if (!orientation) {
                return std::nullopt;
            }

            PanelPose result{};
            result.position = origin +
                              (*forward * settings.positionX) +
                              (*handLateral * settings.positionY) +
                              (*up * settings.positionZ);
            result.orientation = *orientation;
            return isFinitePoint(result.position) ? std::optional<PanelPose>{ result } : std::nullopt;
        }

        [[nodiscard]] const char* spatialResultName(PRISMA_UI_VR_API::SpatialResult result)
        {
            using R = PRISMA_UI_VR_API::SpatialResult;
            switch (result) {
            case R::Ok:
                return "Ok";
            case R::PendingUpdateReplaced:
                return "PendingUpdateReplaced";
            case R::InvalidView:
                return "InvalidView";
            case R::InvalidArgument:
                return "InvalidArgument";
            case R::InvalidStructSize:
                return "InvalidStructSize";
            case R::NotReady:
                return "NotReady";
            case R::Unsupported:
                return "Unsupported";
            case R::StaleSequence:
                return "StaleSequence";
            case R::ShuttingDown:
                return "ShuttingDown";
            case R::InternalError:
                return "InternalError";
            case R::ResourceLimit:
                return "ResourceLimit";
            }
            return "Unknown";
        }

        [[nodiscard]] bool spatialResultAccepted(PRISMA_UI_VR_API::SpatialResult result)
        {
            return result == PRISMA_UI_VR_API::SpatialResult::Ok ||
                   result == PRISMA_UI_VR_API::SpatialResult::PendingUpdateReplaced;
        }

        void reportSpatialResult(
            SpatialResultLogState& state, const char* operation, PRISMA_UI_VR_API::SpatialResult result)
        {
            if (spatialResultAccepted(result)) {
                if (state.failing) {
                    logger::info("ROCK Monitor: {} recovered ({}).", operation, spatialResultName(result));
                }
                state = {};
                return;
            }

            const auto now = std::chrono::steady_clock::now();
            if (!state.failing || state.lastResult != result || now >= state.nextLog) {
                logger::warn("ROCK Monitor: {} failed with {} ({}); panel remains hidden.",
                    operation, spatialResultName(result), static_cast<std::int32_t>(result));
                state.nextLog = now + kSpatialFailureLogInterval;
            }
            state.failing = true;
            state.lastResult = result;
        }

        [[nodiscard]] std::optional<std::uint64_t> nextSpatialSequence()
        {
            auto current = s_nextSpatialSequence.load(std::memory_order_relaxed);
            while (current != (std::numeric_limits<std::uint64_t>::max)()) {
                if (s_nextSpatialSequence.compare_exchange_weak(
                        current, current + 1, std::memory_order_relaxed, std::memory_order_relaxed)) {
                    return current;
                }
            }

            if (!s_sequenceExhaustionLogged.exchange(true, std::memory_order_relaxed)) {
                logger::critical("ROCK Monitor: Prisma spatial sequence space exhausted; spatial updates disabled.");
            }
            return std::nullopt;
        }

        void resetSpatialApplyHealth()
        {
            std::scoped_lock lock(s_spatialApplyHealthMutex);
            s_spatialApplyHealth = {};
        }

        void observeSpatialApplyHealth()
        {
            if (!s_prismaVR || !s_view ||
                !s_worldPresentationActive.load(std::memory_order_acquire)) {
                resetSpatialApplyHealth();
                return;
            }

            std::scoped_lock lock(s_spatialApplyHealthMutex);
            PRISMA_UI_VR_API::SpatialStateV1 state{};
            state.structSize = sizeof(state);
            const auto queryResult = s_prismaVR->GetSpatialState(s_view, &state);
            const auto requiredAppliedFlags =
                PRISMA_UI_VR_API::SpatialState_Applied |
                PRISMA_UI_VR_API::SpatialState_SceneDepthOcclusion;
            const bool hasDepthApplied =
                queryResult == PRISMA_UI_VR_API::SpatialResult::Ok &&
                (state.stateFlags & requiredAppliedFlags) == requiredAppliedFlags &&
                state.appliedSequence > 0;
            const bool appliedProgress = hasDepthApplied &&
                state.appliedSequence > s_spatialApplyHealth.lastAppliedSequence;
            const bool fullyCaughtUp = hasDepthApplied &&
                (state.stateFlags & PRISMA_UI_VR_API::SpatialState_BackendReady) != 0 &&
                (state.stateFlags & PRISMA_UI_VR_API::SpatialState_Pending) == 0 &&
                state.lastApplyResult ==
                    static_cast<std::int32_t>(PRISMA_UI_VR_API::SpatialResult::Ok) &&
                state.appliedSequence == state.acceptedSequence;
            const auto now = std::chrono::steady_clock::now();

            if (appliedProgress || fullyCaughtUp) {
                s_spatialApplyHealth.lastAppliedSequence = std::max(
                    s_spatialApplyHealth.lastAppliedSequence,
                    state.appliedSequence);
                if (!s_spatialApplyHealth.hasObservedReady) {
                    logger::info(
                        "ROCK Monitor: scene-depth WorldQuad produced an applied frame at sequence {}.",
                        state.appliedSequence);
                } else if (!s_spatialApplyHealth.ready &&
                           s_spatialApplyHealth.failureLogged) {
                    logger::info(
                        "ROCK Monitor: scene-depth WorldQuad application recovered at sequence {}.",
                        state.appliedSequence);
                }
                s_spatialApplyHealth.ready = true;
                s_spatialApplyHealth.hasObservedReady = true;
                s_spatialApplyHealth.failureLogged = false;
                s_spatialApplyHealth.failureSince = {};
                s_spatialApplyHealth.nextLog = {};
                return;
            }

            if (s_spatialApplyHealth.failureSince ==
                std::chrono::steady_clock::time_point{}) {
                s_spatialApplyHealth.failureSince = now;
                s_spatialApplyHealth.nextLog = now + kSpatialFailureLogInterval;
            }
            s_spatialApplyHealth.ready = false;
            if (now < s_spatialApplyHealth.nextLog) {
                return;
            }
            logger::warn(
                "ROCK Monitor: scene-depth WorldQuad has not made applied-frame progress "
                "(query={}, accepted={}, applied={}, flags=0x{:08X}, lastApply={}).",
                spatialResultName(queryResult), state.acceptedSequence,
                state.appliedSequence, state.stateFlags, state.lastApplyResult);
            s_spatialApplyHealth.failureLogged = true;
            s_spatialApplyHealth.nextLog = now + kSpatialFailureLogInterval;
        }

        [[nodiscard]] bool validateSpatialCapabilities(const PRISMA_UI_VR_API::SpatialCapabilitiesV1& capabilities)
        {
            const std::uint64_t gameWorldMask =
                1ull << static_cast<std::uint32_t>(PRISMA_UI_VR_API::SpatialCoordinateSpace::GameWorld);
            const std::uint64_t worldQuadMask =
                1ull << static_cast<std::uint32_t>(PRISMA_UI_VR_API::SpatialPresentationMode::WorldQuad);
            const std::uint64_t requiredPixels =
                static_cast<std::uint64_t>(kPanelWidthPixels) * static_cast<std::uint64_t>(kPanelHeightPixels);

            const bool compatible =
                capabilities.structSize >= sizeof(PRISMA_UI_VR_API::SpatialCapabilitiesV1) &&
                capabilities.apiFlavor == kF4VrApiFlavor &&
                capabilities.spatialRevision >= kMinimumSpatialRevision &&
                (capabilities.featureBits & kRequiredSpatialFeatureBits) == kRequiredSpatialFeatureBits &&
                (capabilities.supportedUpdateFlags &
                 PRISMA_UI_VR_API::SpatialUpdate_SceneDepthOcclusion) != 0 &&
                (capabilities.coordinateSpaceMask & gameWorldMask) != 0 &&
                (capabilities.presentationModeMask & worldQuadMask) != 0 &&
                capabilities.maxPixelWidth >= kPanelWidthPixels &&
                capabilities.maxPixelHeight >= kPanelHeightPixels &&
                capabilities.maxSpatialViews > 0 &&
                capabilities.maxAggregateSpatialPixels >= requiredPixels &&
                std::isfinite(capabilities.maxAbsoluteWorldPosition) &&
                capabilities.maxAbsoluteWorldPosition > 0.0f &&
                std::isfinite(capabilities.maxPhysicalDimension) &&
                capabilities.maxPhysicalDimension >= kPanelPhysicalWidthGameUnits &&
                std::isfinite(capabilities.minQuaternionNormSquared) &&
                capabilities.minQuaternionNormSquared > 0.0f &&
                capabilities.minQuaternionNormSquared <= 1.0f;

            if (!compatible) {
                logger::error(
                    "ROCK Monitor: incompatible Prisma FO4VR extension capabilities "
                    "(size={}, flavor=0x{:08X}, revision={}, features=0x{:016X}, spaces=0x{:016X}, "
                    "modes=0x{:016X}, updateFlags=0x{:08X}, maxPixels={}x{}, views={}, aggregate={}, worldBound={}, "
                    "physicalBound={}, quaternionMin={}).",
                    capabilities.structSize, capabilities.apiFlavor, capabilities.spatialRevision,
                    capabilities.featureBits, capabilities.coordinateSpaceMask, capabilities.presentationModeMask,
                    capabilities.supportedUpdateFlags,
                    capabilities.maxPixelWidth, capabilities.maxPixelHeight, capabilities.maxSpatialViews,
                    capabilities.maxAggregateSpatialPixels, capabilities.maxAbsoluteWorldPosition,
                    capabilities.maxPhysicalDimension, capabilities.minQuaternionNormSquared);
            }
            return compatible;
        }

        [[nodiscard]] bool acquireSpatialCapabilities()
        {
            if (s_spatialCapabilitiesReady.load(std::memory_order_acquire)) {
                return true;
            }
            if (!s_prismaVR) {
                return false;
            }

            PRISMA_UI_VR_API::SpatialCapabilitiesV1 capabilities{};
            capabilities.structSize = sizeof(capabilities);
            const auto result = s_prismaVR->GetSpatialCapabilities(&capabilities);
            if (result != PRISMA_UI_VR_API::SpatialResult::Ok) {
                logger::error("ROCK Monitor: Prisma FO4VR capability query failed with {} ({}).",
                    spatialResultName(result), static_cast<std::int32_t>(result));
                return false;
            }
            if (!validateSpatialCapabilities(capabilities)) {
                return false;
            }

            s_spatialCapabilities = capabilities;
            s_spatialCapabilitiesReady.store(true, std::memory_order_release);
            logger::info(
                "ROCK Monitor: Prisma FO4VR GPU WorldQuad + scene-depth contract accepted "
                "(revision {}, max {}x{}, {} Hz, aggregate {} pixels).",
                capabilities.spatialRevision, capabilities.maxPixelWidth, capabilities.maxPixelHeight,
                capabilities.maxRefreshRateHz, capabilities.maxAggregateSpatialPixels);
            return true;
        }

        [[nodiscard]] bool panelPoseWithinCapabilities(const PanelPose& pose)
        {
            if (!s_spatialCapabilitiesReady.load(std::memory_order_acquire)) {
                return false;
            }
            const auto positionBound = s_spatialCapabilities.maxAbsoluteWorldPosition;
            if (!isFinitePoint(pose.position) || std::fabs(pose.position.x) > positionBound ||
                std::fabs(pose.position.y) > positionBound || std::fabs(pose.position.z) > positionBound) {
                return false;
            }

            float normSquared = 0.0f;
            for (const float component : pose.orientation) {
                if (!std::isfinite(component)) {
                    return false;
                }
                normSquared += component * component;
            }
            return std::isfinite(normSquared) && normSquared >= s_spatialCapabilities.minQuaternionNormSquared;
        }

        [[nodiscard]] std::optional<PRISMA_UI_VR_API::SpatialUpdateV1> makeSpatialUpdate(
            PRISMA_UI_VR_API::SpatialPresentationMode mode, const PanelPose* pose)
        {
            const auto sequence = nextSpatialSequence();
            if (!sequence) {
                return std::nullopt;
            }

            PRISMA_UI_VR_API::SpatialUpdateV1 update{};
            update.structSize = sizeof(update);
            update.coordinateSpace = PRISMA_UI_VR_API::SpatialCoordinateSpace::GameWorld;
            update.presentationMode = mode;
            if (mode == PRISMA_UI_VR_API::SpatialPresentationMode::WorldBillboard ||
                mode == PRISMA_UI_VR_API::SpatialPresentationMode::WorldQuad) {
                update.flags = PRISMA_UI_VR_API::SpatialUpdate_SceneDepthOcclusion;
            }
            update.sequence = *sequence;
            update.dimensions.pixelWidth = kPanelWidthPixels;
            update.dimensions.pixelHeight = kPanelHeightPixels;
            if (pose) {
                update.pose.position[0] = pose->position.x;
                update.pose.position[1] = pose->position.y;
                update.pose.position[2] = pose->position.z;
                std::copy(pose->orientation.begin(), pose->orientation.end(), update.pose.orientation);
                update.dimensions.physicalWidth = kPanelPhysicalWidthGameUnits;
                update.dimensions.physicalHeight = kPanelPhysicalHeightGameUnits;
            } else {
                update.pose.orientation[3] = 1.0f;
            }
            return update;
        }

        void hidePanelAndResetSpatialState()
        {
            if (!s_prisma || !s_view) {
                return;
            }

            if (s_panelVisible.exchange(false, std::memory_order_acq_rel)) {
                s_prisma->Hide(s_view);
            }
            if (!s_worldPresentationActive.load(std::memory_order_acquire)) {
                return;
            }

            const auto now = std::chrono::steady_clock::now();
            if (now < s_nextHeadLockedResetAttempt) {
                return;
            }
            s_nextHeadLockedResetAttempt = now + kHeadLockedResetRetryInterval;

            const auto update = makeSpatialUpdate(PRISMA_UI_VR_API::SpatialPresentationMode::HeadLockedQuad, nullptr);
            if (!update) {
                return;
            }
            const auto result = s_prismaVR->SubmitSpatialUpdate(s_view, &*update);
            reportSpatialResult(s_headLockedSubmitLogState, "head-locked reset", result);
            if (spatialResultAccepted(result)) {
                s_worldPresentationActive.store(false, std::memory_order_release);
                resetSpatialApplyHealth();
            }
        }

        void schedulePush()
        {
            bool expected = false;
            if (!s_pushScheduled.compare_exchange_strong(expected, true, std::memory_order_acq_rel)) {
                return;
            }

            const auto* tasks = F4SE::GetTaskInterface();
            if (!tasks) {
                s_pushScheduled.store(false, std::memory_order_release);
                return;
            }

            tasks->AddTask([]() { pushLatestSnapshot(); });
        }

        [[nodiscard]] double roundTo(const float value, const double scale)
        {
            if (!std::isfinite(value)) {
                return 0.0;
            }
            return std::round(static_cast<double>(value) * scale) / scale;
        }

        [[nodiscard]] nlohmann::json vec3Json(const float (&value)[3])
        {
            return nlohmann::json::array({ roundTo(value[0], 100.0), roundTo(value[1], 100.0), roundTo(value[2], 100.0) });
        }

        [[nodiscard]] nlohmann::json vec3Json(const RE::NiPoint3& value)
        {
            return nlohmann::json::array({ roundTo(value.x, 100.0), roundTo(value.y, 100.0), roundTo(value.z, 100.0) });
        }

        [[nodiscard]] nlohmann::json grabFeedJson(const ::rock::debug::RockGrabClockDebugHandV1& feed)
        {
            const auto& phy = feed.physics;
            const auto& pro = feed.producer;
            const auto& pre = feed.preFrik;
            return {
                { "phy", {
                    { "n", phy.writeCount },
                    { "room", vec3Json(phy.roomPos) },
                    { "yaw", roundTo(phy.roomYawDegrees, 1000.0) },
                    { "rv", phy.roomValid },
                    { "held", vec3Json(phy.heldNodePos) },
                    { "hv", phy.heldNodeValid },
                    { "dW", roundTo(phy.heldVsLastWriteGu, 1000.0) },
                } },
                { "pro", {
                    { "n", pro.writeCount },
                    { "seq", pro.schedulerSequence },
                    { "dt", roundTo(pro.deltaSeconds, 100000.0) },
                    { "room", vec3Json(pro.roomPos) },
                    { "yaw", roundTo(pro.roomYawDegrees, 1000.0) },
                    { "rv", pro.roomValid },
                    { "raw", vec3Json(pro.rawHandPos) },
                    { "entry", vec3Json(pro.heldEntryPos) },
                    { "ev", pro.heldEntryValid },
                    { "eLW", roundTo(pro.entryVsLastWriteGu, 1000.0) },
                    { "eB", roundTo(pro.entryVsBodyGu, 1000.0) },
                    { "body", vec3Json(pro.bodyPos) },
                    { "bv", pro.bodyValid },
                    { "anch", vec3Json(pro.anchorPos) },
                    { "blend", roundTo(pro.bodyBlend, 1000.0) },
                    { "eng", pro.anchorEngaged },
                    { "nw", pro.nodeWriteApplied },
                } },
                { "pre", {
                    { "n", pre.writeCount },
                    { "seq", pre.schedulerSequence },
                    { "room", vec3Json(pre.roomPos) },
                    { "yaw", roundTo(pre.roomYawDegrees, 1000.0) },
                    { "rv", pre.roomValid },
                    { "raw", vec3Json(pre.rawHandPos) },
                    { "rhv", pre.rawHandValid },
                    { "rP", roundTo(pre.rawVsProducerGu, 1000.0) },
                    { "roomP", roundTo(pre.roomVsProducerGu, 1000.0) },
                    { "held", vec3Json(pre.heldNodePos) },
                    { "dW", roundTo(pre.heldVsLastWriteGu, 1000.0) },
                    { "rep", vec3Json(pre.republishedHandPos) },
                } },
            };
        }

        [[nodiscard]] nlohmann::json buildModelJson(const MonitorFrameData& data)
        {
            nlohmann::json json{
                { "f", data.frameIndex },
                { "dt", roundTo(data.deltaSeconds, 100000.0) },
                { "ready", data.providerReady ? 1 : 0 },
                { "menu", data.menuBlocking },
                { "cfg", data.configBlocking },
            };
            if (data.room.valid) {
                json["room"] = vec3Json(data.room.position);
                json["yaw"] = roundTo(data.room.yawDegrees, 1000.0);
            }
            if (data.handRight.valid) {
                json["hR"] = vec3Json(data.handRight.value);
            }
            if (data.handLeft.valid) {
                json["hL"] = vec3Json(data.handLeft.value);
            }
            if (data.hmd.valid) {
                json["hmd"] = vec3Json(data.hmd.value);
            }
            if (data.weapon.valid) {
                json["weap"] = vec3Json(data.weapon.value);
            }
            json["gR"] = grabFeedJson(data.grabRight);
            json["gL"] = grabFeedJson(data.grabLeft);
            return json;
        }

        void pushLatestSnapshot()
        {
            // Reset up front so a frame that lands mid-push schedules the next task
            // rather than being dropped: coalescing "latest wins" stream.
            s_pushScheduled.store(false, std::memory_order_release);

            if (!s_prisma || !s_prismaVR || !s_view || !s_domReady.load(std::memory_order_acquire)) {
                return;
            }

            observeSpatialApplyHealth();

            MonitorFrameData data;
            {
                std::scoped_lock lock(s_stateMutex);
                data = s_latestFrameData;
            }

            if (!data.providerReady || !data.panelPose || !panelPoseWithinCapabilities(*data.panelPose)) {
                hidePanelAndResetSpatialState();
                return;
            }

            const auto update = makeSpatialUpdate(PRISMA_UI_VR_API::SpatialPresentationMode::WorldQuad, &*data.panelPose);
            if (!update) {
                hidePanelAndResetSpatialState();
                return;
            }
            const auto result = s_prismaVR->SubmitSpatialUpdate(s_view, &*update);
            reportSpatialResult(s_worldSubmitLogState, "WorldQuad submission", result);
            if (!spatialResultAccepted(result)) {
                hidePanelAndResetSpatialState();
                return;
            }

            s_worldPresentationActive.store(true, std::memory_order_release);
            s_nextHeadLockedResetAttempt = {};
            if (!s_panelVisible.exchange(true, std::memory_order_acq_rel)) {
                s_prisma->Show(s_view);
            }

            const auto payload = buildModelJson(data).dump();
            s_prisma->InteropCall(s_view, "rockMonitorUpdate", payload.c_str());
        }

        void onDomReady(PrismaView view)
        {
            if (!s_prisma || view != s_view) {
                return;
            }
            s_prisma->Hide(s_view);
            s_domReady.store(true, std::memory_order_release);
            logger::info("ROCK Monitor: Prisma view DOM ready ({}).", s_view);
            schedulePush();
        }

        void ensurePrismaView()
        {
            if (!s_prisma || !s_prismaVR ||
                !s_spatialCapabilitiesReady.load(std::memory_order_acquire) || s_view != 0) {
                return;
            }
            bool expected = false;
            if (!s_viewRequested.compare_exchange_strong(expected, true)) {
                return;
            }

            PRISMA_UI_VR_API::ViewCreateOptionsV1 options{};
            options.structSize = sizeof(options);
            options.networkAccessPolicy = PRISMA_UI_VR_API::NetworkAccessPolicy::LocalOnly;
            s_view = s_prismaVR->CreateViewWithOptions("ROCK-Monitor/index.html", &onDomReady, &options);
            if (!s_view) {
                s_viewRequested.store(false, std::memory_order_release);
                logger::error("ROCK Monitor: failed to create LocalOnly Prisma FO4VR view.");
                return;
            }
            // Create registers the view as visible; hide immediately so no default
            // head-locked frame can escape before the first accepted WorldQuad update.
            s_prisma->Hide(s_view);

            PRISMA_UI_VR_API::NetworkAccessPolicy appliedPolicy{};
            if (!s_prismaVR->GetNetworkAccessPolicy(s_view, &appliedPolicy) ||
                appliedPolicy != PRISMA_UI_VR_API::NetworkAccessPolicy::LocalOnly) {
                logger::error("ROCK Monitor: Prisma view did not retain its atomic LocalOnly network policy; destroying view.");
                s_prisma->Destroy(s_view);
                s_view = 0;
                s_viewRequested.store(false, std::memory_order_release);
                return;
            }
            s_prisma->RegisterConsoleCallback(s_view, [](PrismaView, PRISMA_UI_API::ConsoleMessageLevel level, const char* message) {
                switch (level) {
                case PRISMA_UI_API::ConsoleMessageLevel::Error:
                    logger::error("ROCK Monitor JS: {}", message ? message : "");
                    break;
                case PRISMA_UI_API::ConsoleMessageLevel::Warning:
                    logger::warn("ROCK Monitor JS: {}", message ? message : "");
                    break;
                default:
                    logger::info("ROCK Monitor JS: {}", message ? message : "");
                    break;
                }
            });
            s_prisma->SetOrder(s_view, 84);
            logger::info("ROCK Monitor: requested Prisma FO4VR LocalOnly view {}.", s_view);
        }

        [[nodiscard]] RoomSample sampleRoom()
        {
            RoomSample sample{};
            if (const auto* playerNodes = f4vr::getPlayerNodes(); playerNodes && playerNodes->roomnode) {
                const auto& world = playerNodes->roomnode->world;
                sample.position = world.translate;
                sample.yawDegrees = std::atan2(world.rotate.entry[1][0], world.rotate.entry[0][0]) * 57.295779513f;
                sample.valid = isFinitePoint(sample.position) && std::isfinite(sample.yawDegrees);
            }
            return sample;
        }

        [[nodiscard]] Vec3Sample sampleProviderTransform(const ::rock::provider::RockProviderTransform& transform)
        {
            Vec3Sample sample{};
            sample.value = RE::NiPoint3{ transform.translate[0], transform.translate[1], transform.translate[2] };
            sample.valid = isFinitePoint(sample.value);
            return sample;
        }

        [[nodiscard]] Vec3Sample sampleWeaponNode(const RockProviderFrameSnapshot& snapshot)
        {
            Vec3Sample sample{};
            // Non-owning engine pointer filled by ROCK for this frame; consumed on
            // the same game-thread frame it was published on.
            if (snapshot.weaponNode != 0) {
                const auto* node = reinterpret_cast<const RE::NiAVObject*>(snapshot.weaponNode);
                sample.value = node->world.translate;
                sample.valid = isFinitePoint(sample.value);
            }
            return sample;
        }
    }

    void initialize()
    {
        if (s_initialized.exchange(true, std::memory_order_acq_rel)) {
            onGameSessionReady();
            return;
        }

        // Configuration ownership is independent from rendering capability: create
        // and watch the runtime-owned INI even when Prisma is missing/incompatible.
        try {
            s_config = std::make_unique<MonitorConfig>();
            if (!s_config->load()) {
                logger::warn("ROCK Monitor: '{}' was not loaded; compiled Monitor defaults remain active.",
                    s_config->path().string());
            }
        } catch (const std::exception& exception) {
            s_config.reset();
            logger::error("ROCK Monitor: config service initialization failed; compiled Monitor defaults remain active ({}).",
                exception.what());
        }

        const auto prismaBase =
            PRISMA_UI_API::RequestPluginAPI<PRISMA_UI_API::IVPrismaUI4>();
        const auto prismaVR =
            PRISMA_UI_VR_API::RequestPluginVRAPI<PRISMA_UI_VR_API::IVPrismaUIVR1>();
        if (!prismaBase || !prismaVR) {
            logger::warn(
                "ROCK Monitor: PrismaUI_F4 interfaces unavailable (base V4={}, FO4VR extension V1={}); "
                "grab clock panel disabled for this session.",
                prismaBase != nullptr,
                prismaVR != nullptr);
            return;
        }
        s_prisma = prismaBase;
        s_prismaVR = prismaVR;
        if (!acquireSpatialCapabilities()) {
            logger::error(
                "ROCK Monitor: PrismaUI_F4 FO4VR extension lacks the required GPU WorldQuad + "
                "scene-depth contract; grab clock panel disabled.");
            s_prisma = nullptr;
            s_prismaVR = nullptr;
            return;
        }
        logger::info("ROCK Monitor: PrismaUI_F4 base V4 and FO4VR extension V1 acquired.");
        ensurePrismaView();
    }

    void onGameSessionReady()
    {
        if (s_prisma && s_prismaVR) {
            ensurePrismaView();
        }
    }

    void onProviderFrame(const RockProviderFrameSnapshot& snapshot)
    {
        if (!s_prisma || !s_prismaVR || s_view == 0) {
            return;
        }

        // One immutable snapshot gates this entire frame and supplies its pose,
        // so a hot reload cannot combine an old enable state with a new transform.
        const auto configSnapshot = s_config ? s_config->snapshot() : nullptr;
        const bool configuredEnabled =
            configSnapshot ? configSnapshot->enabled : kDefaultMonitorSettings.enabled;
        const int configuredEnabledStatus = configuredEnabled ? 1 : 0;
        const int previousConfiguredEnabledStatus =
            s_lastConfiguredEnabledStatus.exchange(configuredEnabledStatus, std::memory_order_acq_rel);
        if (previousConfiguredEnabledStatus != configuredEnabledStatus) {
            logger::info(
                "ROCK Monitor: panel {} by hot-reloaded [PanelPose] bEnabled.",
                configuredEnabled ? "enabled" : "disabled");
        }
        if (!configuredEnabled && previousConfiguredEnabledStatus == 0) {
            // Steady disabled state: skip all sampling; a future hot reload
            // reactivates the panel without restarting the game.
            return;
        }

        const bool ready = configuredEnabled && snapshotAllowsDisplay(snapshot);
        if (ready != s_lastProviderReadyLogged.load(std::memory_order_relaxed)) {
            s_lastProviderReadyLogged.store(ready, std::memory_order_relaxed);
            logger::info(
                "ROCK Monitor: display {} (configuredEnabled={}, providerReady={}, menuBlocking={}, configBlocking={}).",
                ready ? "enabled" : "disabled",
                configuredEnabled,
                snapshot.providerReady,
                snapshot.menuBlocking,
                snapshot.configBlocking);
        }

        MonitorFrameData data{};
        data.providerReady = ready;
        data.configuredEnabled = configuredEnabled;
        data.frameIndex = snapshot.frameIndex;
        data.deltaSeconds = snapshot.deltaSeconds;
        data.menuBlocking = snapshot.menuBlocking;
        data.configBlocking = snapshot.configBlocking;

        if (ready) {
            data.panelPose = computeRightHandPanelPose(
                snapshot,
                configSnapshot ? *configSnapshot : kDefaultMonitorSettings);
            const int poseStatus = data.panelPose ? 1 : 0;
            const int previousPoseStatus = s_lastPanelPoseStatus.exchange(poseStatus, std::memory_order_relaxed);
            if (previousPoseStatus != poseStatus) {
                if (data.panelPose) {
                    logger::info("ROCK Monitor: right-hand WorldQuad pose available.");
                } else {
                    logger::warn("ROCK Monitor: right-hand transform cannot produce a finite orthonormal WorldQuad pose; panel hidden.");
                }
            }

            data.room = sampleRoom();
            data.handRight = sampleProviderTransform(snapshot.rightHandTransform);
            data.handLeft = sampleProviderTransform(snapshot.leftHandTransform);
            data.hmd = sampleProviderTransform(snapshot.hmdTransform);
            data.weapon = sampleWeaponNode(snapshot);
            ::rock::debug::copyGrabClockDebug(false, data.grabRight);
            ::rock::debug::copyGrabClockDebug(true, data.grabLeft);
        } else {
            s_lastPanelPoseStatus.store(-1, std::memory_order_relaxed);
        }

        {
            std::scoped_lock lock(s_stateMutex);
            s_latestFrameData = data;
        }
        schedulePush();
    }
}
