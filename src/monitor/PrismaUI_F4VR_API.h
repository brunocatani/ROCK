/*
 * Public PrismaUI Fallout 4 VR extension API.
 *
 * This extension has an independent dispatcher and version sequence so the
 * flat V1-V4 vtable remains ABI-stable.
 */
#pragma once

#include "PrismaUI_F4_API.h"

#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace PRISMA_UI_VR_API
{
    enum class InterfaceVersion : std::uint8_t
    {
        V1 = 0
    };

    enum class NetworkAccessPolicy : std::uint32_t
    {
        Unrestricted = 0,
        LocalOnly = 1,
        RemoteNoFile = 2
    };

    struct ViewCreateOptionsV1
    {
        std::uint32_t structSize;
        NetworkAccessPolicy networkAccessPolicy;
        std::uint32_t reserved[6];
    };

    enum class SpatialResult : std::int32_t
    {
        Ok = 0,
        PendingUpdateReplaced = 1,
        InvalidView = -1,
        InvalidArgument = -2,
        InvalidStructSize = -3,
        NotReady = -4,
        Unsupported = -5,
        StaleSequence = -6,
        ShuttingDown = -7,
        InternalError = -8,
        ResourceLimit = -9
    };

    enum class SpatialCoordinateSpace : std::uint32_t
    {
        GameWorld = 0
    };

    enum class SpatialPresentationMode : std::uint32_t
    {
        HeadLockedQuad = 0,
        WorldBillboard = 1,
        WorldQuad = 2
    };

    enum SpatialFeatureBits : std::uint64_t
    {
        SpatialFeature_FullPose = 1ull << 0,
        SpatialFeature_IndependentDimensions = 1ull << 1,
        SpatialFeature_LatestOnlyUpdates = 1ull << 2,
        SpatialFeature_AppliedSequenceQuery = 1ull << 3,
        SpatialFeature_StereoCorrectProjection = 1ull << 4,
        SpatialFeature_CompositorOverlay = 1ull << 5,
        SpatialFeature_GpuRendering = 1ull << 6,
        SpatialFeature_RefreshUpTo120Hz = 1ull << 7,
        SpatialFeature_NativeNetworkPolicy = 1ull << 8,
        SpatialFeature_SceneDepthOcclusion = 1ull << 9,
        SpatialFeature_WorldPointerInput = 1ull << 10,
        SpatialFeature_CentralPointerRouting = 1ull << 11
    };

    enum SpatialUpdateFlags : std::uint32_t
    {
        SpatialUpdate_SceneDepthOcclusion = 1u << 0
    };

    enum SpatialStateFlags : std::uint32_t
    {
        SpatialState_Enabled = 1u << 0,
        SpatialState_Pending = 1u << 1,
        SpatialState_Applied = 1u << 2,
        SpatialState_ViewHidden = 1u << 3,
        SpatialState_BackendReady = 1u << 4,
        SpatialState_SceneDepthOcclusion = 1u << 5
    };

    enum SpatialPointerUpdateFlags : std::uint32_t
    {
        SpatialPointerUpdate_Active = 1u << 0
    };

    enum SpatialPointerButtonBits : std::uint32_t
    {
        SpatialPointerButton_Primary = 1u << 0
    };

    enum SpatialPointerSourceIds : std::uint32_t
    {
        SpatialPointerSource_PerView = 0,
        SpatialPointerSource_PhysicalLeftController = 1,
        SpatialPointerSource_PhysicalRightController = 2
    };

    enum SpatialPointerStateFlags : std::uint32_t
    {
        SpatialPointerState_Active = 1u << 0,
        SpatialPointerState_Pending = 1u << 1,
        SpatialPointerState_Applied = 1u << 2,
        SpatialPointerState_Hit = 1u << 3,
        SpatialPointerState_Captured = 1u << 4,
        SpatialPointerState_BackendReady = 1u << 5,
        SpatialPointerState_Routed = 1u << 6
    };

    struct SpatialPoseV1
    {
        float position[3];
        float orientation[4];
    };

    struct SpatialDimensionsV1
    {
        std::uint32_t pixelWidth;
        std::uint32_t pixelHeight;
        float physicalWidth;
        float physicalHeight;
    };

    struct SpatialUpdateV1
    {
        std::uint32_t structSize;
        SpatialCoordinateSpace coordinateSpace;
        SpatialPresentationMode presentationMode;
        std::uint32_t flags;
        std::uint64_t sequence;
        SpatialPoseV1 pose;
        SpatialDimensionsV1 dimensions;
        std::uint32_t reserved[7];
    };

    struct SpatialCapabilitiesV1
    {
        std::uint32_t structSize;
        std::uint32_t apiFlavor;
        std::uint32_t spatialRevision;
        std::uint32_t reserved0;
        std::uint64_t featureBits;
        std::uint64_t coordinateSpaceMask;
        std::uint64_t presentationModeMask;
        std::uint32_t supportedUpdateFlags;
        std::uint32_t maxPixelWidth;
        std::uint32_t maxPixelHeight;
        std::uint32_t maxSpatialViews;
        std::uint64_t maxAggregateSpatialPixels;
        std::uint32_t maxRefreshRateHz;
        float maxAbsoluteWorldPosition;
        float maxPhysicalDimension;
        float minQuaternionNormSquared;
    };

    struct SpatialStateV1
    {
        std::uint32_t structSize;
        std::uint32_t stateFlags;
        std::int32_t lastApplyResult;
        std::uint32_t reserved0;
        std::uint64_t acceptedSequence;
        std::uint64_t appliedSequence;
        SpatialCoordinateSpace coordinateSpace;
        SpatialPresentationMode presentationMode;
        std::uint32_t appliedPixelWidth;
        std::uint32_t appliedPixelHeight;
        float appliedPhysicalWidth;
        float appliedPhysicalHeight;
        SpatialPoseV1 appliedPose;
        std::uint32_t replacedPendingUpdateCount;
        std::uint32_t reserved[2];
    };

    struct SpatialPointerUpdateV1
    {
        std::uint32_t structSize;
        SpatialCoordinateSpace coordinateSpace;
        std::uint32_t flags;
        std::uint32_t buttonLevels;
        std::uint64_t sequence;
        float rayOrigin[3];
        float maxDistance;
        float rayDirection[3];
        std::int32_t scrollDeltaX;
        std::int32_t scrollDeltaY;
        std::uint32_t pointerSourceId;
        std::uint32_t reserved[8];
    };

    struct SpatialPointerStateV1
    {
        std::uint32_t structSize;
        std::uint32_t stateFlags;
        std::int32_t lastApplyResult;
        std::uint32_t buttonLevels;
        std::uint64_t acceptedSequence;
        std::uint64_t appliedSequence;
        float hitDistance;
        float hitUv[2];
        std::int32_t pixelX;
        std::int32_t pixelY;
        std::uint32_t replacedPendingUpdateCount;
        std::uint32_t pointerSourceId;
        std::uint32_t reserved[9];
    };

    class IVPrismaUIVR1
    {
    protected:
        ~IVPrismaUIVR1() = default;

    public:
        virtual SpatialResult GetSpatialCapabilities(
            SpatialCapabilitiesV1* outCapabilities) noexcept = 0;
        virtual SpatialResult SubmitSpatialUpdate(
            PrismaView view,
            const SpatialUpdateV1* update) noexcept = 0;
        virtual SpatialResult GetSpatialState(
            PrismaView view,
            SpatialStateV1* outState) noexcept = 0;
        virtual PrismaView CreateViewWithOptions(
            const char* htmlPath,
            PRISMA_UI_API::OnDomReadyCallback onDomReadyCallback,
            const ViewCreateOptionsV1* options) noexcept = 0;
        virtual bool SetNetworkAccessPolicy(
            PrismaView view,
            NetworkAccessPolicy policy) noexcept = 0;
        virtual bool GetNetworkAccessPolicy(
            PrismaView view,
            NetworkAccessPolicy* outPolicy) noexcept = 0;
        virtual SpatialResult SubmitSpatialPointerUpdate(
            PrismaView view,
            const SpatialPointerUpdateV1* update) noexcept = 0;
        virtual SpatialResult CancelSpatialPointer(PrismaView view) noexcept = 0;
        virtual SpatialResult GetSpatialPointerState(
            PrismaView view,
            SpatialPointerStateV1* outState) noexcept = 0;
    };

    static_assert(sizeof(ViewCreateOptionsV1) == 32);
    static_assert(alignof(ViewCreateOptionsV1) == 4);
    static_assert(offsetof(ViewCreateOptionsV1, networkAccessPolicy) == 4);
    static_assert(std::is_standard_layout_v<ViewCreateOptionsV1>);
    static_assert(std::is_trivially_copyable_v<ViewCreateOptionsV1>);

    static_assert(sizeof(SpatialPoseV1) == 28);
    static_assert(sizeof(SpatialDimensionsV1) == 16);
    static_assert(sizeof(SpatialUpdateV1) == 96);
    static_assert(sizeof(SpatialCapabilitiesV1) == 80);
    static_assert(sizeof(SpatialStateV1) == 96);
    static_assert(sizeof(SpatialPointerUpdateV1) == 96);
    static_assert(sizeof(SpatialPointerStateV1) == 96);
    static_assert(alignof(SpatialUpdateV1) == 8);
    static_assert(alignof(SpatialCapabilitiesV1) == 8);
    static_assert(alignof(SpatialStateV1) == 8);
    static_assert(alignof(SpatialPointerUpdateV1) == 8);
    static_assert(alignof(SpatialPointerStateV1) == 8);
    static_assert(offsetof(SpatialUpdateV1, sequence) == 16);
    static_assert(offsetof(SpatialUpdateV1, pose) == 24);
    static_assert(offsetof(SpatialCapabilitiesV1, maxAggregateSpatialPixels) == 56);
    static_assert(offsetof(SpatialStateV1, acceptedSequence) == 16);
    static_assert(offsetof(SpatialStateV1, appliedPose) == 56);
    static_assert(offsetof(SpatialPointerUpdateV1, sequence) == 16);
    static_assert(offsetof(SpatialPointerUpdateV1, rayDirection) == 40);
    static_assert(offsetof(SpatialPointerUpdateV1, pointerSourceId) == 60);
    static_assert(offsetof(SpatialPointerStateV1, acceptedSequence) == 16);
    static_assert(offsetof(SpatialPointerStateV1, hitDistance) == 32);
    static_assert(offsetof(SpatialPointerStateV1, pointerSourceId) == 56);
    static_assert(std::is_standard_layout_v<SpatialUpdateV1>);
    static_assert(std::is_trivially_copyable_v<SpatialUpdateV1>);
    static_assert(std::is_standard_layout_v<SpatialCapabilitiesV1>);
    static_assert(std::is_trivially_copyable_v<SpatialCapabilitiesV1>);
    static_assert(std::is_standard_layout_v<SpatialStateV1>);
    static_assert(std::is_trivially_copyable_v<SpatialStateV1>);
    static_assert(std::is_standard_layout_v<SpatialPointerUpdateV1>);
    static_assert(std::is_trivially_copyable_v<SpatialPointerUpdateV1>);
    static_assert(std::is_standard_layout_v<SpatialPointerStateV1>);
    static_assert(std::is_trivially_copyable_v<SpatialPointerStateV1>);

    template <class Interface>
    struct InterfaceVersionMap;

    template <>
    struct InterfaceVersionMap<IVPrismaUIVR1>
    {
        static constexpr auto version = InterfaceVersion::V1;
    };

    using RequestPluginVRAPIFunc = void* (*)(InterfaceVersion version);

    [[nodiscard]] inline void* RequestPluginVRAPI(
        InterfaceVersion version = InterfaceVersion::V1) noexcept
    {
        const auto module = GetModuleHandleW(L"PrismaUI_F4.dll");
        if (!module) {
            return nullptr;
        }

        const auto request = reinterpret_cast<RequestPluginVRAPIFunc>(
            GetProcAddress(module, "RequestPluginVRAPI"));
        return request ? request(version) : nullptr;
    }

    template <class Interface>
    [[nodiscard]] inline Interface* RequestPluginVRAPI() noexcept
    {
        return static_cast<Interface*>(
            RequestPluginVRAPI(InterfaceVersionMap<Interface>::version));
    }
}
