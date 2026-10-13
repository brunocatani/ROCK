#pragma once
#include "Abi.h"

#pragma pack(push, 8)
namespace rock::api::references {
    inline constexpr InterfaceId kInterfaceId = InterfaceId::References;
    inline constexpr std::uint32_t kMajor = 1;
    inline constexpr std::uint32_t kMinor = 0;
    inline constexpr std::uint32_t kPowerArmorPointCount = 2;


    enum class NativeOpenStateV1 : std::uint32_t
    {
        NotApplicable = 0, Open = 1, Opening = 2, Closed = 3, Closing = 4,
    };

    enum class TargetDetailFlagV1 : std::uint32_t
    {
        Reference = 1u << 0, Body = 1u << 1, Anchor = 1u << 2,
        Normal = 1u << 3, MeshPart = 1u << 4,
        ActivationBlocked = 1u << 5, OpenState = 1u << 6,
        FurnitureUse = 1u << 7, PowerArmorClassification = 1u << 8,
        PowerArmorFrame = 1u << 9, PowerArmorActor = 1u << 10,
    };

    struct ReferenceQueryV1
    {
        std::uint32_t size{ sizeof(ReferenceQueryV1) };
        std::uint32_t version{ 1 };
        std::uint32_t referenceFormId{ 0 };
        std::uint32_t referenceNativeHandle{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        std::int32_t furnitureMarkerIndex{ 0 };
    };

    struct PowerArmorPointPoseV1
    {
        PowerArmorPointV1 point{};
        std::uint32_t valid{ 0 };
        Transform world{};
        Transform frameLocal{};
    };

    struct ReferenceInteractionV1
    {
        std::uint32_t size{ sizeof(ReferenceInteractionV1) };
        std::uint32_t version{ 1 };
        std::uint32_t flags{ 0 };
        std::uint32_t referenceFormId{ 0 };
        std::uint32_t referenceNativeHandle{ 0 };
        std::uint32_t baseFormId{ 0 };
        std::uint32_t baseFormType{ 0 };
        std::uint32_t activationBlocked{ 0 };
        NativeOpenStateV1 openState{};
        std::uint32_t furnitureInUse{ 0 };
        std::uint32_t furnitureInUseIncludingReservations{ 0 };
        std::int32_t furnitureMarkerIndex{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint64_t frameIndex{ 0 };
    };

    struct PowerArmorTargetV1
    {
        std::uint32_t size{ sizeof(PowerArmorTargetV1) };
        std::uint32_t version{ 1 };
        ReferenceInteractionV1 touchedReference{};
        ReferenceInteractionV1 frameReference{};
        std::uint32_t flags{ 0 };
        std::uint32_t actorFormId{ 0 };
        PowerArmorPointPoseV1 points[2]{};
    };


    // Permission bits are local to this interface major. Discovery grants none.
    enum class PermissionV1 : std::uint32_t { Read=1 };
    inline constexpr std::uint32_t kSupportedPermissions = 1;

    struct ApiV1 {
        static constexpr InterfaceId interfaceId=kInterfaceId;
        static constexpr std::uint32_t majorVersion=kMajor;
        static constexpr std::uint32_t minorVersion=kMinor;
        Status(ROCK_CALL* queryReferenceInteractionV1)(std::uint64_t ownerToken, const ReferenceQueryV1* query, ReferenceInteractionV1* outState) noexcept;
        Status(ROCK_CALL* queryPowerArmorTargetV1)(std::uint64_t ownerToken, const ReferenceQueryV1* query, PowerArmorTargetV1* outTarget) noexcept;
        // Family-local publication identity; compare before combining readbacks.
        Status(ROCK_CALL* getSample)(OwnerToken owner, SampleV1* outSample) noexcept;
    };
}

#pragma pack(pop)
