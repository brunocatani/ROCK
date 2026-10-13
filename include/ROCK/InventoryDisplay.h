#pragma once
#include "StorageInteraction.h"
#include "WeaponV1_1.h"

#pragma pack(push,8)
namespace rock::api::inventory_display
{
    inline constexpr char kExport[]="ROCK_InventoryDisplay_QueryV1";
    inline constexpr std::uint32_t kMajor=1,kMinor=0,kMaximumDisplays=32;
    enum class GripRole : std::uint32_t { Nearest, Firing, Support };
    struct Display {
        std::uint32_t size{sizeof(Display)},version{1};
        std::uint64_t displayId{};
        weapon::v1_1::InventoryWeapon item;
        std::uint32_t parentReference{},leaseFrames{3};
        char nodeName[64]{};
    };
    struct Contact {
        std::uint32_t size{sizeof(Contact)},version{1};
        SampleV1 sample;
        std::uint64_t displayId{},contactFrame{};
        weapon::v1_1::InventoryWeapon item;
        Hand hand{Hand::None};
        std::uint32_t valid{};
    };
    struct GripRequest {
        std::uint32_t size{sizeof(GripRequest)},version{1};
        SampleV1 sample;
        std::uint64_t displayId{};
        Hand hand{Hand::None};
        GripRole role{GripRole::Nearest};
    };
    struct GripPose {
        std::uint32_t size{sizeof(GripPose)},version{1};
        SampleV1 sample;
        std::uint64_t displayId{},poseSequence{};
        weapon::v1_1::InventoryWeapon item;
        std::uint32_t valid{};
        Hand hand{Hand::None};
        GripRole role{GripRole::Nearest};
        std::uint32_t fingerMask{};
        Transform displayWorld{},handInDisplay{},fingerLocals[15]{};
        Point3 controllerInDisplay{},controllerInTracking{},gripInDisplay{};
        float distanceGame{},acquisitionRadiusGame{};
    };
    // Independent, opt-in provider. StorageInteraction 1.0/1.1 remain unchanged.
    // Bind Core plus Grab Read|Control and Weapon Read on the frame-owner thread.
    // Capture a fresh Weapon 1.1 InventoryWeapon before setDisplay. The provider
    // revalidates that exact native stack/instance; stale identities fail closed.
    // Parent must be the player skeleton or a reference physically held by a
    // ROCK hand. nodeName names a caller-owned attached visual, never a new item.
    // Geometry is prepared cooperatively; registration acceptance is not proof
    // of collider readiness. Collisions are limited to empty physical hands.
    // No inventory ownership, equip state, count or saved data is changed.
    // getAuthoredGrip reports copied authored poses and raw controller positions
    // in game units, with the same display-local and room-relative conventions
    // as StorageInteraction 1.1. Missing authored data returns valid=0.
    // Lease is 1..3 owner frames. Clear/expiry/lifecycle loss retires contacts.
    // Clear calls are idempotent. Display capacity is independent of legacy
    // StorageInteraction's capacity; identifiers are scoped to this provider.
    struct Api {
        std::uint32_t size{sizeof(Api)},version{1};
        Status(ROCK_CALL* setDisplay)(OwnerToken,const Display*) noexcept;
        Status(ROCK_CALL* clearDisplay)(OwnerToken,std::uint64_t) noexcept;
        Status(ROCK_CALL* getContact)(OwnerToken,Hand,Contact*) noexcept;
        Status(ROCK_CALL* getAuthoredGrip)(OwnerToken,const GripRequest*,GripPose*) noexcept;
        Status(ROCK_CALL* clearOwner)(OwnerToken) noexcept;
    };
    using Query=const Api*(ROCK_CALL*)(std::uint32_t major,std::uint32_t minor,std::uint32_t bytes) noexcept;
    static_assert(sizeof(Api)==48 && sizeof(Display)==144 && sizeof(GripRequest)==56);
}
#pragma pack(pop)
