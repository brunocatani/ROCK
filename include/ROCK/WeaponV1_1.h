#pragma once
#include "Weapon.h"
#include <cstddef>
#include <type_traits>

#pragma pack(push, 8)
namespace rock::api::weapon::v1_1 {
    inline constexpr InterfaceId kInterfaceId = weapon::kInterfaceId;
    inline constexpr std::uint32_t kMajor = 1;
    inline constexpr std::uint32_t kMinor = 1;

    // Capture on the ROCK frame thread immediately before requesting equip.
    // Keys are opaque comparison witnesses, never pointers to dereference or
    // identities to save. stackIndex is the native inventory stack index.
    struct InventoryWeapon {
        std::uint32_t size{sizeof(InventoryWeapon)};
        std::uint32_t baseFormId{};
        std::uint32_t stackIndex{};
        std::uint32_t count{};
        std::uint64_t stackKey{};
        std::uint64_t instanceKey{};
        std::uint64_t frameIndex{};
        std::uint32_t worldGeneration{};
        std::uint32_t skeletonGeneration{};
        std::uint32_t providerGeneration{};
        std::uint32_t reserved{};
    };

    struct EquipRequest {
        std::uint32_t size{sizeof(EquipRequest)};
        Hand hand{Hand::None};
        InventoryWeapon item{};
    };

    enum class EquipState : std::uint32_t {
        Unknown, Queued, Switching, Succeeded, Failed, Cancelled
    };
    enum class EquipFailure : std::uint32_t {
        None, LifecycleLost, OwnerLost, HandUnavailable, HandBusy,
        InventoryChanged, InfrastructureUnavailable, OutgoingTransferFailed,
        NativeEquipFailed, GripOrPresentationFailed, CancelledBeforeCommit,
        EquippedWeaponChanged, MenuBlocked
    };
    enum class EquipResultFlag : std::uint32_t {
        OutgoingRemoved = 1u << 0,
        OutgoingRetained = 1u << 1,
        IncomingEquipAccepted = 1u << 2,
        IncomingGripAcquired = 1u << 3,
        IncomingPresented = 1u << 4
    };
    struct EquipResult {
        std::uint32_t size{sizeof(EquipResult)};
        EquipState state{EquipState::Unknown};
        std::uint64_t commandId{};
        EquipFailure failure{EquipFailure::None};
        std::uint32_t flags{};
        Hand hand{Hand::None};
        std::uint32_t incomingFormId{};
        std::uint32_t outgoingFormId{};
        std::uint32_t outgoingReferenceFormId{};
        std::uint64_t transferSequence{};
        std::uint32_t worldGeneration{};
        std::uint32_t skeletonGeneration{};
        std::uint32_t providerGeneration{};
        std::uint32_t reserved{};
    };

    // Negotiate major 1, minimum minor 1 and sizeof(Api). Bind Weapon Read |
    // Write. All four additions are ROCK frame-thread calls, outside synchronous
    // Grab event callbacks. The original 1.0 table and headers remain unchanged.
    struct Api {
        static constexpr InterfaceId interfaceId=kInterfaceId;
        static constexpr std::uint32_t majorVersion=kMajor;
        static constexpr std::uint32_t minorVersion=kMinor;
        weapon::ApiV1 v1;
        Status(ROCK_CALL* captureInventoryWeapon)(OwnerToken, std::uint32_t baseFormId,
            std::uint32_t stackIndex, InventoryWeapon*) noexcept;
        // An explicit draw gesture equips into the requested physical hand.
        // A weapon carried only by the other hand is retained there using the
        // existing Toggle Drop controls. A busy destination rejects the request.
        // One switch may be outstanding. RequestQueued is admission, not success.
        Status(ROCK_CALL* requestInventoryEquip)(OwnerToken, const EquipRequest*, std::uint64_t*) noexcept;
        // Results are owner-scoped; the latest 64 terminal results are retained
        // for this provider lifetime. Inspect flags even when the switch fails.
        Status(ROCK_CALL* getInventoryEquipResult)(OwnerToken, std::uint64_t, EquipResult*) noexcept;
        // Cancellation is accepted only before outgoing removal/native equip.
        // Once committed, returns AlreadyCommitted and ROCK finishes cleanup
        // even if the requesting owner disappears. No consumer callback is kept.
        Status(ROCK_CALL* cancelInventoryEquip)(OwnerToken, std::uint64_t) noexcept;
    };
    static_assert(std::is_standard_layout_v<Api>);
    static_assert(offsetof(Api, v1) == 0);
    static_assert(offsetof(Api, captureInventoryWeapon) == sizeof(weapon::ApiV1));
    static_assert(sizeof(Api) == sizeof(weapon::ApiV1) + 4 * sizeof(void*));
}
#pragma pack(pop)
