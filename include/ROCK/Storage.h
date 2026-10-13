#pragma once
#include "Abi.h"
#include "Grab.h"

// Independent, opt-in storage service. Resolve kExport on the selected ROCK
// DLL; querying is read-only and accepts exactly major 1 and this table size.
// Existing Core/Grab tables and their negotiation remain unchanged.
// All calls require the ROCK frame owner thread and a Core owner bound to
// Grab V1 with Read|Control. No native pointer crosses this boundary.
#pragma pack(push, 8)
namespace rock::api::storage
{
    inline constexpr char kExport[] = "ROCK_Storage_QueryV1";
    inline constexpr unsigned kMaximumClaims = 256;
    struct TargetV1
    {
        std::uint32_t size{sizeof(TargetV1)}, version{1};
        ReferenceKey reference{};
        Hand hand{Hand::None};
        std::uint64_t gripSequence{};
    };
    struct EquippedExtractionV1
    {
        std::uint32_t size{sizeof(EquippedExtractionV1)}, version{1};
        Hand hand{Hand::None};
        std::uint32_t weapon{}, worldGeneration{}, skeletonGeneration{}, providerGeneration{}, reserved{};
    };
    struct ApiV1
    {
        std::uint32_t size{sizeof(ApiV1)}, version{1};
        // First claimant wins. Held targets require the actual carrying hand
        // and matching Grab gripSequence; None requires neither hand holds it.
        // A claim reserves for 120 frames until commit. Calling claim again
        // with the same owner/reference renews it and returns the same token.
        Status (ROCK_CALL* claim)(OwnerToken, const TargetV1*, std::uint64_t* lease) noexcept;
        // Commit only after physical release. Pins ROCK-owned firearm state;
        // capacity failure leaves the item with its original owner. Consumer
        // may disable/transfer the native item only after Ok.
        Status (ROCK_CALL* commit)(OwnerToken, std::uint64_t lease) noexcept;
        // Release only after a withdrawal succeeds or a deposit is cancelled.
        // Stored claims survive world/skeleton changes, but owner retirement
        // clears runtime leases. Consumers restore claims from their co-save.
        Status (ROCK_CALL* release)(OwnerToken, std::uint64_t lease) noexcept;
        Status (ROCK_CALL* claimant)(OwnerToken, std::uint32_t reference, OwnerToken* claimant) noexcept;
        // Held-reference contact lease, renewed each frame (expires after 3).
        // Preserves the actual grab body. Losing the exact grab, generation,
        // owner or world restores this owner's changes through ROCK's registry.
        Status (ROCK_CALL* suppressContacts)(OwnerToken, const TargetV1*) noexcept;
        Status (ROCK_CALL* clearContacts)(OwnerToken) noexcept;
        // Released ROCK only: exact native-equipped item -> world placement.
        // RequestQueued returns its reference; claim it, then retry commit
        // while Busy until ROCK's native placement handoff has ended. V2
        // returns UnsupportedVersion: its firearms already are loose items.
        Status (ROCK_CALL* extractEquipped)(OwnerToken, const EquippedExtractionV1*, std::uint32_t* reference) noexcept;
        // Same command/result protocol as Grab. V2 firearm delivery also
        // prepares its authored firing grip; NotReady means retry on a later
        // frame before any command has been admitted. Ordinary items use Grab.
        Status (ROCK_CALL* draw)(OwnerToken, const grab::ForceGrabRequest*, std::uint64_t* command) noexcept;
    };
    using Query = const ApiV1* (ROCK_CALL*)(std::uint32_t major, std::uint32_t bytes) noexcept;
    static_assert(sizeof(TargetV1) == 40 && sizeof(EquippedExtractionV1) == 32 && sizeof(ApiV1) == 72);
}
#pragma pack(pop)
