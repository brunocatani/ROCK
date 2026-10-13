#pragma once
#include "Abi.h"

// Independent opt-in service. Storage/StorageDisplay V1 and every modular
// provider table retain their existing layout, requirements and behavior.
#pragma pack(push,8)
namespace rock::api::storage_interaction
{
    inline constexpr char kExport[]="ROCK_StorageInteraction_QueryV1";
    inline constexpr std::uint32_t kMaximumDisplays=16;
    struct DisplayV1 {
        std::uint32_t size{sizeof(DisplayV1)}, version{1};
        std::uint64_t displayId{};
        std::uint32_t parentReference{}, storedReference{};
        std::uint32_t worldGeneration{}, skeletonGeneration{}, providerGeneration{}, leaseFrames{3};
        char nodeName[64]{};
    };
    struct ShoulderRoutingV1 {
        std::uint32_t size{sizeof(ShoulderRoutingV1)}, version{1};
        std::uint32_t handMask{}, leaseFrames{3}; // right=1, left=2; zero clears
        std::uint32_t worldGeneration{}, skeletonGeneration{}, providerGeneration{}, reserved{};
    };
    struct ContactV1 {
        std::uint32_t size{sizeof(ContactV1)}, version{1};
        SampleV1 sample{};
        std::uint64_t displayId{}, contactFrame{};
        std::uint32_t storedReference{}, valid{};
        Hand hand{Hand::None};
        std::uint32_t reserved{};
    };
    // All calls run on the ROCK frame-owner thread and require an existing
    // Core owner bound to Grab Read|Control (contact reads need Read only).
    // Resolve this export on the selected regular ROCK module. Query is read
    // only and accepts exactly major 1 and sizeof(ApiV1); no V1 table is extended.
    //
    // A display names a loaded, collisionless child beneath the player or a
    // reference held in either hand. Its storedReference must be this owner's
    // committed Storage claim. The real stored item remains disabled/owned by
    // its consumer. Only generated empty-hand solver bodies collide with the
    // display: held items, weapons, world, actors and player capsule do not.
    // ROCK creates isolated per-visible-mesh convex colliders; it never enables
    // the stored native body's collision or modifies the consumer's visual.
    // Refresh leaseFrames 1..3 every frame; clear before detaching a display.
    // Pose/source/parent/claim/generation/owner loss retires its generated body.
    // Geometry is prepared cooperatively; accepted registration can precede
    // collision readiness. Contact valid=0 means no fresh eligible contact.
    // Contacts expire after two owner frames and are never returned for an
    // occupied hand, changed item/parent or stale generation. setDisplay's Ok
    // means accepted, not geometrically ready; unsupported meshes stay visual.
    // There are 16 display records and eight routing owners process-wide.
    // CapacityFull never evicts another owner's record. InvalidSize and
    // UnsupportedVersion reject record changes; GenerationMismatch rejects
    // an obsolete request; OwnerConflict means the stored claim is not owned.
    // Clear calls are idempotent and need a live owner on the owner thread.
    //
    // Shoulder routing suppresses only ROCK's built-in shoulder stash/sheath
    // and shoulder retrieval for the named physical hands. It does not consume
    // raw input, disable locomotion or change mouth/trigger/other interactions.
    // Expiry, clear, owner retirement or generation loss restores that routing.
    struct ApiV1 {
        std::uint32_t size{sizeof(ApiV1)}, version{1};
        Status(ROCK_CALL* setDisplay)(OwnerToken,const DisplayV1*) noexcept;
        Status(ROCK_CALL* clearDisplay)(OwnerToken,std::uint64_t displayId) noexcept;
        Status(ROCK_CALL* getContact)(OwnerToken,Hand,ContactV1*) noexcept;
        Status(ROCK_CALL* setShoulderRouting)(OwnerToken,const ShoulderRoutingV1*) noexcept;
        Status(ROCK_CALL* clearOwner)(OwnerToken) noexcept;
    };
    using Query=const ApiV1*(ROCK_CALL*)(std::uint32_t major,std::uint32_t bytes) noexcept;
    static_assert(sizeof(DisplayV1)==104 && sizeof(ShoulderRoutingV1)==32 && sizeof(ApiV1)==48);
}
#pragma pack(pop)
