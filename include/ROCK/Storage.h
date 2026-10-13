#pragma once
#include "Abi.h"
#include "Grab.h"
#include "WeaponV1_1.h"

// Physical storage for holster, backpack and stash mods: claims on stored
// references, held-item contact suppression, highlights, inventory-item
// displays with authored grips, shoulder routing and stash conversion.
// Every call runs on the ROCK frame-owner thread. No engine pointer crosses
// this boundary.
#pragma pack(push, 8)
namespace rock::api::storage
{
    inline constexpr InterfaceId kInterfaceId = InterfaceId::Storage;
    inline constexpr std::uint32_t kMajor = 2;
    inline constexpr std::uint32_t kMinor = 0;
    inline constexpr std::uint32_t kMaxClaims = 256;
    inline constexpr std::uint32_t kClaimReservationFrames = 120;
    inline constexpr std::uint32_t kMaxLeaseFrames = 3;
    inline constexpr std::uint32_t kMaxDisplays = 32;
    inline constexpr std::uint32_t kMaxRoutingOwners = 8;
    inline constexpr std::uint32_t kMaxConverters = 8;
    inline constexpr std::uint32_t kFingerLocalTransformCount = 15;

    // ------------------------------------------------------------------
    // Claims and held items
    // ------------------------------------------------------------------

    // hand names the hand carrying reference with Grab gripSequence, or None
    // when neither hand may hold it.
    struct Target
    {
        std::uint32_t size{ sizeof(Target) };
        std::uint32_t version{ 1 };
        ReferenceKey reference{};
        Hand hand{ Hand::None };
        std::uint64_t gripSequence{ 0 };
    };

    struct EquippedExtraction
    {
        std::uint32_t size{ sizeof(EquippedExtraction) };
        std::uint32_t version{ 1 };
        Hand hand{ Hand::None };
        std::uint32_t weaponFormId{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
    };

    // ------------------------------------------------------------------
    // Highlights
    // ------------------------------------------------------------------

    // Fast green pulse on an item held by hand. reference 0 names released
    // ROCK's native-equipped expectedBase.
    struct Candidate
    {
        std::uint32_t size{ sizeof(Candidate) };
        std::uint32_t version{ 1 };
        Hand hand{ Hand::None };
        std::uint32_t reference{ 0 };
        std::uint32_t expectedBase{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        std::uint64_t gripSequence{ 0 };
    };

    // Steady green selection of nodeName, a loaded child of parentReference
    // held by the opposite hand while hand is empty. storedReference must be
    // the caller's committed claim.
    struct Selection
    {
        std::uint32_t size{ sizeof(Selection) };
        std::uint32_t version{ 1 };
        Hand hand{ Hand::None };
        std::uint32_t parentReference{ 0 };
        std::uint32_t storedReference{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        char nodeName[64]{};
    };

    // ------------------------------------------------------------------
    // Inventory-item displays
    // ------------------------------------------------------------------

    enum class DisplayFlag : std::uint32_t
    {
        // Visual and authored grips only: no collider and no contacts.
        NoCollision = 1u << 0,
    };

    /*
     * A caller-owned visual of an inventory item, attached beneath the player
     * skeleton or a reference held by a ROCK hand. item is a fresh Weapon
     * capture; ROCK revalidates that exact stack and instance every frame. Only
     * empty physical hands collide with the display's generated collider.
     * Refresh 1..kMaxLeaseFrames frames every frame; clear before detaching.
     */
    struct Display
    {
        std::uint32_t size{ sizeof(Display) };
        std::uint32_t version{ 1 };
        std::uint64_t displayId{ 0 };
        weapon::v1_1::InventoryWeapon item{};
        std::uint32_t parentReference{ 0 };
        std::uint32_t leaseFrames{ 3 };
        std::uint32_t flags{ 0 };
        char nodeName[64]{};
    };

    // valid 0: no fresh contact for an empty hand.
    struct Contact
    {
        std::uint32_t size{ sizeof(Contact) };
        std::uint32_t version{ 1 };
        Sample sample{};
        std::uint64_t displayId{ 0 };
        std::uint64_t contactFrame{ 0 };
        weapon::v1_1::InventoryWeapon item{};
        Hand hand{ Hand::None };
        std::uint32_t valid{ 0 };
    };

    enum class GripRole : std::uint32_t
    {
        Nearest = 0,
        Firing = 1,
        Support = 2,
    };

    // sample must be the current publication.
    struct GripRequest
    {
        std::uint32_t size{ sizeof(GripRequest) };
        std::uint32_t version{ 1 };
        Sample sample{};
        std::uint64_t displayId{ 0 };
        Hand hand{ Hand::None };
        GripRole role{ GripRole::Nearest };
    };

    /*
     * An authored grip seat on a display. valid 0: no complete authored pose
     * yet; no guessed grip is returned. handInDisplay and fingerLocals are
     * copied values. controllerInDisplay is the tracked wand (never the
     * rendered wrist) in display-local units scaled by displayWorld.scale;
     * controllerInTracking is the same wand in tracking-room game units. The
     * consumer owns pull thresholds and delivery.
     */
    struct GripPose
    {
        std::uint32_t size{ sizeof(GripPose) };
        std::uint32_t version{ 1 };
        Sample sample{};
        std::uint64_t displayId{ 0 };
        std::uint64_t poseSequence{ 0 };
        weapon::v1_1::InventoryWeapon item{};
        std::uint32_t valid{ 0 };
        Hand hand{ Hand::None };
        GripRole role{ GripRole::Nearest };
        std::uint32_t fingerMask{ 0 };
        Transform displayWorld{};
        Transform handInDisplay{};
        Transform fingerLocals[kFingerLocalTransformCount]{};
        Point3 controllerInDisplay{};
        Point3 controllerInTracking{};
        Point3 gripInDisplay{};
        float distanceGame{ 0.0f };
        float acquisitionRadiusGame{ 0.0f };
    };

    // ------------------------------------------------------------------
    // Shoulder routing
    // ------------------------------------------------------------------

    // Suppresses ROCK's built-in shoulder stash and retrieval for the hands in
    // handMask (right 1, left 2; 0 clears). Raw input and other interactions
    // are unaffected.
    struct ShoulderRouting
    {
        std::uint32_t size{ sizeof(ShoulderRouting) };
        std::uint32_t version{ 1 };
        std::uint32_t handMask{ 0 };
        std::uint32_t leaseFrames{ 3 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
    };

    // ------------------------------------------------------------------
    // Stash conversion
    // ------------------------------------------------------------------

    enum class StashDecision : std::uint32_t
    {
        Pass = 0,
        Reject = 1,
        Consumed = 2,
    };

    struct StashResult
    {
        StashDecision decision{ StashDecision::Reject };
        std::uint32_t inventoryFormId{ 0 };
        std::uint32_t count{ 0 };
    };

    // Called synchronously on the frame owner after physical detachment and
    // before native inventory activation. A converter owns its transaction and
    // must never return Pass for a recognized item it could not convert. It
    // must not call ROCK; registering a converter inside it returns Busy.
    using StashConverter = StashResult(ROCK_CALL*)(std::uint32_t referenceFormId, std::uint32_t baseFormId, void* userData) noexcept;

    // Permission bits are local to this interface major. Discovery grants none.
    enum class Permission : std::uint32_t { Read = 1, Write = 2, Callbacks = 4 };
    inline constexpr std::uint32_t kSupportedPermissions = 7;

    struct Api
    {
        static constexpr InterfaceId interfaceId = kInterfaceId;
        static constexpr std::uint32_t majorVersion = kMajor;
        static constexpr std::uint32_t minorVersion = kMinor;
        // Write. First claimant wins; a claim reserves the reference for
        // kClaimReservationFrames until commit, and repeating it renews it and
        // returns the same lease.
        Status(ROCK_CALL* claim)(OwnerToken owner, const Target* target, std::uint64_t* outLease) noexcept;
        // Write. After physical release only. The consumer may disable or
        // transfer the native item only after Ok. Committed claims survive
        // generation changes; owner retirement clears the runtime lease, so
        // consumers restore claims from their co-save.
        Status(ROCK_CALL* commit)(OwnerToken owner, std::uint64_t lease) noexcept;
        // Owner only. After a withdrawal succeeds or a deposit is cancelled.
        Status(ROCK_CALL* release)(OwnerToken owner, std::uint64_t lease) noexcept;
        // Read. Owner holding a live claim on the reference, or 0.
        Status(ROCK_CALL* getClaimant)(OwnerToken owner, std::uint32_t referenceFormId, OwnerToken* outClaimant) noexcept;
        // Write. Stops the held reference's bodies from colliding for three
        // frames; renew each frame. Loss of the exact grab, generation or owner
        // restores them.
        Status(ROCK_CALL* suppressHeldContacts)(OwnerToken owner, const Target* target) noexcept;
        Status(ROCK_CALL* clearHeldContacts)(OwnerToken owner) noexcept;
        // Write. Released ROCK: places the exact native-equipped weapon in the
        // world and returns its reference (RequestQueued); claim it, then retry
        // commit while Busy until the native placement handoff ends.
        Status(ROCK_CALL* extractEquipped)(OwnerToken owner, const EquippedExtraction* request, std::uint32_t* outReferenceFormId) noexcept;
        // Write. Grab's command protocol; firearm delivery also prepares the
        // authored firing grip. NotReady: retry on a later frame.
        Status(ROCK_CALL* draw)(OwnerToken owner, const grab::ForceGrabRequest* request, std::uint64_t* outCommandId) noexcept;
        // Write to set, owner only to clear; three-frame leases. Busy yields to
        // other native selection owners; a candidate pulse wins over a
        // selection.
        Status(ROCK_CALL* setCandidate)(OwnerToken owner, const Candidate* candidate) noexcept;
        Status(ROCK_CALL* clearCandidate)(OwnerToken owner, Hand hand) noexcept;
        Status(ROCK_CALL* setSelection)(OwnerToken owner, const Selection* selection) noexcept;
        Status(ROCK_CALL* clearSelection)(OwnerToken owner, Hand hand) noexcept;
        // Write. Ok means accepted, not that the collider is ready. CapacityFull
        // never evicts another owner's display.
        Status(ROCK_CALL* setDisplay)(OwnerToken owner, const Display* display) noexcept;
        Status(ROCK_CALL* clearDisplay)(OwnerToken owner, std::uint64_t displayId) noexcept;
        // Read. Contacts expire after two frames and never report an occupied
        // hand or a changed item.
        Status(ROCK_CALL* getDisplayContact)(OwnerToken owner, Hand hand, Contact* outContact) noexcept;
        // Read. Nearest prefers an in-range firing or support seat, otherwise
        // the nearest seat; a concrete role keeps that seat during a pull.
        Status(ROCK_CALL* getAuthoredGrip)(OwnerToken owner, const GripRequest* request, GripPose* outPose) noexcept;
        // Write; at most kMaxRoutingOwners owners.
        Status(ROCK_CALL* setShoulderRouting)(OwnerToken owner, const ShoulderRouting* routing) noexcept;
        // Owner only. Clears the owner's displays and shoulder routing.
        Status(ROCK_CALL* clearDisplaysAndRouting)(OwnerToken owner) noexcept;
        // Callbacks. One converter per owner, kMaxConverters in total; null
        // unregisters. userData must outlive the registration.
        Status(ROCK_CALL* registerStashConverter)(OwnerToken owner, StashConverter converter, void* userData) noexcept;
    };
}
#pragma pack(pop)
