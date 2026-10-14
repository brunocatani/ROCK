#pragma once
#include "Abi.h"

// FO4VR-native extension of ROCK's physics step: callbacks in ROCK's physics
// frame, Havok substep and world retirement; driving ROCK's weapon bodies by
// weapon slot; and extension handling of grab selection, extraction, grenade
// proximity and impact. Extensions own their bodies and constraints and read
// Havok directly. Native addresses are borrowed only inside the callback that
// supplied them; they are not save identities and never cross threads.
#pragma pack(push, 8)
namespace rock::api::native_interaction {
    inline constexpr InterfaceId kInterfaceId = InterfaceId::NativeInteraction;
    inline constexpr std::uint32_t kMajor = 2;
    inline constexpr std::uint32_t kMinor = 0;
    inline constexpr std::uint32_t kSupportedPermissions = 3;
    inline constexpr std::uint32_t kInvalidBody = 0x7FFF'FFFF;
    inline constexpr std::uint32_t kMaxExtensions = 8;
    inline constexpr std::uint32_t kMaxWeaponSources = 128;
    inline constexpr std::uint32_t kPassiveObstacleLayer = 55;

    // Value-only marshalling: real NiMatrix3 rows contain a fourth padding lane.
    // Never memcpy nine floats from/to that native matrix storage.
    template <class NativeTransform> Transform packTransform(const NativeTransform& native) {
        Transform value{};
        for (int row = 0; row < 3; ++row)
            for (int column = 0; column < 3; ++column) value.rotate[row * 3 + column] = native.rotate.entry[row][column];
        value.translate[0] = native.translate.x;
        value.translate[1] = native.translate.y;
        value.translate[2] = native.translate.z;
        value.scale = native.scale;
        return value;
    }
    template <class NativeTransform> NativeTransform unpackTransform(const Transform& value) {
        NativeTransform native{};
        for (int row = 0; row < 3; ++row)
            for (int column = 0; column < 3; ++column) native.rotate.entry[row][column] = value.rotate[row * 3 + column];
        if constexpr (std::extent_v<decltype(native.rotate.entry), 1> > 3)
            for (int row = 0; row < 3; ++row) native.rotate.entry[row][3] = 0;
        native.translate = { value.translate[0], value.translate[1], value.translate[2] };
        native.scale = value.scale;
        return native;
    }

    // Value 3 (the retired Render phase) is never reused.
    enum class Phase : std::uint32_t
    {
        Update = 1,
        PreCollide = 2,
        Retire = 4,
        WeaponUpdate = 5,
        WeaponContact = 6,
        WeaponRetire = 7,
        Selection = 8,
        Extraction = 9,
        Gameplay = 10,
    };

    // world and bhkWorld are the live hknpWorld and bhkWorld of this step.
    struct Frame
    {
        std::uint32_t size{ sizeof(Frame) };
        std::uint32_t version{ 1 };
        Phase phase{ Phase::Update };
        Sample sample{};
        std::uintptr_t world{ 0 };
        std::uintptr_t bhkWorld{ 0 };
        Point3 hmdPosition{};
        float deltaSeconds{ 0.0f };
        float gameToHavok{ 0.0f };
        float havokToGame{ 0.0f };
        float substepSeconds{ 0.0f };
        std::uint32_t worldReady{ 0 };
        std::uint32_t menuBlocked{ 0 };
        std::uint32_t hmdValid{ 0 };
        std::uint32_t timingValid{ 0 };
        std::uint32_t timingFallback{ 0 };
    };

    // A live Havok body. ROCK compares every field with the body before acting,
    // so a reused body id never matches. group is filter >> 16.
    struct BodyIdentity
    {
        std::uint32_t body{ kInvalidBody };
        std::uint32_t motion{ kInvalidBody };
        std::uintptr_t collisionObject{ 0 };
        std::uintptr_t ownerNode{ 0 };
        std::uintptr_t shape{ 0 };
        std::uint32_t group{ 0 };
        bool operator==(const BodyIdentity&) const = default;
    };

    // ------------------------------------------------------------------
    // Weapon driving
    // ------------------------------------------------------------------

    struct WeaponSource
    {
        std::uint32_t size{ sizeof(WeaponSource) };
        std::uint32_t version{ 1 };
        std::uint32_t body{ kInvalidBody };
        std::uint64_t generation{ 0 };
        char name[64]{};
    };

    // A generated weapon body's source in weapon space; bounds and the support
    // point in the direction asked for are filled only when one is given.
    struct SourceGeometry
    {
        std::uint32_t size{ sizeof(SourceGeometry) };
        std::uint32_t version{ 1 };
        std::uint32_t valid{ 0 };
        std::uint32_t triangleCount{ 0 };
        std::uint64_t generation{ 0 };
        Transform sourceInWeapon{};
        Bounds3 boundsLocal{};
        Point3 supportPointLocal{};
    };

    // The weapon slot's dynamic body and the pose ROCK is about to drive it to.
    struct WeaponFrame
    {
        std::uint32_t size{ sizeof(WeaponFrame) };
        std::uint32_t version{ 1 };
        std::uint32_t weaponSlot{ 0 };
        std::uint32_t body{ kInvalidBody };
        Frame frame{};
        std::uint64_t generation{ 0 };
        std::uint32_t form{ 0 };
        std::uint32_t supportOwnsPose{ 0 };
        Transform requestedWeapon{};
        Point3 centerWeaponLocal{};
        Point3 primaryGripLocal{};
        std::uint32_t primaryGripValid{ 0 };
        float recoveryDistanceGame{ 0.0f };
    };

    // Value 2 (the never-accepted RetireWeapon) is never reused.
    enum class DriveResult : std::uint32_t
    {
        Unchanged = 0,
        Controlled = 1,
    };

    // Controlled replaces the drive target and the visual weapon pose for
    // this frame; Unchanged leaves ROCK's.
    struct WeaponCommand
    {
        std::uint32_t size{ sizeof(WeaponCommand) };
        std::uint32_t version{ 1 };
        DriveResult result{ DriveResult::Unchanged };
        Transform requestedWeapon{};
        Transform presentation{};
    };

    // One contact of the slot's weapon body. normalWorld is the unit normal of
    // the target surface toward the weapon, when normalValid.
    struct Contact
    {
        std::uint32_t size{ sizeof(Contact) };
        std::uint32_t version{ 1 };
        std::uint32_t weaponSlot{ 0 };
        std::uint32_t weapon{ kInvalidBody };
        Frame frame{};
        std::uint32_t target{ kInvalidBody };
        std::uint32_t targetLayer{ 0 };
        Point3 pointWorld{};
        std::uint32_t normalValid{ 0 };
        Point3 normalWorld{};
        std::uint32_t pointCount{ 0 };
    };

    // ------------------------------------------------------------------
    // Selection, extraction, proximity, impact
    // ------------------------------------------------------------------

    enum class SelectionKind : std::uint32_t
    {
        Unhandled = 0,
        Blocked = 1,
        DirectConnectedBody = 2,
        DirectDetachedBody = 3,
        ExtractableItem = 4,
    };
    enum class SelectionStage : std::uint32_t
    {
        BeforeCoreTypes = 0,
        AfterCoreTypes = 1,
    };

    struct SelectionQuery
    {
        std::uint32_t size{ sizeof(SelectionQuery) };
        std::uint32_t version{ 1 };
        std::uint32_t body{ kInvalidBody };
        std::uintptr_t reference{ 0 };
        std::uintptr_t world{ 0 };
        std::uintptr_t hitNode{ 0 };
        Point3 hitPoint{};
        std::uint32_t hasHitPoint{ 0 };
        std::uint32_t farSelection{ 0 };
        SelectionStage stage{ SelectionStage::BeforeCoreTypes };
    };

    struct SelectionReply
    {
        std::uint32_t size{ sizeof(SelectionReply) };
        std::uint32_t version{ 1 };
        SelectionKind kind{ SelectionKind::Unhandled };
        OwnerToken owner{ 0 };
        std::uintptr_t visualNode{ 0 };
        // Extension-owned value ticket. Addresses may be opaque identity
        // values, never pointers to dereference later. Re-resolve on action.
        std::uint32_t ticket[8]{};
        char reason[64]{};
    };

    struct ExtractedReference
    {
        std::uint32_t size{ sizeof(ExtractedReference) };
        std::uint32_t version{ 1 };
        std::uint32_t nativeHandle{ 0 };
        std::uint32_t referenceForm{ 0 };
        std::uint32_t itemForm{ 0 };
    };

    struct ProximityQuery
    {
        std::uint32_t size{ sizeof(ProximityQuery) };
        std::uint32_t version{ 1 };
        float radiusGame{ 0.0f };
        std::uintptr_t reference{ 0 };
    };

    struct ProximityResult
    {
        std::uint32_t size{ sizeof(ProximityResult) };
        std::uint32_t version{ 1 };
        std::uint32_t handled{ 0 };
        std::uint32_t targetForm{ 0 };
        std::uint32_t scanned{ 0 };
        std::uint32_t truncated{ 0 };
    };

    struct ImpactQuery
    {
        std::uint32_t size{ sizeof(ImpactQuery) };
        std::uint32_t version{ 1 };
        float authoredDamage{ 0.0f };
        std::uintptr_t source{ 0 };
        std::uintptr_t target{ 0 };
    };

    struct ImpactResult
    {
        std::uint32_t size{ sizeof(ImpactResult) };
        std::uint32_t version{ 1 };
        std::uint32_t handled{ 0 };
        std::uint32_t targetForm{ 0 };
    };

    // ------------------------------------------------------------------
    // Callbacks
    // ------------------------------------------------------------------

    using ExtractionCallback = Status(ROCK_CALL*)(const SelectionQuery*, const SelectionReply*, ExtractedReference*, void*) noexcept;

    /*
     * Every callback is optional. update runs once per ROCK physics frame,
     * preCollide on each Havok substep before collision, retire when the world
     * goes away or the extension is removed: remove everything the extension
     * holds in that world there.
     *
     * One extension at a time drives weapons (weaponUpdate; then
     * weaponPreCollide and weaponRetire are required). weaponPreCollide
     * returns nonzero to keep driving this substep. weaponRetire runs before
     * ROCK retires the slot's weapon body: remove every constraint on it.
     * extractSelection requires classifySelection and validateSelection.
     * The first extension that handles a selection, proximity or impact query
     * answers it. A faulting callback removes its extension.
     */
    struct Callbacks
    {
        std::uint32_t size{ sizeof(Callbacks) };
        std::uint32_t version{ 1 };
        void* context{ nullptr };
        void(ROCK_CALL* update)(const Frame*, void*) noexcept {};
        void(ROCK_CALL* preCollide)(const Frame*, void*) noexcept {};
        void(ROCK_CALL* retire)(const Frame*, void*) noexcept {};
        void(ROCK_CALL* weaponUpdate)(const WeaponFrame*, WeaponCommand*, void*) noexcept {};
        std::uint32_t(ROCK_CALL* weaponPreCollide)(const Frame*, std::uint32_t weaponSlot, void*) noexcept {};
        void(ROCK_CALL* weaponContact)(const Contact*, void*) noexcept {};
        void(ROCK_CALL* weaponRetire)(const Frame*, std::uint32_t weaponSlot, void*) noexcept {};
        void(ROCK_CALL* classifySelection)(const SelectionQuery*, SelectionReply*, void*) noexcept {};
        ExtractionCallback extractSelection{};
        std::uint32_t(ROCK_CALL* validateSelection)(const SelectionQuery*, const SelectionReply*, void*) noexcept {};
        void(ROCK_CALL* proximityTarget)(const ProximityQuery*, ProximityResult*, void*) noexcept {};
        void(ROCK_CALL* impactEffect)(const ImpactQuery*, ImpactResult*, void*) noexcept {};
    };

    /*
     * register/unregister: game thread, outside callbacks (a Core callback
     * may register before the first frame); unregister runs retire.
     * The other calls are valid only inside the owner's callbacks:
     * setDynamicObstacleLayers and the contact exclusion calls in Update or
     * WeaponUpdate; activateWeapon also in PreCollide; copyWeaponSources and
     * getSourceGeometry in WeaponUpdate of that slot. Otherwise WrongThread.
     * Weapon calls take weaponSlot < kMaxWeaponSlots (InvalidArgument
     * otherwise) and are the weapon driver's; ROCK drives slot 0 only yet, so
     * slot 1 returns TargetUnavailable and clearing it is a no-op.
     */
    struct Api
    {
        static constexpr InterfaceId interfaceId = kInterfaceId;
        static constexpr std::uint32_t majorVersion = kMajor;
        static constexpr std::uint32_t minorVersion = kMinor;
        Status(ROCK_CALL* registerExtension)(OwnerToken owner, const Callbacks* callbacks) noexcept;
        Status(ROCK_CALL* unregisterExtension)(OwnerToken owner) noexcept;
        // Which passive layers (33 and kPassiveObstacleLayer) ROCK's dynamic
        // hand and weapon bodies collide with, as a layer bit mask.
        Status(ROCK_CALL* setDynamicObstacleLayers)(OwnerToken owner, std::uint64_t layerMask) noexcept;
        Status(ROCK_CALL* activateWeapon)(OwnerToken owner, std::uint32_t weaponSlot) noexcept;
        // One target body the slot's weapon body stops colliding with.
        Status(ROCK_CALL* setWeaponContactExclusion)(OwnerToken owner, std::uint32_t weaponSlot, const BodyIdentity* target) noexcept;
        Status(ROCK_CALL* clearWeaponContactExclusion)(OwnerToken owner, std::uint32_t weaponSlot) noexcept;
        Status(ROCK_CALL* hasWeaponContactExclusion)(OwnerToken owner, std::uint32_t weaponSlot, const BodyIdentity* target, std::uint32_t* outExcluded) noexcept;
        // Reports the total in outCount and copies min(capacity, total).
        Status(ROCK_CALL* copyWeaponSources)(OwnerToken owner, std::uint32_t weaponSlot, WeaponSource* outSources, std::uint32_t capacity, std::uint32_t* outCount) noexcept;
        Status(ROCK_CALL* getSourceGeometry)(OwnerToken owner, std::uint32_t weaponSlot, std::uint32_t body, const Point3* direction, SourceGeometry* outGeometry) noexcept;
    };
    static_assert(std::is_standard_layout_v<Api> && sizeof(Api) == 9 * sizeof(void*));
    static_assert(std::is_trivially_copyable_v<Callbacks> && std::is_trivially_copyable_v<BodyIdentity>);
}
#pragma pack(pop)
