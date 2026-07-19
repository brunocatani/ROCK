#pragma once

/*
 * Weapon type declarations are grouped here so semantic IDs, evidence payloads, and hard limits stay in one stable public weapon surface without changing their namespaces.
 */


// ---- WeaponSemanticTypes.h ----

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

#include "RE/NetImmerse/NiPoint.h"

namespace RE
{
    class NiAVObject;
}

namespace rock
{
    /*
     * Weapon collision hulls are now gameplay entities, not anonymous debug
     * shapes. ROCK treats the equipped weapon as one coherent object with a
     * fixed primary grip and a semantic offhand/support point so contact
     * routing, finger poses, reload sockets, and generated collision bodies all
     * talk about the same weapon part.
     */

    enum class WeaponPartKind : std::uint8_t
    {
        Receiver = 0,
        Barrel = 1,
        Handguard = 2,
        Foregrip = 3,
        Pump = 4,
        Stock = 5,
        Grip = 6,
        Magazine = 7,
        Magwell = 8,
        Bolt = 9,
        Slide = 10,
        ChargingHandle = 11,
        BreakAction = 12,
        Cylinder = 13,
        Chamber = 14,
        Shell = 15,
        Round = 16,
        LaserCell = 17,
        Lever = 18,
        Sight = 19,
        Accessory = 20,
        CosmeticAmmo = 21,
        Other = 22,
        LaserSight = 23,
        Flashlight = 24,
        LaserFlashlightCombo = 25,
        Scope = 26,
        MuzzleDevice = 27,
        Bipod = 28,
        Count = 29,
    };

    /*
     * Coarse physical size class for the whole equipped weapon, distinct from
     * WeaponPartKind (which classifies individual nodes). Used to pick a
     * generated-collision max-distance-from-origin budget: legitimate long/heavy
     * weapons need a larger allowance than a pistol, so one flat radius cannot
     * safely reject misplaced/detached attachment geometry (e.g. laser or
     * holosight NiNodes authored off-mesh) on every weapon at once.
     */
    enum class WeaponSizeClass : std::uint8_t
    {
        Melee,
        Pistol,
        Rifle,
        Heavy
    };

    /*
     * Records which signal produced a WeaponSizeClass decision. RE::WEAPON_TYPE
     * collapses every firearm to kGun, and Fallout4.esm's WeaponType* keyword
     * convention is author-discretion on modded weapons (verified 2026-07-03:
     * of two installed Glock pistol mods, one tags WeaponTypePistol on every
     * weapon, the other tags none), so classification must degrade gracefully
     * rather than silently trusting an absent or partial signal.
     */
    enum class WeaponClassificationSource : std::uint8_t
    {
        None,
        Keyword,
        WeightFallback,
        Default
    };

    /*
     * One bit per Fallout4.esm WeaponType* keyword ROCK checks on the equipped
     * weapon's own form (verified directly against Fallout4.esm; no OMOD/template
     * indirection was observed for any sampled weapon). A bitmask instead of a
     * single enum value because vanilla weapons can legitimately carry more than
     * one bucket keyword at once (e.g. CombatShotgun carries both
     * WeaponTypeRifle, the grip/animation category, and WeaponTypeShotgun, the
     * specific family) - collapsing that to one value would silently discard the
     * more specific tag that a future reload/scope consumer would want.
     */
    enum class WeaponKeywordFlag : std::uint64_t
    {
        None = 0,
        Pistol = 1ull << 0,
        Rifle = 1ull << 1,
        Shotgun = 1ull << 2,
        AssaultRifle = 1ull << 3,
        Sniper = 1ull << 4,
        GaussRifle = 1ull << 5,
        LaserMusket = 1ull << 6,
        HeavyGun = 1ull << 7,
        HandToHand = 1ull << 8,
        Melee1H = 1ull << 9,
        Melee2H = 1ull << 10,
        Unarmed = 1ull << 11,
        Minigun = 1ull << 12,
        Fatman = 1ull << 13,
        MissileLauncher = 1ull << 14,
        GatlingLaser = 1ull << 15,
        Flamer = 1ull << 16,
        Cryolater = 1ull << 17,
        JunkJet = 1ull << 18,
        RailwayRifle = 1ull << 19,
        Broadsider = 1ull << 20,
        Syringer = 1ull << 21,
        FlareGun = 1ull << 22,
        GammaGun = 1ull << 23,
        AlienBlaster = 1ull << 24,
        Ripper = 1ull << 25,
        Shishkebab = 1ull << 26,
        Laser = 1ull << 27,
        Plasma = 1ull << 28,
        Ballistic = 1ull << 29,
        Thrown = 1ull << 30,
        Grenade = 1ull << 31,
        Mine = 1ull << 32,
        Explosive = 1ull << 33,
        Automatic = 1ull << 34,
    };

    [[nodiscard]] inline constexpr bool hasWeaponKeywordFlag(std::uint64_t flags, WeaponKeywordFlag flag)
    {
        return (flags & static_cast<std::uint64_t>(flag)) != 0;
    }

    enum class WeaponReloadRole : std::uint8_t
    {
        None,
        MagazineBody,
        AmmoPiece,
        CosmeticAmmo,
        Receiver
    };

    enum class WeaponSupportGripRole : std::uint8_t
    {
        None,
        SupportSurface,
        Foregrip,
        PumpGrip,
        MagwellHold,
        StockForward,
        ReceiverSupport
    };

    enum class WeaponSocketRole : std::uint8_t
    {
        None,
        Magwell,
        Chamber,
        Cylinder,
        LaserCell,
        LoadingGate
    };

    enum class WeaponActionRole : std::uint8_t
    {
        None,
        Bolt,
        Slide,
        ChargingHandle,
        Pump,
        BreakAction,
        Cylinder,
        Lever,
        Latch
    };

    enum class WeaponGripPoseId : std::uint8_t
    {
        None,
        BarrelWrap,
        HandguardClamp,
        VerticalForegrip,
        AngledForegrip,
        PumpGrip,
        MagwellHold,
        ReceiverSupport
    };

    enum class WeaponInteractionKind : std::uint8_t
    {
        None,
        SupportGrip,
        PassiveTouch
    };

    enum class WeaponInteractionAcquisitionSource : std::uint8_t
    {
        None,
        PhysicalContact,
        ProximityProbe,
    };

    /*
     * Which signal produced a part classification. NIF name tokens are author
     * discretion; slot/rig anchors are structural (connect points and engine
     * rig nodes the game itself depends on) and therefore outrank names.
     */
    enum class WeaponPartClassificationSource : std::uint8_t
    {
        NameToken = 0,
        SlotAnchor = 1,
        RigAnchor = 2,
        AttachmentEvidence = 3,
    };

    struct WeaponPartClassification
    {
        WeaponPartKind partKind{ WeaponPartKind::Other };
        WeaponReloadRole reloadRole{ WeaponReloadRole::None };
        WeaponSupportGripRole supportGripRole{ WeaponSupportGripRole::None };
        WeaponSocketRole socketRole{ WeaponSocketRole::None };
        WeaponActionRole actionRole{ WeaponActionRole::None };
        WeaponGripPoseId fallbackGripPose{ WeaponGripPoseId::None };
        std::uint8_t priority{ 10 };
        bool gameplayCritical{ false };
        bool cosmetic{ false };
        WeaponPartClassificationSource classificationSource{ WeaponPartClassificationSource::NameToken };
        /*
         * Vanilla attach-point keyword FormID of the owning slot when a slot
         * anchor classified this part; 0 otherwise (including mod-added attach
         * points, which cannot be paired to a keyword at runtime yet).
         */
        std::uint32_t attachPointFormId{ 0 };
    };

    struct WeaponInteractionContact
    {
        bool valid{ false };
        std::uint32_t bodyId{ 0x7FFF'FFFF };
        WeaponPartKind partKind{ WeaponPartKind::Other };
        WeaponReloadRole reloadRole{ WeaponReloadRole::None };
        WeaponSupportGripRole supportGripRole{ WeaponSupportGripRole::None };
        WeaponSocketRole socketRole{ WeaponSocketRole::None };
        WeaponActionRole actionRole{ WeaponActionRole::None };
        WeaponGripPoseId fallbackGripPose{ WeaponGripPoseId::None };
        bool reloadActionAuthority{ false };
        /*
         * Semantic generated colliders need two roots. interactionRoot is the
         * gameplay transform authority used by support-grip solvers and must
         * stay on the equipped weapon package root. sourceRoot is the authored
         * visual part that produced this contact and is retained for debug and
         * future profile binding. Mixing these is what lets child parts detach.
         */
        RE::NiAVObject* interactionRoot{ nullptr };
        RE::NiAVObject* sourceRoot{ nullptr };
        std::uint64_t weaponGenerationKey{ 0 };
        float probeDistanceGame{ 0.0f };
        std::uint32_t sequence{ 0 };
        WeaponInteractionAcquisitionSource acquisitionSource{ WeaponInteractionAcquisitionSource::None };
    };

    /*
     * Grab diagnostics intentionally live beside the semantic contact types
     * instead of inside the contact itself. ROCK keeps collision callbacks on
     * compact numeric data, while the main-thread grab edge reports the weapon
     * form, current weapon node, and generated NIF source that produced a
     * routed two-hand contact.
     */
    struct WeaponInteractionDebugInfo
    {
        std::string weaponName;
        std::uint32_t weaponFormId{ 0 };
        std::string weaponNodeName;
        std::string sourceName;
        std::string interactionRootName;
        std::string sourceRootName;
    };

    struct WeaponInteractionDecision
    {
        WeaponInteractionKind kind{ WeaponInteractionKind::None };
        WeaponPartKind partKind{ WeaponPartKind::Other };
        WeaponReloadRole reloadRole{ WeaponReloadRole::None };
        WeaponSocketRole socketRole{ WeaponSocketRole::None };
        WeaponActionRole actionRole{ WeaponActionRole::None };
        WeaponGripPoseId gripPose{ WeaponGripPoseId::None };
        bool reloadActionAuthority{ false };
        std::uint32_t bodyId{ 0x7FFF'FFFF };
        RE::NiAVObject* interactionRoot{ nullptr };
        RE::NiAVObject* sourceRoot{ nullptr };
        std::uint64_t weaponGenerationKey{ 0 };
        WeaponInteractionAcquisitionSource acquisitionSource{ WeaponInteractionAcquisitionSource::None };
    };

    inline constexpr std::size_t kWeaponProviderSourceNameCapacity = 64;

    struct WeaponProviderPartAuthority
    {
        bool active{ false };
        std::uint64_t ownerToken{ 0 };
        std::uint64_t weaponGenerationKey{ 0 };
        std::uint32_t bodyId{ 0x7FFF'FFFF };
        std::uintptr_t sourceRoot{ 0 };
        std::uint32_t partKind{ 0 };
        std::uint32_t reloadRole{ 0 };
        std::uint32_t supportRole{ 0 };
        std::uint32_t socketRole{ 0 };
        std::uint32_t actionRole{ 0 };
        std::uint32_t groupId{ 0 };
        std::uint32_t grabMode{ 0 };
        std::array<char, kWeaponProviderSourceNameCapacity> sourceName{};
    };

    struct WeaponInteractionRuntimeState
    {
        bool supportGripAllowed{ true };
        WeaponProviderPartAuthority providerPartAuthority{};
    };
}

// ---- WeaponCollisionEvidence.h ----


#include <cstdint>
#include <string>
#include <vector>

namespace rock
{
    /*
     * ROCK publishes equipped-weapon evidence for external systems without
     * owning any reload profile or authoring policy. PAPER consumes these POD-ish
     * descriptors to decide how reload bodies map onto the current weapon, while
     * ROCK keeps the evidence tied only to generated collision geometry and
     * semantic contact classification.
     */

    struct WeaponEvidencePoint3
    {
        float x{ 0.0f };
        float y{ 0.0f };
        float z{ 0.0f };
    };

    struct WeaponEvidenceBounds3
    {
        WeaponEvidencePoint3 min{};
        WeaponEvidencePoint3 max{};
        bool valid{ false };
    };

    struct WeaponCollisionProfileEvidenceDescriptor
    {
        bool valid{ false };
        std::uint32_t bodyId{ 0x7FFF'FFFF };
        std::uint64_t weaponGenerationKey{ 0 };
        std::uintptr_t sourceRootAddress{ 0 };
        std::uintptr_t geometryRootAddress{ 0 };
        std::string sourceRootPath;
        std::string geometryRootPath;
        std::string sourceRootName;
        std::string geometryRootName;
        std::string sourceName;
        WeaponPartClassification semantic{};
        WeaponEvidenceBounds3 localBoundsGame{};
        std::vector<WeaponEvidencePoint3> localMeshPointsGame{};
        std::uint32_t pointCount{ 0 };
        /*
         * FormID of the installed OMOD whose attach point owns this part's
         * slot anchor; 0 when the part was not slot-classified or no installed
         * mod matches the slot's attach-point keyword.
         */
        std::uint32_t omodFormId{ 0 };
    };

    inline constexpr std::size_t MAX_WEAPON_EMITTERS = 32;

    struct WeaponEmitterDescriptor
    {
        bool valid{ false };
        bool active{ false };
        bool visible{ false };
        bool effectStateKnown{ false };
        bool hasAddOnNodeValue{ false };
        std::uint32_t kind{ 0 };
        std::uint32_t source{ 0 };
        std::uint32_t transformPriority{ 0 };
        std::uint32_t addOnNodeValue{ 0 };
        std::uint32_t omodFormId{ 0 };
        std::uint32_t attachPointFormId{ 0 };
        std::uint64_t weaponGenerationKey{ 0 };
        /*
         * Frame-to-frame identity tokens only. Refresh compares them with
         * addresses encountered during a fresh bounded traversal and never
         * dereferences a retained address. Rebuild-time part classification
         * may dereference ownerRootAddress only from a just-collected local
         * snapshot while the same main-thread tree traversal is still active.
         */
        std::uintptr_t transformNodeAddress{ 0 };
        std::uintptr_t effectNodeAddress{ 0 };
        std::uintptr_t ownerRootAddress{ 0 };
        // Internal-only trust bit: ownerRootAddress came from an actual P-* or
        // rig anchor, rather than the candidate traversal root fallback.
        bool ownerRootStructural{ false };
        std::array<float, 9> rotate{};
        std::array<float, 3> translate{};
        float scale{ 1.0f };
        std::array<float, 3> forwardWeaponLocal{};
        std::array<char, 64> sourceName{};
    };

    struct WeaponEmitterSnapshot
    {
        std::array<WeaponEmitterDescriptor, MAX_WEAPON_EMITTERS> emitters{};
        std::size_t count{ 0 };
        std::uint64_t weaponGenerationKey{ 0 };
        std::uint64_t equippedWeaponKey{ 0 };
        std::uint64_t rootSetKey{ 0 };
        std::uintptr_t weaponRootAddress{ 0 };
    };

    inline WeaponEvidencePoint3 makeWeaponEvidencePoint(float x, float y, float z)
    {
        return WeaponEvidencePoint3{ .x = x, .y = y, .z = z };
    }
}

// ---- WeaponCollisionLimits.h ----

#include <cstddef>

namespace rock
{
    /*
     * Weapon mesh collision is generated as several convex hull bodies so long
     * guns can keep stock, receiver, barrel, magazine, and accessories separated
     * instead of collapsing into one broad hull. This cap is shared rather than a
     * private literal so collision creation, debug publishing, and tests stay on
     * the same body budget.
     */
    inline constexpr std::size_t MAX_WEAPON_COLLISION_BODIES = 100;
}
