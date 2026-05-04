#pragma once

/*
 * Weapon type declarations are grouped here so semantic IDs, evidence payloads, and hard limits stay in one stable public weapon surface without changing their namespaces.
 */


// ---- WeaponSemanticTypes.h ----

#include <cstdint>
#include <string>

namespace RE
{
    class NiAVObject;
}

namespace rock
{
    /*
     * Weapon collision hulls are now gameplay entities, not anonymous debug
     * shapes. HIGGS/FRIK treat the supported weapon as a coherent object with a
     * fixed primary grip and a semantic offhand/support point; ROCK needs the
     * same structure so contact routing, finger poses, reload sockets, and the
     * generated collision bodies all talk about the same weapon part.
     */

    enum class WeaponPartKind : std::uint8_t
    {
        Receiver,
        Barrel,
        Handguard,
        Foregrip,
        Pump,
        Stock,
        Grip,
        Magazine,
        Magwell,
        Bolt,
        Slide,
        ChargingHandle,
        BreakAction,
        Cylinder,
        Chamber,
        Shell,
        Round,
        LaserCell,
        Lever,
        Sight,
        Accessory,
        CosmeticAmmo,
        Other
    };

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
    };

    /*
     * Grab diagnostics intentionally live beside the semantic contact types
     * instead of inside the contact itself. HIGGS exposes grabbed node names as
     * diagnostic/API metadata, while its live interaction path stays focused on
     * physics state. ROCK follows that split so collision callbacks keep using
     * compact numeric data and the main-thread grab edge can still report the
     * weapon form, current weapon node, and generated NIF source that produced
     * a routed two-hand contact.
     */
    struct WeaponInteractionDebugInfo
    {
        std::string weaponName;
        std::uint32_t weaponFormId{ 0 };
        std::string weaponNodeName;
        std::string sourceName;
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
    };

    struct WeaponInteractionRuntimeState
    {
        bool supportGripAllowed{ true };
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
