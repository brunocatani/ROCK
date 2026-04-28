#pragma once

#include <cstdint>
#include <string>

namespace frik::rock
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
        RemoveMagazine,
        SocketInsert,
        ManipulateAction,
        PassiveTouch
    };

    enum class WeaponReloadState : std::uint8_t
    {
        Idle,
        ReloadRequested,
        WeaponOpened,
        WeaponUnloaded,
        PouchAvailable,
        AmmoHeld,
        AmmoAligned,
        AmmoInserted,
        ActionRequired,
        ActionManipulating,
        Completing,
        Canceled
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
        WeaponGripPoseId gripPose{ WeaponGripPoseId::None };
        std::uint32_t bodyId{ 0x7FFF'FFFF };
    };

    struct WeaponReloadRuntimeState
    {
        WeaponReloadState state{ WeaponReloadState::Idle };
        bool supportGripAllowed{ true };

        bool isReloadActive() const
        {
            return state != WeaponReloadState::Idle && state != WeaponReloadState::Canceled && state != WeaponReloadState::Completing;
        }
    };
}
