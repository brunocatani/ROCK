#pragma once

#include "WeaponSemanticTypes.h"

#include <cstdint>

namespace frik::rock
{
    /*
     * ROCK needs vanilla reload compatibility without making every weapon a
     * hand-authored patch. Virtual Reloads proved the useful pieces are the
     * vanilla weapon/ammo lifecycle, ammo count transitions, and per-weapon
     * node ownership. ROCK keeps that idea but separates the native event
     * boundary from the physical reload coordinator so unverified hooks cannot
     * leak into gameplay state.
     */

    enum class WeaponVanillaReloadStage : std::uint8_t
    {
        Idle,
        ReloadRequested,
        VanillaReloadStarted,
        AmmoDetachWindow,
        AmmoCommitted,
        ActionWindow,
        Completing,
        Complete,
        Canceled,
        UnsafeUnknown
    };

    enum class WeaponReloadStageSource : std::uint8_t
    {
        None,
        NativeReloadEvent,
        NativeAmmoCountEvent,
        PollingFallback,
        ConfigFallback
    };

    struct WeaponReloadNativeSnapshot
    {
        bool weaponEquipped{ false };
        bool reloadEventReceived{ false };
        bool reloadEventValue{ false };
        bool ammoEventReceived{ false };
        std::uint32_t clipAmmo{ 0 };
        std::uint32_t reserveAmmo{ 0 };
        std::uint32_t reloadSequence{ 0 };
        std::uint32_t ammoSequence{ 0 };
        std::uint32_t sequence{ 0 };
    };

    struct WeaponReloadObserverState
    {
        WeaponVanillaReloadStage stage{ WeaponVanillaReloadStage::Idle };
        WeaponReloadStageSource source{ WeaponReloadStageSource::None };
        bool active{ false };
        std::uint32_t lastClipAmmo{ 0 };
        std::uint32_t lastReserveAmmo{ 0 };
        std::uint32_t lastSequence{ 0 };
    };

    struct WeaponReloadObserverOutput
    {
        WeaponVanillaReloadStage stage{ WeaponVanillaReloadStage::Idle };
        WeaponReloadStageSource source{ WeaponReloadStageSource::None };
        std::uint32_t clipAmmo{ 0 };
        std::uint32_t reserveAmmo{ 0 };
        std::uint32_t sequence{ 0 };
        bool stageChanged{ false };
    };

    inline WeaponReloadObserverOutput advanceWeaponReloadStageObserver(WeaponReloadObserverState& state, const WeaponReloadNativeSnapshot& snapshot)
    {
        const auto previousStage = state.stage;
        WeaponReloadStageSource source = WeaponReloadStageSource::None;

        auto processReloadEvent = [&]() {
            source = WeaponReloadStageSource::NativeReloadEvent;
            state.active = snapshot.reloadEventValue;
            state.stage = snapshot.reloadEventValue ? WeaponVanillaReloadStage::VanillaReloadStarted : WeaponVanillaReloadStage::Complete;
        };

        auto processAmmoEvent = [&]() {
            source = WeaponReloadStageSource::NativeAmmoCountEvent;
            state.lastClipAmmo = snapshot.clipAmmo;
            state.lastReserveAmmo = snapshot.reserveAmmo;
            state.stage = state.active ? WeaponVanillaReloadStage::AmmoCommitted : WeaponVanillaReloadStage::Idle;
        };

        if (!snapshot.weaponEquipped) {
            state.stage = state.active ? WeaponVanillaReloadStage::Canceled : WeaponVanillaReloadStage::Idle;
            state.active = false;
        } else if (snapshot.reloadEventReceived && snapshot.ammoEventReceived) {
            const bool hasOrderedSequences = snapshot.reloadSequence != 0 && snapshot.ammoSequence != 0;
            const bool ammoAfterReload = !hasOrderedSequences || snapshot.ammoSequence >= snapshot.reloadSequence;
            if (ammoAfterReload) {
                processReloadEvent();
                if (snapshot.reloadEventValue) {
                    processAmmoEvent();
                }
            } else {
                processAmmoEvent();
                processReloadEvent();
            }
        } else if (snapshot.reloadEventReceived) {
            processReloadEvent();
        } else if (snapshot.ammoEventReceived) {
            processAmmoEvent();
        }

        if (source != WeaponReloadStageSource::None) {
            state.source = source;
        }
        state.lastSequence = snapshot.sequence;

        return WeaponReloadObserverOutput{
            .stage = state.stage,
            .source = source == WeaponReloadStageSource::None ? state.source : source,
            .clipAmmo = state.lastClipAmmo,
            .reserveAmmo = state.lastReserveAmmo,
            .sequence = snapshot.sequence,
            .stageChanged = previousStage != state.stage,
        };
    }

    struct WeaponReloadCoordinatorState
    {
        WeaponReloadRuntimeState runtime{};
        WeaponVanillaReloadStage vanillaStage{ WeaponVanillaReloadStage::Idle };
        WeaponReloadStageSource source{ WeaponReloadStageSource::None };
        std::uint32_t lastSequence{ 0 };
        bool physicalAmmoInserted{ false };
    };

    struct WeaponReloadCoordinatorOutput
    {
        WeaponReloadRuntimeState runtime{};
        WeaponVanillaReloadStage vanillaStage{ WeaponVanillaReloadStage::Idle };
        WeaponReloadStageSource source{ WeaponReloadStageSource::None };
        bool stateChanged{ false };
        bool vanillaCompletionNeedsPhysicalGate{ false };
    };

    inline bool shouldRequireWeaponReloadPhysicalCompletion(bool configuredRequirePhysicalCompletion, bool allowFallbackWhenUnsupported, bool physicalCompletionSupported)
    {
        if (!configuredRequirePhysicalCompletion) {
            return false;
        }
        return physicalCompletionSupported || !allowFallbackWhenUnsupported;
    }

    inline bool shouldFallbackCompleteStaleReload(
        bool runtimeActive, bool requirePhysicalCompletion, bool allowFallbackWhenUnsupported, std::uint32_t staleFrames, std::uint32_t timeoutFrames)
    {
        if (!runtimeActive || requirePhysicalCompletion || !allowFallbackWhenUnsupported || timeoutFrames == 0) {
            return false;
        }
        return staleFrames >= timeoutFrames;
    }

    inline WeaponReloadCoordinatorOutput advanceWeaponReloadCoordinator(
        WeaponReloadCoordinatorState& state, const WeaponReloadObserverOutput& observed, bool physicalAmmoInserted, bool requirePhysicalCompletion = true)
    {
        /*
         * FO4VR can expose reload start and ammo-count commit without a distinct
         * later completion event on every weapon path. When ROCK has not yet
         * enabled a verified physical insertion gate, ammo commit must be allowed
         * to complete the coordinator; otherwise support grip and left-hand
         * weapon interaction stay disabled after a normal vanilla reload.
         */
        const auto previousState = state.runtime.state;

        state.vanillaStage = observed.stage;
        state.source = observed.source;
        state.lastSequence = observed.sequence;
        state.physicalAmmoInserted = state.physicalAmmoInserted || physicalAmmoInserted;

        switch (observed.stage) {
        case WeaponVanillaReloadStage::ReloadRequested:
        case WeaponVanillaReloadStage::VanillaReloadStarted:
        case WeaponVanillaReloadStage::AmmoDetachWindow:
            state.runtime.state = WeaponReloadState::ReloadRequested;
            state.runtime.supportGripAllowed = false;
            break;
        case WeaponVanillaReloadStage::AmmoCommitted:
            if (!requirePhysicalCompletion && state.physicalAmmoInserted) {
                state.runtime.state = WeaponReloadState::Idle;
                state.runtime.supportGripAllowed = true;
                state.physicalAmmoInserted = false;
                break;
            }
            state.runtime.state = state.physicalAmmoInserted ? WeaponReloadState::AmmoInserted : WeaponReloadState::WeaponUnloaded;
            state.runtime.supportGripAllowed = false;
            break;
        case WeaponVanillaReloadStage::ActionWindow:
            state.runtime.state = WeaponReloadState::ActionRequired;
            state.runtime.supportGripAllowed = false;
            break;
        case WeaponVanillaReloadStage::Completing:
            state.runtime.state = WeaponReloadState::Completing;
            state.runtime.supportGripAllowed = false;
            break;
        case WeaponVanillaReloadStage::Complete:
            if (requirePhysicalCompletion && !state.physicalAmmoInserted && state.runtime.isReloadActive()) {
                state.runtime.supportGripAllowed = false;
                break;
            }
            state.runtime.state = WeaponReloadState::Idle;
            state.runtime.supportGripAllowed = true;
            state.physicalAmmoInserted = false;
            break;
        case WeaponVanillaReloadStage::Canceled:
        case WeaponVanillaReloadStage::UnsafeUnknown:
            state.runtime.state = WeaponReloadState::Canceled;
            state.runtime.supportGripAllowed = true;
            state.physicalAmmoInserted = false;
            break;
        case WeaponVanillaReloadStage::Idle:
        default:
            state.runtime.state = WeaponReloadState::Idle;
            state.runtime.supportGripAllowed = true;
            state.physicalAmmoInserted = false;
            break;
        }

        return WeaponReloadCoordinatorOutput{
            .runtime = state.runtime,
            .vanillaStage = state.vanillaStage,
            .source = state.source,
            .stateChanged = previousState != state.runtime.state,
            .vanillaCompletionNeedsPhysicalGate = requirePhysicalCompletion && state.runtime.isReloadActive() && !state.physicalAmmoInserted,
        };
    }
}
