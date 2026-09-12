#pragma once

#include "physics-interaction/body/BodyZone.h"
#include "physics-interaction/timing/GameFrameTimingPolicy.h"

#include <algorithm>
#include <cstdint>
#include <string_view>

namespace rock::immersive_aid
{
    enum class Injector : std::uint8_t { None, Stimpak, MedX, Psycho, Serum };

    // Explicit Fallout4.esm ALCH identities. Syringer ammunition, inhalers,
    // oral chems, and unknown forms never acquire an injection gesture.
    [[nodiscard]] constexpr Injector classify(std::string_view plugin, std::uint32_t localFormId)
    {
        if (plugin != "Fallout4.esm") {
            return Injector::None;
        }
        switch (localFormId) {
        case 0x00023736: // Stimpak
        case 0x0019C0D9: // CurieHealthpak
        case 0x00055F10: // MS19Cure
            return Injector::Stimpak;
        case 0x00033779: // MedX
        case 0x00058AA7: // Calmex
            return Injector::MedX;
        case 0x0003377D: // Psycho
        case 0x00058AAC: // Psychobuff
        case 0x00058AAA: // Psychotats
        case 0x00058AA8: // PsychoJet
            return Injector::Psycho;
        case 0x000EC4F7: // MS09LorenzoSerum
            return Injector::Serum;
        default:
            return Injector::None;
        }
    }

    [[nodiscard]] constexpr bool usesStimpakPose(Injector injector)
    {
        return injector == Injector::Stimpak || injector == Injector::MedX;
    }

    [[nodiscard]] constexpr bool eligibleBodyZone(body_zone::BodyZoneKind zone, bool leftHand)
    {
        return zone != body_zone::BodyZoneKind::Unknown &&
            zone != (leftHand ? body_zone::BodyZoneKind::LeftHand : body_zone::BodyZoneKind::RightHand);
    }

    inline constexpr double kContactSeconds = 0.5;

    struct ContactState
    {
        std::uint64_t grabIdentity = 0;
        std::uint64_t collisionGeneration = 0;
        std::uint64_t lastSequence = 0;
        double dwellSeconds = 0.0;
        bool touching = false;
        bool committed = false;
    };

    // Only consecutive, observed gameplay frames count. A pause, hitch, lost
    // body provider, new grab, or rebuilt collider set starts a fresh gesture.
    [[nodiscard]] inline bool advanceContact(ContactState& state, bool touching,
        std::uint64_t grabIdentity, std::uint64_t collisionGeneration,
        const game_frame_timing_policy::GameFrameTiming& timing)
    {
        if (!grabIdentity || !touching || !timing.valid || timing.menuPaused || timing.discontinuity ||
            !std::isfinite(timing.deltaSeconds) || timing.deltaSeconds <= 0.0f ||
            timing.deltaSeconds > game_frame_timing_policy::kMaxOrdinaryDeltaSeconds) {
            state = {};
            return false;
        }
        if (state.grabIdentity != grabIdentity || state.collisionGeneration != collisionGeneration ||
            timing.sequence < state.lastSequence) {
            state = {};
            state.grabIdentity = grabIdentity;
            state.collisionGeneration = collisionGeneration;
        }
        if (timing.sequence == state.lastSequence) {
            return false;
        }
        const bool consecutive = state.touching && timing.sequence == state.lastSequence + 1;
        state.lastSequence = timing.sequence;
        state.dwellSeconds = consecutive ? state.dwellSeconds + timing.deltaSeconds : 0.0;
        state.touching = true;
        if (!state.committed && state.dwellSeconds >= kContactSeconds) {
            state.committed = true;
            return true;
        }
        return false;
    }
}
