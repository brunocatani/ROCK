#include "physics-interaction/weapon/KnifeReverseGrip.h"

#include "physics-interaction/hand/HandFrame.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/TransformMath.h"

#include "RE/Bethesda/TESBoundObjects.h"

#include <array>
#include <atomic>

namespace rock::knife_reverse_grip
{
    namespace
    {
        struct HandState
        {
            std::uint32_t formID{ 0 };
            bool reversed{ false };
        };

        // Frame thread only: written by the grab input pass, read by the
        // equipped weapon placements later in the same frame.
        std::array<HandState, 2> s_hands{};

        // form << 1 | reversed, plus kPresent. The saved snapshot is written by
        // the frame thread on every change; the pending restore by the
        // save-load callbacks, consumed by the frame thread.
        constexpr std::uint64_t kPresent = 1ull << 40;
        std::array<std::atomic<std::uint64_t>, 2> s_saved{};
        std::array<std::atomic<std::uint64_t>, 2> s_pending{};

        constexpr std::uint64_t pack(const std::uint32_t formID, const bool reversed) noexcept
        {
            return kPresent | (std::uint64_t{ formID } << 1) | (reversed ? 1u : 0u);
        }

        void publish(const bool isLeft, const HandState& state) noexcept
        {
            s_saved[isLeft ? 1u : 0u].store(state.formID ? pack(state.formID, state.reversed) : 0, std::memory_order_relaxed);
        }

        HandState& handState(const bool isLeft) noexcept
        {
            return s_hands[isLeft ? 1u : 0u];
        }
    }

    bool appliesTo(const RE::TESObjectWEAP* weapon) noexcept
    {
        // Automatic melee (the Ripper) keeps its native trigger attack.
        if (!weapon || weapon->weaponData.flags.any(RE::WEAPON_FLAGS::kAutomatic)) return false;
        const auto type = weapon->weaponData.type.get();
        return type == RE::WEAPON_TYPE::kOneHandSword || type == RE::WEAPON_TYPE::kOneHandDagger ||
               type == RE::WEAPON_TYPE::kOneHandAxe || type == RE::WEAPON_TYPE::kOneHandMace;
    }

    void observeHand(const bool isLeft, const std::uint32_t knifeFormID, const std::uint32_t wornForm0,
        const std::uint32_t wornForm1, const bool triggerPressed) noexcept
    {
        auto& state = handState(isLeft);
        const auto hand = isLeft ? "left" : "right";
        // A menu or detached grip reports no knife for a while; the grip is
        // kept until the weapon itself leaves the hand's equipment.
        if (state.formID && state.formID != wornForm0 && state.formID != wornForm1) {
            if (state.reversed) ROCK_LOG_INFO(Weapon, "Knife grip normal hand={} weapon={:08X}: unequipped", hand, state.formID);
            state = {};
            publish(isLeft, state);
        }
        if (knifeFormID && state.formID != knifeFormID) {
            state = { .formID = knifeFormID, .reversed = false };
            publish(isLeft, state);
        }
        // A loaded save's grip, the first time the hand carries a knife.
        if (const auto pending = knifeFormID ? s_pending[isLeft ? 1u : 0u].exchange(0, std::memory_order_relaxed) : 0;
            pending && static_cast<std::uint32_t>((pending & ~kPresent) >> 1) == knifeFormID) {
            state.reversed = (pending & 1u) != 0;
            publish(isLeft, state);
            ROCK_LOG_INFO(Weapon, "Knife grip {} hand={} weapon={:08X}: saved", state.reversed ? "reversed" : "normal", hand,
                knifeFormID);
        }
        if (!knifeFormID || !triggerPressed) {
            return;
        }
        state.reversed = !state.reversed;
        publish(isLeft, state);
        ROCK_LOG_INFO(Weapon, "Knife grip {} hand={} weapon={:08X}", state.reversed ? "reversed" : "normal", hand, knifeFormID);
    }

    SavedGrip savedGrip(const bool isLeft) noexcept
    {
        const auto value = s_saved[isLeft ? 1u : 0u].load(std::memory_order_relaxed);
        if (!value) return {};
        return { static_cast<std::uint32_t>((value & ~kPresent) >> 1), (value & 1u) != 0 };
    }

    void restoreGrip(const bool isLeft, const SavedGrip grip) noexcept
    {
        s_pending[isLeft ? 1u : 0u].store(grip.formID ? pack(grip.formID, grip.reversed) : 0, std::memory_order_relaxed);
    }

    bool reversed(const bool isLeft, const std::uint32_t formID) noexcept
    {
        const auto& state = handState(isLeft);
        return formID != 0 && state.formID == formID && state.reversed;
    }

    RE::NiTransform reverseSeat(const RE::NiTransform& seat, const bool isLeft, const float flatPlaneZ) noexcept
    {
        const auto palm = computeGrabPalmPointWorldFromHandBasis(seat, isLeft);
        // Half turn about weapon X through (any, palm.y, flatPlaneZ).
        auto flip = transform_math::makeIdentityTransform<RE::NiTransform>();
        flip.rotate.entry[1][1] = -1.0f;
        flip.rotate.entry[2][2] = -1.0f;
        flip.translate = { 0.0f, 2.0f * palm.y, 2.0f * flatPlaneZ };
        return transform_math::composeTransforms(flip, seat);
    }

    RE::NiTransform presentedSeat(const RE::NiTransform& seat, const bool isLeft, const std::uint32_t formID,
        const float flatPlaneZ) noexcept
    {
        return reversed(isLeft, formID) ? reverseSeat(seat, isLeft, flatPlaneZ) : seat;
    }
}
