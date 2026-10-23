#include "physics-interaction/weapon/KnifeReverseGrip.h"

#include "physics-interaction/hand/HandFrame.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/TransformMath.h"

#include "RE/Bethesda/TESBoundObjects.h"

#include <array>

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

    void observeHand(const bool isLeft, const std::uint32_t knifeFormID, const bool triggerPressed) noexcept
    {
        auto& state = handState(isLeft);
        if (state.formID != knifeFormID) {
            state = { .formID = knifeFormID, .reversed = false };
        }
        if (!knifeFormID || !triggerPressed) {
            return;
        }
        state.reversed = !state.reversed;
        ROCK_LOG_INFO(Weapon, "Knife grip {} hand={} weapon={:08X}", state.reversed ? "reversed" : "normal",
            isLeft ? "left" : "right", knifeFormID);
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
