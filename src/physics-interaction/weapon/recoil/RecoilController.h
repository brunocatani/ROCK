#pragma once

#include "physics-interaction/weapon/recoil/RecoilProfiles.h"
#include "physics-interaction/weapon/recoil/RecoilMath.h"

namespace rock::weapon_recoil_policy
{
    // Matches the API role bits; physical left/right are resolved per sample.
    enum class HandMask : std::uint32_t { None = 0, Primary = 1, Offhand = 2 };

    [[nodiscard]] inline constexpr HandMask deliveryHand(
        const bool ownedCarry, const bool firingHandIsLeft,
        const bool nativePrimaryIsLeft) noexcept
    {
        // Owned carry/solver publishes already-recoiled weapon AND hand targets.
        // FRIK must not add a second kick to those targets next skeleton frame.
        return ownedCarry ? HandMask::None :
               firingHandIsLeft == nativePrimaryIsLeft ? HandMask::Primary : HandMask::Offhand;
    }

    [[nodiscard]] inline constexpr bool needsOneHandPresentation(
        const bool kickActive, const bool needsNeutralFrame) noexcept
    {
        return kickActive || needsNeutralFrame;
    }

    [[nodiscard]] inline constexpr bool canPresentRightRecoilForGrip(
        const bool gripping, const bool closeSupportAssist,
        const bool transferredPrimaryGrip) noexcept
    {
        // Close support follows the firing-hand aim and must share its recoil.
        // Full two-hand and transferred grips already have their own solve.
        return !gripping || (closeSupportAssist && !transferredPrimaryGrip);
    }

    struct SampleIdentity
    {
        std::uint32_t formID{ 0 };
        Family family{ Family::Default };
        float familyPercent{ 100.0f };
        std::uintptr_t weaponNode{ 0 };  // Identity only; never dereferenced.
        std::uint64_t weaponGeneration{ 0 };
        std::uint64_t equippedOwnership{ 0 };
        Profile profile{ Profile::OneHand };
        bool firingHandIsLeft{ false };
        bool nativePrimaryIsLeft{ false };
        bool fullTwoHanded{ false };
        bool oneHanded{ false };

        [[nodiscard]] bool operator==(const SampleIdentity&) const = default;
    };

    // One callback ticket per ROCK update. State/role changes cannot replay a
    // kick captured for another owner, profile, native hand mapping, or solve.
    struct SampleTicket
    {
        SampleIdentity identity{};
        std::uint64_t sequence{ 0 };
        std::uint64_t observed{ 0 };
        bool valid{ false };
        bool ready{ false };

        void invalidate() noexcept { valid = false; ready = false; }
        void beginUpdate(const bool enabled) noexcept
        {
            if (!enabled) {
                invalidate();
            }
            ready = valid && sequence != observed;
            observed = sequence;
        }
        [[nodiscard]] bool consume(const SampleIdentity& current) noexcept
        {
            const bool accepted = ready && valid && identity == current;
            ready = false;
            return accepted;
        }
    };
}

