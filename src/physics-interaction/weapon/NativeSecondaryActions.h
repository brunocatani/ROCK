#pragma once

#include "physics-interaction/weapon/EquippedAmmoPolicy.h"

#include <cstdint>

namespace RE { class NiNode; class NiTransform; }
namespace rock::native_secondary_actions
{
    bool install() noexcept;
    // True for ROCK's forwarder in the player's graph-manager holder slot.
    // Outside a second-weapon action it returns the player's own manager.
    [[nodiscard]] bool ownsPlayerGraphManagerSlot(std::uintptr_t function) noexcept;
    // All lifecycle and action calls are on the existing interaction frame owner.
    void update(RE::NiNode* attachment, float deltaSeconds) noexcept;
    void present(RE::NiNode* attachment) noexcept;
    void carryPresentation(const RE::NiTransform& previous,const RE::NiTransform& target) noexcept;
    void suspend() noexcept;
    // Promotion: the index-1 model leaves the private skeleton at its
    // rendered pose, for a carrier that moves only its root, and the private
    // graph retires.
    void handOffModelPose() noexcept;
    // Diagnostic: traces the next 3 s of a dispatched primary (index 0)
    // reload, which a restored two-weapon save plays without animation.
    void tracePrimaryReload() noexcept;
    // Called by the existing physical A/X arbiter, after UI suppression.
    bool requestReload() noexcept;
    // Uses the secondary item's native recoil and the existing physical-hand
    // recoil math. Does not change FRIK's primary recoil sample or ownership.
    void applyRecoil(RE::NiTransform& weapon, RE::NiTransform& hand, const RE::NiTransform& controller) noexcept;
    // The native ammo facts of equipment index 0 or 1 (EquippedAmmoPolicy.h),
    // read with the HUD producer's helpers. Game thread; false before install.
    bool readAmmo(std::uint32_t index, equipped_ammo::Facts& out) noexcept;
}
