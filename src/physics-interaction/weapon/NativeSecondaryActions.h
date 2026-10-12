#pragma once

namespace RE { class NiNode; class NiTransform; }
namespace rock::native_secondary_actions
{
    bool install() noexcept;
    // All lifecycle and action calls are on the existing interaction frame owner.
    void update(RE::NiNode* attachment, float deltaSeconds) noexcept;
    void present(RE::NiNode* attachment) noexcept;
    void carryPresentation(const RE::NiTransform& previous,const RE::NiTransform& target) noexcept;
    void suspend() noexcept;
    // Called by the existing physical A/X arbiter, after UI suppression.
    bool requestReload() noexcept;
    // Uses the secondary item's native recoil and the existing physical-hand
    // recoil math. Does not change FRIK's primary recoil sample or ownership.
    void applyRecoil(RE::NiTransform& weapon, RE::NiTransform& hand, const RE::NiTransform& controller) noexcept;
}
