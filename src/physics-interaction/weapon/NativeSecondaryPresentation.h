#pragma once

namespace RE { class PlayerCharacter; class NiNode; }

namespace rock::native_secondary_presentation
{
    // Game-frame owner only. The native index-1 model is placed in the hand
    // that holds it, using its own authored firing seat; the primary weapon
    // keeps its existing owner.
    void beginFrame() noexcept;
    void update() noexcept;
    void afterWeaponPosition() noexcept;
    void afterWorldFinal() noexcept;
    // The return value belongs to this exact native call, including nested calls.
    // Only the secondary offset's arm pass leaves the left branch in world space.
    [[nodiscard]] bool beforeNativeArm(const RE::PlayerCharacter* player, const RE::NiNode* weapon, const RE::NiNode* offset) noexcept;
    void afterNativeArm(const RE::PlayerCharacter* player, bool secondaryPass) noexcept;
    void endFrame(bool skeletonUpdated) noexcept;
}
