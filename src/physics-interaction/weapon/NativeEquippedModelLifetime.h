#pragma once

namespace rock::native_equipped_model_lifetime
{
    [[nodiscard]] bool install() noexcept;
    // UI/native transaction -> frame publication, no borrowed pointers.
    void noteEquipmentChange() noexcept;
    // Main FRIK frame only. Completes a pending, drawn inventory handoff once
    // native hide ownership has ended; never clears an active native hide lease.
    void updateVisibility() noexcept;
    // The player's native weapon hide-owner flags (process middle-high
    // +4A0); zero means no native owner hides the weapons, 0xFF unreadable.
    [[nodiscard]] std::uint8_t playerWeaponHideOwnerFlags() noexcept;

    /*
     * Open while ROCK removes one of two worn weapons. The equip executor's
     * reason-2 weapon hide then runs after the removed weapon has already
     * left the equipment list, so it would blank only the survivor.
     */
    class SurvivorVisibleScope
    {
    public:
        SurvivorVisibleScope() noexcept;
        ~SurvivorVisibleScope();
        SurvivorVisibleScope(const SurvivorVisibleScope&) = delete;
        SurvivorVisibleScope& operator=(const SurvivorVisibleScope&) = delete;
    };
}
