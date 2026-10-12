#pragma once

namespace rock::native_equipped_model_lifetime
{
    [[nodiscard]] bool install() noexcept;
    // UI/native transaction -> frame publication, no borrowed pointers.
    void noteEquipmentChange() noexcept;
    // Main FRIK frame only. Completes a pending, drawn inventory handoff once
    // native hide ownership has ended; never clears an active native hide lease.
    void updateVisibility() noexcept;
}
