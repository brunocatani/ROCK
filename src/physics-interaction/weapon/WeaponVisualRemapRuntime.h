#pragma once

/*
 * ROCK repairs a stale first-person weapon attach tree by queueing FO4VR's
 * narrow native weapon attach task only after the current equipped instance
 * matches ROCK's pending form, instance-data, and object-instance-extra
 * witness. This deliberately avoids broad equip, animation reload, or
 * mod-mutation paths: collision generation stays blocked until the visible
 * source tree actually changes, while the native task gets one chance per
 * pending instance witness to rebuild the tree the game left one transaction
 * behind.
 */

#include <cstdint>

namespace rock::weapon_visual_remap_runtime
{
    enum class RequestResult : std::uint8_t
    {
        Queued,
        Disabled,
        MissingPendingInstance,
        MissingPlayer,
        MissingCurrentProcess,
        MissingMiddleHighProcess,
        MissingEquippedWeapon,
        MissingTaskQueue,
        InvalidEquippedWeapon,
        MismatchedEquippedWeapon,
    };

    struct RequestInput
    {
        bool enabled = false;
        std::uint64_t pendingInstanceSignature = 0;
        std::uint32_t expectedWeaponFormID = 0;
        std::uintptr_t expectedWeaponFormAddress = 0;
        std::uintptr_t expectedInstanceDataAddress = 0;
        std::uintptr_t expectedObjectInstanceExtraAddress = 0;
        const char* reason = "";
    };

    struct RequestOutcome
    {
        RequestResult result = RequestResult::Disabled;
        std::uint64_t pendingInstanceSignature = 0;
        std::uint32_t weaponFormID = 0;
        std::uintptr_t weaponFormAddress = 0;
        std::uintptr_t instanceDataAddress = 0;
        std::uintptr_t objectInstanceExtraAddress = 0;
        std::uint32_t equipIndex = 0;
        std::uint32_t sourceSlotIndex = 0;
        const char* source = "";
        const char* detail = "";

        [[nodiscard]] bool queued() const noexcept { return result == RequestResult::Queued; }
    };

    [[nodiscard]] const char* resultName(RequestResult result) noexcept;
    [[nodiscard]] RequestOutcome requestCurrentFirstPersonWeaponVisualRemap(const RequestInput& input);
}
