#include "physics-interaction/weapon/NativeEquippedWeaponAttach.h"

#include "physics-interaction/native/HavokOffsets.h"
#include "physics-interaction/native/NativeMemory.h"
#include "rock_support/Fo4VrRuntime.h"

#include "RE/Bethesda/Actor.h"
#include "RE/Bethesda/TESObjectREFRs.h"

#include <array>
#include <cstring>

namespace rock::native_equipped_weapon_attach
{
    namespace
    {
        constexpr std::array<std::uint8_t, 16> kExpectedAttachEntry{
            0x48, 0x89, 0x5C, 0x24, 0x08, 0x48, 0x89, 0x6C,
            0x24, 0x10, 0x48, 0x89, 0x74, 0x24, 0x18, 0x57,
        };

        [[nodiscard]] bool attachEntryMatchesVerifiedRuntime() noexcept
        {
            static const bool matches = []() noexcept {
                if (!REL::Module::IsVR() ||
                    REL::Module::get().version() != F4SE::RUNTIME_VR_1_2_72) {
                    return false;
                }

                std::array<std::uint8_t, kExpectedAttachEntry.size()> actual{};
                const auto address = REL::Offset(offsets::kFunc_QueueEquippedWeaponAttach).address();
                return native_memory::guardedCopyFromMemory(
                           reinterpret_cast<const void*>(address),
                           actual.data(),
                           actual.size()) &&
                       actual == kExpectedAttachEntry;
            }();
            return matches;
        }
    }

    SubmitResult submitExactCurrent(const Identity& expected) noexcept
    {
        if (!REL::Module::IsVR() ||
            REL::Module::get().version() != F4SE::RUNTIME_VR_1_2_72) {
            return SubmitResult::UnsupportedRuntime;
        }
        if (!attachEntryMatchesVerifiedRuntime()) {
            return SubmitResult::ContractMismatch;
        }

        auto* player = f4vr::getPlayer();
        if (!player) {
            return SubmitResult::MissingPlayer;
        }

        auto* equipped = f4vr::getEquippedItem();
        auto* object = equipped ? equipped->item.object : nullptr;
        auto* instanceData = equipped ? equipped->item.instanceData.get() : nullptr;
        if (!object || object->formType != RE::ENUM_FORM_ID::kWEAP) {
            return SubmitResult::MissingEquippedWeapon;
        }

        const auto currentEquipIndex = equipped->equipIndex.index;
        if (object->formID != expected.formID ||
            reinterpret_cast<std::uintptr_t>(instanceData) != expected.instanceData ||
            currentEquipIndex != expected.equipIndex) {
            return SubmitResult::IdentityChanged;
        }

        void* manager = nullptr;
        const auto managerAddress = REL::Offset(offsets::kData_EquippedWeaponAttachManager).address();
        if (!native_memory::tryReadValue(
                reinterpret_cast<void* const*>(managerAddress),
                manager) ||
            !manager) {
            return SubmitResult::MissingManager;
        }

        RE::BGSObjectInstance exactInstance(object, instanceData);
        using QueueAttach = void (*)(
            void*,
            RE::PlayerCharacter*,
            RE::BGSObjectInstance*,
            std::uint32_t);
        const auto queueAttach = reinterpret_cast<QueueAttach>(
            REL::Offset(offsets::kFunc_QueueEquippedWeaponAttach).address());
        queueAttach(manager, player, &exactInstance, currentEquipIndex);
        return SubmitResult::Submitted;
    }

    const char* submitResultName(const SubmitResult result) noexcept
    {
        switch (result) {
        case SubmitResult::Submitted:
            return "submitted";
        case SubmitResult::UnsupportedRuntime:
            return "unsupported-runtime";
        case SubmitResult::ContractMismatch:
            return "contract-mismatch";
        case SubmitResult::MissingPlayer:
            return "missing-player";
        case SubmitResult::MissingEquippedWeapon:
            return "missing-equipped-weapon";
        case SubmitResult::IdentityChanged:
            return "identity-changed";
        case SubmitResult::MissingManager:
            return "missing-manager";
        default:
            return "unknown";
        }
    }
}
