#include "physics-interaction/weapon/recoil/RecoilRecovery.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/PhysicsLog.h"
#include "rock_support/Fo4VrRuntime.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace rock::recoil_recovery
{
    namespace
    {
        template <std::size_t Size>
        bool matchesCode(const std::uintptr_t rva, const std::array<std::uint8_t, Size>& expected)
        {
            std::array<std::uint8_t, Size> actual{};
            const auto address = REL::Module::get().base() + rva;
            return native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(address), actual.data(), actual.size()) &&
                actual == expected;
        }

        bool verifyRuntimeLayout()
        {
            // FO4VR 1.2.72: EC2E30/EC3AB0 allocate a private 0xA0 AimModel
            // into EquippedWeaponData+20; EC2FD0 frees it. 834D30 copies the
            // 0x40-byte AMDL data through 2CAC20 and stores its Actor at +88.
            // 834DF0 reads +18 (times +1C when sighted), writes spring+64,
            // then drives the +54 recovery state toward zero through 81D280.
            // These live witnesses guard the new write, not an engine hook.
            static_assert(offsetof(RE::EquippedWeaponData, aimModel) == 0x20);
            static_assert(offsetof(RE::AimModel, aimModelData) == 0);
            static_assert(offsetof(RE::AimModel, actor) == 0x88);
            static_assert(offsetof(RE::BGSAimModel::Data, aimModelRecoilDiminishSpringForce) == 0x18);
            const bool valid =
                matchesCode(0xEC2FE5, std::array<std::uint8_t, 4>{ 0x48, 0x8B, 0x79, 0x20 }) &&
                matchesCode(0x834DAA, std::array<std::uint8_t, 7>{ 0x48, 0x89, 0x9F, 0x88, 0, 0, 0 }) &&
                matchesCode(0x834F69, std::array<std::uint8_t, 10>{ 0xF3, 0x0F, 0x59, 0x4F, 0x18, 0xF3, 0x0F, 0x11, 0x4F, 0x64 });
            if (!valid) ROCK_LOG_ERROR(Weapon, "Recoil recovery: VR layout witnesses differ; correction disabled");
            return valid;
        }
    }

    void repairEquippedWeapon() noexcept
    {
        auto* item = fo4vr::getEquippedWeaponItem();
        if (!item || !item->data) return;
        const auto* weapon = static_cast<const RE::TESObjectWEAP*>(item->item.object);
        if (weapon->weaponData.type.get() != RE::WEAPON_TYPE::kGun) return;
        static const bool layoutVerified = verifyRuntimeLayout();
        if (!layoutVerified) return;

        const auto reject = [weapon](const char* stage) {
            ROCK_LOG_SAMPLE_WARN(Weapon, 2000, "Recoil recovery: form={:08X} rejected at {}", weapon->formID, stage);
        };
        auto* data = static_cast<RE::EquippedWeaponData*>(item->data.get());
        std::uintptr_t vtable{};
        if (!native_memory::tryReadValue(reinterpret_cast<const std::uintptr_t*>(data), vtable) ||
            vtable != fo4vr::EquippedWeaponData_vtable.address()) {
            reject("equipped-data-type");
            return;
        }
        RE::AimModel* aim{};
        if (!native_memory::tryReadValue(&data->aimModel, aim)) {
            reject("aim-model-pointer");
            return;
        }
        if (!aim) return;
        RE::Actor* owner{};
        if (!native_memory::tryReadValue(&aim->actor, owner) || owner != fo4vr::getPlayer()) {
            reject("aim-model-player-owner");
            return;
        }
        float spring{};
        auto* field = &aim->aimModelData.aimModelRecoilDiminishSpringForce;
        if (!native_memory::tryReadValue(field, spring)) {
            reject("recovery-spring-read");
            return;
        }
        if (!repairZeroSpring(spring)) return;
        // Correct this player's live copy only. No AMDL form, OMOD, save data,
        // or NPC is changed; re-equip/load creates a fresh copy checked here.
        if (!native_memory::tryWriteValue(field, spring)) {
            reject("recovery-spring-write");
            return;
        }
        ROCK_LOG_SAMPLE_INFO(Weapon, 1000,
            "Recoil recovery: form={:08X} equipped runtime spring corrected 0 -> {}", weapon->formID, spring);
    }
}
