#include "physics-interaction/weapon/NativePrimaryWeaponGraph.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/native/NativeTriggerEquipGraph.h"
#include "physics-interaction/PhysicsLog.h"
#include "rock_support/Fo4VrRuntime.h"
#include "RE/Bethesda/PlayerCharacter.h"
#include "RE/Bethesda/TESBoundObjects.h"
#include <REL/Relocation.h>
#include <array>

namespace rock::native_primary_weapon_graph
{
    namespace
    {
        bool s_installed = false;
        bool gun(const RE::BGSObjectInstance& item) noexcept
        {
            if (!item.object || !item.object->IsWeapon()) return false;
            const auto* weapon = static_cast<const RE::TESObjectWEAP*>(item.object);
            const auto* data = item.instanceData ? static_cast<const RE::TESObjectWEAP::InstanceData*>(item.instanceData.get()) : &weapon->weaponData;
            return data->type == RE::WEAPON_TYPE::kGun;
        }
        template<std::size_t N>
        bool matches(std::uintptr_t rva, const std::array<std::uint8_t, N>& expected) noexcept
        {
            std::array<std::uint8_t, N> actual{};
            return native_memory::guardedCopyFromMemory(reinterpret_cast<void*>(REL::Offset(rva).address()), actual.data(), N) && actual == expected;
        }
    }

    bool collect(RE::Actor* actor, RE::BSScrapArray<RE::IKeywordFormBase*>* output)
    {
        if (!s_installed || !actor || actor != f4vr::getPlayer() || !output) return false;
        using Read = RE::BGSObjectInstance* (*)(RE::Actor*, RE::BGSObjectInstance*, std::uint32_t);
        const auto read = reinterpret_cast<Read>(REL::Offset(0xE50DA0).address());
        // Native retained index lookup is also safe during preload jobs. Never
        // borrow ROCK's frame view or the cached animation-equipment array.
        RE::BGSObjectInstance left{nullptr, nullptr};
        read(actor, &left, 1);
        if (!gun(left)) return false;
        RE::BGSObjectInstance right{nullptr, nullptr};
        read(actor, &right, 0);
        if (right.object) {
            using Add = void (*)(RE::BGSObjectInstance*, RE::BSScrapArray<RE::IKeywordFormBase*>*);
            reinterpret_cast<Add>(REL::Offset(0xEDA2F0).address())(&right, output);
        }
        // With only a secondary gun the primary weapon role has no item.
        // Loading the left graph here would give its parts two evaluators.
        try {
            ROCK_LOG_SAMPLE_INFO(Animation, 1000,
                "AKIMBO primary graph selection right={:08X} instance={:#x} left={:08X} keywords={} scope=all-native-collectors",
                right.object ? right.object->formID : 0, reinterpret_cast<std::uintptr_t>(right.instanceData.get()),
                left.object->formID, output->size());
        } catch (...) { /* Logging cannot interrupt a native preload job. */ }
        return true;
    }

    bool install() noexcept
    {
        if (s_installed) return true;
        if (!REL::Module::IsVR() || REL::Module::get().version() != F4SE::RUNTIME_VR_1_2_72) return false;
        native_trigger_equip_graph::install();
        if (!native_trigger_equip_graph::installed() ||
            !matches(0xE50DA0, std::to_array<std::uint8_t>({0x40,0x53,0x48,0x83,0xEC,0x20,0x48,0x8B,0xDA})) ||
            !matches(0xEDA2F0, std::to_array<std::uint8_t>({0x48,0x89,0x5C,0x24,0x08,0x57,0x48,0x83,0xEC,0x30}))) {
            ROCK_LOG_ERROR(Init, "Primary graph selection unavailable: shared collector or native instance lookup invalid");
            return false;
        }
        // ED93E0 serves EB7840, EB81A0, C9650, C98D0 and ECEB70.
        // A single EB78B5 redirect missed native preloading AND comparison,
        // allowing a mixed left/right graph to replace the isolated request.
        s_installed = true;
        ROCK_LOG_INFO(Init, "Native primary firearm graph isolation active for loading, preloading and comparison");
        return true;
    }
}
