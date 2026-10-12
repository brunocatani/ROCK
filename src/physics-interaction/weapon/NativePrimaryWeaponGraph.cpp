#include "physics-interaction/weapon/NativePrimaryWeaponGraph.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/PhysicsLog.h"
#include "rock_support/Fo4VrRuntime.h"
#include "RE/Bethesda/PlayerCharacter.h"
#include "RE/Bethesda/TESBoundObjects.h"
#include <REL/Relocation.h>
#include <Windows.h>
#include <array>
#include <atomic>
#include <cstring>

namespace rock::native_primary_weapon_graph
{
    namespace
    {
        using Keywords = RE::BSScrapArray<RE::IKeywordFormBase*>;
        using Collect = void (*)(RE::Actor*, Keywords*);
        using ReadWeapon = RE::BGSObjectInstance* (*)(RE::Actor*, RE::BGSObjectInstance*, std::uint32_t);
        using AddItem = void (*)(RE::BGSObjectInstance*, Keywords*);
        Collect s_collect = nullptr;
        ReadWeapon s_readWeapon = nullptr;
        AddItem s_addItem = nullptr;
        bool s_installed = false;
        std::atomic<std::uint32_t> s_diagnostics{0};

        bool gun(const RE::BGSObjectInstance& item) noexcept
        {
            if (!item.object || !item.object->IsWeapon()) return false;
            const auto* weapon = static_cast<const RE::TESObjectWEAP*>(item.object);
            const auto* data = item.instanceData ? static_cast<const RE::TESObjectWEAP::InstanceData*>(item.instanceData.get()) : &weapon->weaponData;
            return data->type == RE::WEAPON_TYPE::kGun;
        }

        void collect(RE::Actor* actor, Keywords* output) noexcept
        {
            if (!actor || actor != f4vr::getPlayer() || !output) { s_collect(actor, output); return; }
            // E50DA0 resolves by the native equipment INDEX and returns retained
            // instance data. No frame-owned array is read on an animation task.
            RE::BGSObjectInstance left{nullptr, nullptr};
            s_readWeapon(actor, &left, 1);
            if (!gun(left)) { s_collect(actor, output); return; }
            RE::BGSObjectInstance right{nullptr, nullptr};
            s_readWeapon(actor, &right, 0);
            if (!gun(right)) { s_collect(actor, output); return; }
            // ED93E0/EDE310 collect every flagged equipped item's keywords into
            // ONE role-1 query. A second gun may consequently replace the first
            // gun's action subgraph. This callsite belongs to that primary query;
            // the native selector still resolves variants/mod keywords itself.
            s_addItem(&right, output);
            if (s_diagnostics.fetch_add(1, std::memory_order_relaxed) < 32) {
                try {
                    ROCK_LOG_INFO(Animation,
                        "AKIMBO primary graph targets right={:08X} instance={:#x} left={:08X} keywords={} scope=native-weapon-role",
                        right.object->formID, reinterpret_cast<std::uintptr_t>(right.instanceData.get()), left.object->formID, output->size());
                } catch (...) { OutputDebugStringA("ROCK: primary graph target diagnostic failed\n"); }
            }
        }
        template<std::size_t N> bool matches(std::uintptr_t rva, const std::array<std::uint8_t, N>& expected) noexcept
        {
            std::array<std::uint8_t, N> actual{};
            return native_memory::guardedCopyFromMemory(reinterpret_cast<void*>(REL::Offset(rva).address()), actual.data(), N) && actual == expected;
        }
    }

    bool install() noexcept
    {
        if (s_installed) return true;
        if (!REL::Module::IsVR() || REL::Module::get().version() != F4SE::RUNTIME_VR_1_2_72) return false;
        constexpr std::array<std::uint8_t, 5> original{0xE8,0x26,0x1B,0x02,0};
        if (!matches(0xEB78B5, original) ||
            !matches(0xEB78DC, std::array<std::uint8_t,8>{0xC7,0x44,0x24,0x40,0x01,0,0,0}) ||
            !matches(0xE50DA0, std::array<std::uint8_t,9>{0x40,0x53,0x48,0x83,0xEC,0x20,0x48,0x8B,0xDA}) ||
            !matches(0xEDA2F0, std::array<std::uint8_t,13>{0x48,0x89,0x5C,0x24,0x08,0x57,0x48,0x83,0xEC,0x30,0x48,0x8B,0xD9})) {
            ROCK_LOG_ERROR(Init, "Primary weapon graph target validation failed; keyword selection unchanged");
            return false;
        }
        s_collect = reinterpret_cast<Collect>(REL::Offset(0xED93E0).address());
        s_readWeapon = reinterpret_cast<ReadWeapon>(REL::Offset(0xE50DA0).address());
        s_addItem = reinterpret_cast<AddItem>(REL::Offset(0xEDA2F0).address());
        try {
            auto& trampoline = F4SE::GetTrampoline();
            if (trampoline.free_size() < 32) return false;
            const auto address = REL::Offset(0xEB78B5).address();
            (void)trampoline.write_call<5>(address, &collect);
            std::array<std::uint8_t, 5> call{};
            std::array<std::uint8_t, 14> relay{};
            std::int32_t displacement = 0;
            std::uintptr_t destination = 0;
            bool installed = native_memory::guardedCopyFromMemory(reinterpret_cast<void*>(address), call.data(), call.size()) && call[0] == 0xE8;
            std::memcpy(&displacement, call.data() + 1, 4);
            installed = installed && native_memory::guardedCopyFromMemory(reinterpret_cast<void*>(address + 5 + displacement), relay.data(), relay.size());
            std::memcpy(&destination, relay.data() + 6, 8);
            constexpr std::array<std::uint8_t, 6> jump{0xFF,0x25,0,0,0,0};
            if (!installed || std::memcmp(relay.data(), jump.data(), jump.size()) != 0 ||
                destination != reinterpret_cast<std::uintptr_t>(&collect) || !FlushInstructionCache(GetCurrentProcess(), nullptr, 0)) {
                REL::safe_write(address, original.data(), original.size());
                FlushInstructionCache(GetCurrentProcess(), nullptr, 0);
                ROCK_LOG_ERROR(Init, "Primary weapon graph redirect failed; original call restored");
                return false;
            }
            s_installed = true;
            ROCK_LOG_INFO(Init, "Native primary firearm graph target isolation installed");
            return true;
        } catch (...) {
            REL::safe_write(REL::Offset(0xEB78B5).address(), original.data(), original.size());
            FlushInstructionCache(GetCurrentProcess(), nullptr, 0);
            ROCK_LOG_ERROR(Init, "Primary weapon graph installation failed; original call restored");
            return false;
        }
    }
}
