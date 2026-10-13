#include "physics-interaction/native/NativeTriggerEquipGraph.h"
#include "physics-interaction/native/EntryTrampolineHook.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/weapon/NativeIdleGripPreharvest.h"
#include "physics-interaction/weapon/NativePrimaryWeaponGraph.h"
#include "rock_support/Fo4VrRuntime.h"
#include "RE/Bethesda/Actor.h"
#include "RE/Bethesda/PlayerCharacter.h"
#include "RE/Bethesda/TESBoundObjects.h"
#include <REL/Relocation.h>
#include <array>

namespace rock::native_trigger_equip_graph
{
    namespace
    {
        using Keywords = RE::BSScrapArray<RE::IKeywordFormBase*>;
        using Collect = std::uintptr_t (*)(RE::PlayerCharacter*, Keywords*);
        Collect s_collect{};
        bool s_ready{};
        struct Scope
        {
            RE::PlayerCharacter* player;
            RE::BGSObjectInstance* object;
        };
        thread_local const Scope* t_scope{};
        class ScopedKeywords
        {
        public:
            explicit ScopedKeywords(const Scope& scope) : _previous(t_scope) { t_scope = &scope; }
            ~ScopedKeywords() { t_scope = _previous; }
            ScopedKeywords(const ScopedKeywords&) = delete;
            ScopedKeywords& operator=(const ScopedKeywords&) = delete;
        private:
            const Scope* _previous;
        };

        std::uintptr_t collect(RE::PlayerCharacter* player, Keywords* output)
        {
            if (t_scope && t_scope->player == player && output) {
                // ED93E0 prefers animation equipment cached in AIProcess over
                // the newly equipped item. Supply the committed exact instance
                // only for this synchronous native weapon-subgraph operation.
                // EDA2F0 is also used by native idle-grip preharvest.
                reinterpret_cast<void (*)(RE::BGSObjectInstance*, Keywords*)>(
                    REL::Offset(0xEDA2F0).address())(t_scope->object, output);
                return 1;
            }
            if (native_primary_weapon_graph::collect(player, output)) return 1;
            return s_collect(player, output);
        }

        template <std::size_t N>
        bool matches(std::uintptr_t rva, const std::array<std::uint8_t, N>& expected)
        {
            std::array<std::uint8_t, N> actual{};
            return native_memory::guardedCopyFromMemory(reinterpret_cast<void*>(REL::Offset(rva).address()),
                actual.data(), actual.size()) && actual == expected;
        }
    }

    void install() noexcept
    {
        if (s_ready || !REL::Module::IsVR() || REL::Module::get().version() != F4SE::RUNTIME_VR_1_2_72) return;
        if (!matches(0xE8D090, std::array<std::uint8_t, 10>{0x48,0x89,0x5C,0x24,0x10,0x48,0x89,0x74,0x24,0x18}) ||
            !matches(0xE15B50, std::array<std::uint8_t, 9>{0x40,0x55,0x53,0x57,0x48,0x8D,0x6C,0x24,0xB9}) ||
            !matches(0xEB81A0, std::array<std::uint8_t, 7>{0x48,0x8B,0xC4,0x48,0x89,0x50,0x10}) ||
            !matches(0xEDA2F0, std::array<std::uint8_t, 10>{0x48,0x89,0x5C,0x24,0x08,0x57,0x48,0x83,0xEC,0x30})) {
            ROCK_LOG_ERROR(Init, "Trigger equip native graph preparation unavailable: entry mismatch");
            return;
        }
        constexpr std::array<std::uint8_t, 14> prefix{
            0x48,0x89,0x5C,0x24,0x10,0x48,0x89,0x6C,0x24,0x18,0x56,0x57,0x41,0x54};
        void* original{};
        s_ready = entry_trampoline_hook::install("Trigger equip exact graph keywords", 0xED93E0,
            prefix.data(), prefix.size(), reinterpret_cast<void*>(&collect), original);
        s_collect = reinterpret_cast<Collect>(original);
        s_ready = s_ready && s_collect;
    }

    bool installed() noexcept { return s_ready; }

    Result prepare(RE::PlayerCharacter* player) noexcept
    {
        auto* equipped = f4vr::getEquippedWeaponItem();
        auto* object = equipped ? equipped->item.object : nullptr;
        if (!player || player != f4vr::getPlayer() || !object || object->formType != RE::ENUM_FORM_ID::kWEAP) return Result::Unavailable;
        auto* weapon = static_cast<RE::TESObjectWEAP*>(object);
        auto* instance = equipped->item.instanceData.get();
        // This path is exclusively for firearms. Melee equipment keeps its
        // native draw ordering and does not wait on firearm graph evidence.
        auto* weaponData = instance ? static_cast<RE::TESObjectWEAP::InstanceData*>(instance) : &weapon->weaponData;
        if (weaponData->type != RE::WEAPON_TYPE::kGun) return Result::Ready;
        if (!s_ready || t_scope) return Result::Unavailable;
        void* process{};
        void* middle{};
        void* handles{};
        if (!native_memory::tryReadField(player, 0x300, process) || !process ||
            !native_memory::tryReadField(process, 8, middle) || !middle ||
            !native_memory::tryReadField(middle, 0x2C0, handles) || !handles) return Result::Unavailable;
        RE::BGSObjectInstance exact(object, instance);
        const Scope scope{player, &exact};
        const ScopedKeywords useExact(scope);
        // Native DCF5E0 uses this request/update order. E8D090 stages new
        // handles; E15B50 checks loading and commits them through E8D190.
        // Do not move handles ourselves or submit draw against a pending set.
        const bool requested = reinterpret_cast<bool (*)(void*, RE::PlayerCharacter*)>(
            REL::Offset(0xE8D090).address())(process, player);
        // Native pending handle array is at handles+60 (current at +40).
        // Record exact first-person resource reuse, not merely a same-form hit.
        std::uint64_t firstPersonHandle{};
        std::uint32_t capacity{}, count{};
        void* currentMiddle{};
        void* currentHandles{};
        if (native_memory::tryReadField(process, 8, currentMiddle) && currentMiddle &&
            native_memory::tryReadField(currentMiddle, 0x2C0, currentHandles) && currentHandles) {
            auto* array = static_cast<char*>(currentHandles) + (requested ? 0x60 : 0x40);
            if (native_memory::tryReadField(array, 0, capacity) && native_memory::tryReadField(array, 0x18, count) &&
                count >= 2 && count <= 8) {
                void* data = array + 8;
                if ((capacity & 0x80000000u) != 0 || native_memory::tryReadField(array, 8, data)) {
                    (void)native_memory::tryReadField(data, 8, firstPersonHandle);
                }
            }
        }
        const bool warmResource = native_idle_grip_preharvest::isSubgraphWarm(firstPersonHandle);
        reinterpret_cast<void (*)(RE::PlayerCharacter*, bool)>(REL::Offset(0xE15B50).address())(player, !requested);
        // Native action/event callbacks may replace process storage. Reacquire
        // it before inspecting the post-commit set; keep no borrowed AIProcess
        // storage pointer across those callbacks.
        if (!native_memory::tryReadField(player, 0x300, process) || !process ||
            !native_memory::tryReadField(process, 8, middle) || !middle) return Result::Unavailable;
        const bool mismatch = reinterpret_cast<bool (*)(RE::PlayerCharacter*, void*)>(
            REL::Offset(0xEB81A0).address())(player, static_cast<char*>(middle) + 0x318);
        std::uint32_t pending{};
        if (!native_memory::tryReadField(middle, 0x350, pending)) return Result::Unavailable;
        ROCK_LOG_SAMPLE_DEBUG(Weapon, 50,
            "Trigger equip graph preparation form={:08X} instance={} requested={} pending={} currentMismatch={} warmResource={} firstPersonHandle={:016X}",
            object->formID, static_cast<void*>(instance), requested, pending, mismatch, warmResource, firstPersonHandle);
        return !mismatch && pending == 0 ? Result::Ready : Result::Pending;
    }
}
