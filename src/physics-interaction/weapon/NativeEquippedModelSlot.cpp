#include "physics-interaction/weapon/NativeEquippedModelSlot.h"
#include "physics-interaction/weapon/NativeEquippedModelSlotHook.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/PhysicsLog.h"
#include "RE/Bethesda/Actor.h"
#include "RE/Bethesda/PlayerCharacter.h"
#include "RE/Bethesda/TESBoundObjects.h"
#include "RE/Bethesda/TESObjectREFRs.h"
#include <array>
#include <atomic>
#include <cstring>
#include <memory>
#include <xbyak/xbyak.h>

namespace rock::native_equipped_model_slot
{
    namespace
    {
        // One atomic value makes every native callback observe a complete
        // mapping. Form identity is a value; no game pointer is retained here.
        std::atomic<std::uint64_t> mapping{};
        bool installed{};
        using ModelSlot = std::uint32_t (*)(RE::Actor*, RE::TESForm*, std::uint32_t);
        using RaceSlot = std::uint32_t (*)(RE::Actor*);

        bool mappedPlayer(RE::Actor* actor, std::uint64_t value) noexcept
        {
            return value && actor && actor == RE::PlayerCharacter::GetSingleton();
        }

        std::uint32_t comparisonSlot(RE::Actor* actor, std::uint32_t candidate) noexcept
        {
            const auto value = mapping.load(std::memory_order_acquire);
            if (mappedPlayer(actor, value) && candidate == static_cast<std::uint32_t>(value)) return candidate;
            return reinterpret_cast<RaceSlot>(REL::Offset(0x3DE1A0).address())(actor);
        }

        std::uint32_t cleanupSlot(RE::Actor* actor, const void* record, const RE::BipedAnim* biped) noexcept
        {
            const auto value = mapping.load(std::memory_order_acquire);
            const auto slot = static_cast<std::uint32_t>(value);
            if (mappedPlayer(actor, value) && biped && slot >= 33 && slot <= 39 &&
                (record == &biped->object[slot] || record == &biped->bufferedObjects[slot])) return slot;
            return reinterpret_cast<RaceSlot>(REL::Offset(0x3DE1A0).address())(actor);
        }

        struct Site
        {
            std::uintptr_t rva;
            Arguments arguments;
            std::array<std::uint8_t, 21> bytes;
        };

        // Complete raw callers, including their register assignments, are
        // archived with the native-equipped investigation. Preserve the race
        // slot for armor/light callers: a global GetShieldSlot override would
        // change worn armor and torch behavior.
        constexpr std::array sites{
            Site{0x1C805F, Arguments::EquipmentIndex,{0x24,0x60,0x48,0x8B,0x17,0x48,0x8B,0xCB,0xE8,0x5C,0x2F,0x00,0x00,0x83,0xF8,0xFF,0x0F,0x84,0xAA,0x00,0x00}},
            Site{0x1C81DF, Arguments::EquipmentIndex,{0x4C,0x89,0xAC,0x24,0x88,0x01,0x00,0x00,0xE8,0xDC,0x2D,0x00,0x00,0x4C,0x63,0xE8,0x41,0x83,0xCC,0xFF,0x45}},
            Site{0xF0B41C, Arguments::EquipmentIndex,{0x24,0x50,0x48,0x8B,0x16,0x48,0x8B,0xCB,0xE8,0x9F,0xFB,0x2B,0xFF,0x45,0x0F,0xB6,0xC6,0x48,0x8B,0xCD,0x8B}},
            Site{0xDC82F5, Arguments::EquipmentIndex,{0x00,0x00,0x48,0x8B,0xCB,0x4C,0x8B,0xF0,0xE8,0xC6,0x2C,0x40,0xFF,0x49,0x83,0x3E,0x00,0x8B,0xF8,0x74,0x22}},
            Site{0x1CAA79, Arguments::Attach,{0x83,0xFE,0xFF,0x74,0x66,0x48,0x8B,0xCF,0xE8,0x22,0x37,0x21,0x00,0x3B,0xF0,0x75,0x5A,0x48,0x8B,0x03,0x48}},
            Site{0x1C7208, Arguments::Rebuild,{0xE8,0x2B,0xE6,0xFF,0xFF,0x48,0x8B,0xCB,0xE8,0x93,0x6F,0x21,0x00,0x4D,0x8B,0x54,0x36,0x10,0x44,0x3B,0xF8}},
            Site{0x1CB023, Arguments::Classification,{0x8B,0xF9,0x48,0x8B,0xCA,0x41,0x8B,0xD9,0xE8,0x78,0x31,0x21,0x00,0x33,0xD2,0x3B,0xD8,0x48,0x8B,0x5C,0x24}},
            Site{0x1CB4C8, Arguments::BodyFilter,{0x48,0x85,0xDB,0x74,0x1C,0x48,0x8B,0xCB,0xE8,0xD3,0x2C,0x21,0x00,0x83,0xFF,0xFF,0x74,0x0F,0x3B,0xF8,0x75}},
            Site{0x1C61BD, Arguments::Cleanup,{0x84,0xAE,0x01,0x00,0x00,0x48,0x8B,0xCB,0xE8,0xDE,0x7F,0x21,0x00,0x48,0x8B,0x4D,0x7F,0x48,0x85,0xC9,0x0F}},
        };

        bool vacant(const RE::BipedAnim* biped, std::uint32_t slot) noexcept
        {
            if (!biped) return true;
            const auto& live = biped->object[slot];
            const auto& buffered = biped->bufferedObjects[slot];
            return !live.parent.object && !live.partClone && !buffered.parent.object && !buffered.partClone;
        }
    }

    bool install() noexcept try
    {
        if (installed) return true;
        if (!REL::Module::IsVR() || REL::Module::get().version() != F4SE::RUNTIME_VR_1_2_72) return false;
        std::array<std::unique_ptr<Stub>, sites.size()> stubs;
        std::size_t required = sites.size() * 16;
        for (std::size_t i = 0; i < sites.size(); ++i) {
            const auto& site = sites[i];
            std::array<std::uint8_t, 21> bytes{};
            if (!native_memory::guardedCopyFromMemory(reinterpret_cast<void*>(REL::Offset(site.rva - 8).address()),
                    bytes.data(), bytes.size()) || bytes != site.bytes) {
                ROCK_LOG_ERROR(Init, "Native equipped model slot unavailable: caller contract rva={:X}", site.rva);
                return false;
            }
            stubs[i] = std::make_unique<Stub>(site.arguments, reinterpret_cast<std::uintptr_t>(&resolve),
                reinterpret_cast<std::uintptr_t>(&comparisonSlot), reinterpret_cast<std::uintptr_t>(&cleanupSlot));
            required += stubs[i]->getSize();
        }
        auto& trampoline = F4SE::GetTrampoline();
        if (trampoline.free_size() < required) {
            ROCK_LOG_ERROR(Init, "Native equipped model slot unavailable: trampoline needs {} bytes, has {}", required, trampoline.free_size());
            return false;
        }
        // Warm the singleton relocation before any native callback can run.
        (void)RE::PlayerCharacter::GetSingleton();
        for (std::size_t i = 0; i < sites.size(); ++i) {
            auto* code = trampoline.allocate(stubs[i]->getSize());
            std::memcpy(code, stubs[i]->getCode(), stubs[i]->getSize());
            FlushInstructionCache(GetCurrentProcess(), code, stubs[i]->getSize());
            trampoline.write_call<5>(REL::Offset(sites[i].rva).address(), reinterpret_cast<std::uintptr_t>(code));
        }
        installed = true;
        return true;
    }
    catch (...) {
        try { ROCK_LOG_ERROR(Init, "Native equipped model slot installation failed; second-slot admission disabled"); } catch (...) {}
        return false;
    }

    std::uint32_t resolve(RE::Actor* actor, RE::TESForm* weapon, std::uint32_t index) noexcept
    {
        const auto value = mapping.load(std::memory_order_acquire);
        if (index == 1 && mappedPlayer(actor, value) && (!weapon || weapon->formID == static_cast<std::uint32_t>(value >> 32)))
            return static_cast<std::uint32_t>(value);
        return reinterpret_cast<ModelSlot>(REL::Offset(0x1CAFC0).address())(actor, weapon, index);
    }

    bool reserve(RE::TESForm* weapon) noexcept
    {
        auto* player = RE::PlayerCharacter::GetSingleton();
        if (!installed || !player || !weapon || weapon->formType != RE::ENUM_FORM_ID::kWEAP) return false;
        const auto current = mapping.load(std::memory_order_acquire);
        if (current) return static_cast<std::uint32_t>(current >> 32) == weapon->formID;
        // Native weapon slots, not armor bits or the shared shield/torch slot.
        // Never evict an existing biped record to make room for this feature.
        for (std::uint32_t slot = 33; slot <= 39; ++slot) {
            if (!vacant(player->firstPersonBipedAnim.get(), slot) || !vacant(player->biped.get(), slot)) continue;
            mapping.store((std::uint64_t{weapon->formID} << 32) | slot, std::memory_order_release);
            ROCK_LOG_INFO(Weapon, "Native akimbo model slot reserved form={:08X} equipment=1 biped={}", weapon->formID, slot);
            return true;
        }
        ROCK_LOG_WARN(Weapon, "Native akimbo admission declined form={:08X}: native weapon model slots are occupied", weapon->formID);
        return false;
    }

    bool releaseIfUnused() noexcept
    {
        const auto value = mapping.load(std::memory_order_acquire);
        if (!value) return true;
        auto* player = RE::PlayerCharacter::GetSingleton();
        if (!player) return false;
        auto* middle = player->currentProcess ? player->currentProcess->middleHigh : nullptr;
        if (middle) {
            if (middle->equippedItems.size() > 16) return false;
            for (const auto& item : middle->equippedItems) if (item.equipIndex.index == 1 && item.item.object) return false;
        }
        const auto slot = static_cast<std::uint32_t>(value);
        if (!vacant(player->firstPersonBipedAnim.get(), slot) || !vacant(player->biped.get(), slot)) return false;
        mapping.store(0, std::memory_order_release);
        return true;
    }

    void resetForGameLoad() noexcept { mapping.store(0, std::memory_order_release); }
}
