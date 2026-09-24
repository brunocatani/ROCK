#include "physics-interaction/weapon/NativeCarriedWeaponContext.h"
#include "physics-interaction/native/EntryTrampolineHook.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/PhysicsLog.h"
#include "RE/Bethesda/MemoryManager.h"

#include <array>
#include <memory>

namespace rock::native_carried_weapon_context
{
    namespace
    {
        using ReadItem = bool (*)(RE::AIProcess*, std::uint32_t, RE::EquippedItem*);
        using Construct = RE::EquippedWeaponData* (*)(void*, RE::Actor*, const RE::BGSObjectInstance*, std::uint32_t);
        ReadItem originalRead{};
        bool installed{};
        thread_local RE::AIProcess* activeProcess{};
        thread_local const RE::EquippedItem* activeItem{};

        bool read(RE::AIProcess* process, std::uint32_t index, RE::EquippedItem* output)
        {
            if (output) {
                if (const auto* item = current(process, index)) {
                    // E803D0 and 701E30 assign into an initialized output and
                    // retain/release both the instance and equipped-data refs.
                    *output = *item;
                    return true;
                }
            }
            return originalRead(process, index, output);
        }
    }

    bool install() noexcept try
    {
        if (installed) return true;
        constexpr std::array<std::uint8_t, 15> constructorBytes{
            0x48,0x89,0x5C,0x24,0x08,0x48,0x89,0x6C,0x24,0x10,0x44,0x89,0x4C,0x24,0x20};
        std::array<std::uint8_t, constructorBytes.size()> actual{};
        if (!native_memory::guardedCopyFromMemory(reinterpret_cast<void*>(REL::Offset(0xEC2E30).address()),
                actual.data(), actual.size()) || actual != constructorBytes) return false;
        constexpr std::array<std::uint8_t, 18> readBytes{
            0x48,0x89,0x5C,0x24,0x18,0x48,0x89,0x6C,0x24,0x20,0x89,0x54,0x24,0x10,0x56,0x57,0x41,0x56};
        void* original{};
        if (!entry_trampoline_hook::install("carried-weapon indexed context", 0xE803D0,
                readBytes.data(), readBytes.size(), reinterpret_cast<void*>(&read), original)) return false;
        originalRead = reinterpret_cast<ReadItem>(original);
        installed = true;
        return true;
    }
    catch (...) { return false; }

    RE::NiPointer<RE::EquippedItemData> create(RE::Actor* actor,
        const RE::BGSObjectInstance& weapon, std::uint32_t index) noexcept
    {
        if (!installed || !actor || !weapon.object) return {};
        // E806B0 allocates exactly A8 bytes and invokes EC2E30 with this ABI.
        // Construct only the data object: E806B0's insertion and E80CD0's
        // player equipment notifications do not belong to a carried item.
        const auto freeMemory = [](void* pointer) { RE::free(pointer); };
        std::unique_ptr<void, decltype(freeMemory)> memory(RE::malloc(0xA8), freeMemory);
        if (!memory) return {};
        auto* data = reinterpret_cast<Construct>(REL::Offset(0xEC2E30).address())(
            memory.get(), actor, &weapon, index);
        if (data != memory.get()) return {};
        RE::NiPointer<RE::EquippedItemData> result(data);
        (void)memory.release();
        return result;
    }

    Scope::Scope(RE::AIProcess* process, const RE::EquippedItem& item) noexcept :
        _previousProcess(activeProcess), _previousItem(activeItem)
    {
        activeProcess = process;
        activeItem = &item;
    }

    Scope::~Scope()
    {
        activeProcess = _previousProcess;
        activeItem = _previousItem;
    }

    const RE::EquippedItem* current(RE::AIProcess* process, std::uint32_t index) noexcept
    {
        return process && process == activeProcess && activeItem && activeItem->equipIndex.index == index ? activeItem : nullptr;
    }
}
