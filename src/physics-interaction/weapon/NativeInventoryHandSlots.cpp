#include "physics-interaction/weapon/NativeInventoryHandSlots.h"
#include "physics-interaction/weapon/NativeInventorySlotStub.h"
#include "physics-interaction/native/EntryTrampolineHook.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/PhysicsLog.h"
#include "rock_support/Fo4VrRuntime.h"
#include "RE/Bethesda/PlayerCharacter.h"
#include "RE/Bethesda/TESBoundObjects.h"
#include <array>
#include <tuple>
#include <type_traits>
#include <Windows.h>
#include <cstring>
#include <stdexcept>

namespace rock::native_inventory_hand_slots
{
    namespace
    {
        bool s_installed{};
        thread_local bool t_player{};
        using Item=RE::BGSObjectInstance;
        using Slot=RE::BGSEquipSlot;
        template<std::uintptr_t Rva> struct Entry { static inline void* original{}; };
        template<class Fn,std::uintptr_t Rva> Fn original() { return reinterpret_cast<Fn>(Entry<Rva>::original); }
        bool firearm(const Item* item)
        {
            if (!item || !item->object || !item->object->IsWeapon()) return false;
            const auto* data=item->instanceData?static_cast<const RE::TESObjectWEAP::InstanceData*>(item->instanceData.get()):
                &static_cast<const RE::TESObjectWEAP*>(item->object)->weaponData;
            return data->type==RE::WEAPON_TYPE::kGun;
        }
        Slot* hand(std::uint32_t index)
        {
            return index<2?RE::TESForm::GetFormByID<Slot>(index?0x13F43:0x13F42):nullptr;
        }
        int relative(const Item* item,const Slot* slot)
        {
            if (s_installed && t_player && firearm(item)) {
                if (slot && slot==hand(0)) return 0;
                if (slot && slot==hand(1)) return 1;
            }
            return original<int(*)(const Item*,const Slot*),0x146340>()(item,slot);
        }
        Slot* ordinal(const Item* item,int index)
        {
            if (s_installed && t_player && firearm(item) && index>=0 && index<2)
                if (auto* slot=hand(static_cast<std::uint32_t>(index))) return slot;
            return original<Slot*(*)(const Item*,int),0x1462E0>()(item,index);
        }
        const void* readPointer(const void* object,std::ptrdiff_t offset=0)
        {
            const void* value{};
            return native_memory::tryReadField(object,offset,value)?value:nullptr;
        }
        bool playerRecord(const void* record)
        {
            const auto* player=f4vr::getPlayer();
            const auto* inventory=player?player->inventoryList:nullptr;
            const void* data{}; std::uint32_t count{};
            // Native item iterators retain their current inventory storage.
            // Identify ownership by this exact element, never by a base form
            // or shared InstanceData that another actor can also possess.
            if (!record || !inventory || !native_memory::tryReadField(inventory,0x58,data) || !data ||
                !native_memory::tryReadField(inventory,0x68,count) || count>65536) return false;
            const auto address=reinterpret_cast<std::uintptr_t>(record),begin=reinterpret_cast<std::uintptr_t>(data);
            return address>=begin && (address-begin)%0x10==0 && (address-begin)/0x10<count;
        }
        enum class Owner { ActorFirst,ActorSecond,InventoryFirst,InventoryCallback,ItemFirst,ItemCallback,DoubleFirst,SingleSecond,DoubleSecond,Rebuild,RebuildFirst };
        const RE::TESObjectREFR* owner(Owner kind,const void* a,const void* b)
        {
            auto* player=f4vr::getPlayer();
            switch (kind) {
            case Owner::ActorFirst: return static_cast<const RE::TESObjectREFR*>(a);
            case Owner::ActorSecond: return static_cast<const RE::TESObjectREFR*>(b);
            case Owner::InventoryFirst: return player && a==player->inventoryList?player:nullptr;
            case Owner::InventoryCallback: return player && readPointer(a,8)==player->inventoryList?player:nullptr;
            case Owner::ItemFirst: return playerRecord(a)?player:nullptr;
            case Owner::ItemCallback: return playerRecord(readPointer(a))?player:nullptr;
            case Owner::DoubleFirst: return static_cast<const RE::TESObjectREFR*>(readPointer(readPointer(a)));
            case Owner::SingleSecond: return static_cast<const RE::TESObjectREFR*>(readPointer(b));
            case Owner::DoubleSecond: return static_cast<const RE::TESObjectREFR*>(readPointer(readPointer(b)));
            case Owner::Rebuild: return static_cast<const RE::TESObjectREFR*>(readPointer(readPointer(b,8)));
            case Owner::RebuildFirst: return static_cast<const RE::TESObjectREFR*>(readPointer(readPointer(a,8)));
            }
            return nullptr;
        }
        template<class T> const void* pointer(T value)
        {
            if constexpr (std::is_pointer_v<T>) return value;
            else return reinterpret_cast<const void*>(static_cast<std::uintptr_t>(value));
        }
        template<std::uintptr_t Rva,Owner Kind,class R,class... Args> R scoped(Args... args)
        {
            const auto values=std::tuple{args...};
            const void* second=nullptr;
            if constexpr (sizeof...(Args)>1) second=pointer(std::get<1>(values));
            const Scope scope(owner(Kind,pointer(std::get<0>(values)),second));
            return original<R(*)(Args...),Rva>()(args...);
        }
        void createEquipped(void* process,RE::Actor* actor,const Item* item,const Slot* slot)
        {
            const Scope scope(actor);
            const Slot* selected=slot;
            if (s_installed && t_player && firearm(item) && slot &&
                (slot->formID==0x13F44 || slot->formID==0x13F45)) {
                // This is the native construction boundary, including save
                // reconstruction. Preserve the real item and native ammo/data
                // constructor; canonicalize only the primary ownership leaf.
                if (auto* right=hand(0)) selected=right;
                try {
                    ROCK_LOG_SAMPLE_INFO(Weapon,1000,"AKIMBO native primary construction form={:08X} authoredSlot={:08X} actualSlot={:08X}",
                        item->object->formID,slot->formID,selected->formID);
                } catch (...) { OutputDebugStringA("ROCK: native inventory construction trace failed\n"); }
            }
            original<void(*)(void*,RE::Actor*,const Item*,const Slot*),0xE806B0>()(process,actor,item,selected);
        }
        Slot* indexedUnequip(const Item* item,int index,const void* callback)
        {
            const Scope scope(owner(Owner::DoubleFirst,callback,nullptr));
            return ordinal(item,index);
        }
        bool installIndexedUnequip()
        {
            constexpr std::uintptr_t rva=0xE202ED;
            constexpr std::array<std::uint8_t,5> expected{0xE8,0xEE,0x5F,0x32,0xFF};
            const auto address=REL::Offset(rva).address();
            std::array<std::uint8_t,5> live{};
            if (!native_memory::guardedCopyFromMemory(reinterpret_cast<void*>(address),live.data(),live.size()) || live!=expected) return false;
            try {
                OrdinalStub stub(reinterpret_cast<std::uintptr_t>(&indexedUnequip));
                auto& trampoline=F4SE::GetTrampoline();
                if (trampoline.free_size()<stub.getSize()+14) return false;
                const auto target=reinterpret_cast<std::uintptr_t>(trampoline.allocate(stub.getSize()));
                std::memcpy(reinterpret_cast<void*>(target),stub.getCode(),stub.getSize());
                (void)trampoline.write_call<5>(address,target);
                std::int32_t displacement{};
                if (!native_memory::guardedCopyFromMemory(reinterpret_cast<void*>(address),live.data(),live.size()) || live[0]!=0xE8)
                    throw std::runtime_error("inventory call write failed");
                std::memcpy(&displacement,live.data()+1,4);
                std::array<std::uint8_t,14> relay{};
                std::uintptr_t destination{};
                const bool readable=native_memory::guardedCopyFromMemory(reinterpret_cast<void*>(address+5+displacement),relay.data(),relay.size());
                std::memcpy(&destination,relay.data()+6,8);
                constexpr std::array<std::uint8_t,6> jump{0xFF,0x25,0,0,0,0};
                if (!readable || std::memcmp(relay.data(),jump.data(),jump.size()) || destination!=target ||
                    !FlushInstructionCache(GetCurrentProcess(),nullptr,0))
                    throw std::runtime_error("inventory call destination failed");
                return true;
            } catch (...) {
                REL::safe_write(address,expected.data(),expected.size());
                FlushInstructionCache(GetCurrentProcess(),nullptr,0);
                return false;
            }
        }
        template<std::uintptr_t Rva,std::size_t N,class Fn>
        bool hook(const char* label,const std::array<std::uint8_t,N>& bytes,Fn fn)
        {
            return entry_trampoline_hook::install(label,Rva,bytes.data(),N,reinterpret_cast<void*>(fn),Entry<Rva>::original);
        }
    }
    Scope::Scope(const RE::TESObjectREFR* owner) noexcept:_previous(t_player)
    {
        t_player=owner && owner==f4vr::getPlayer();
    }
    Scope::~Scope() { t_player=_previous; }

    bool ownsNativeUnequipEntry() noexcept
    {
        if (!Entry<0xE70280>::original) return false;
        std::array<std::uint8_t,14> bytes{};
        if (!native_memory::guardedCopyFromMemory(reinterpret_cast<void*>(REL::Offset(0xE70280).address()),bytes.data(),bytes.size())) return false;
        constexpr std::array<std::uint8_t,6> jump{0xFF,0x25,0,0,0,0};
        std::uintptr_t destination{};
        std::memcpy(&destination,bytes.data()+6,8);
        const auto expected=reinterpret_cast<std::uintptr_t>(&scoped<0xE70280,Owner::ActorSecond,bool,void*,RE::Actor*,const Item*,std::int32_t,const Slot*,std::uint32_t,bool,bool,bool,bool,void*>);
        return std::memcmp(bytes.data(),jump.data(),jump.size())==0 && destination==expected;
    }

    bool install() noexcept
    {
        if (s_installed) return true;
        if (!REL::Module::IsVR() || REL::Module::get().version()!=F4SE::RUNTIME_VR_1_2_72) return false;
        // All owner callbacks preserve their native arguments/return and mask
        // nested foreign actors. Both slot directions stay scoped together.
        const auto slotPrefix=std::to_array<std::uint8_t>({0x48,0x89,0x5C,0x24,0x08,0x57,0x48,0x83,0xEC,0x20,0x48,0x8B,0x79,0x08});
        if (!hook<0x146340>("Player inventory relative hand",slotPrefix,&relative) ||
            !hook<0x1462E0>("Player inventory ordinal hand",slotPrefix,&ordinal) ||
            !hook<0xE806B0>("Player canonical equipped construction",std::to_array<std::uint8_t>({0x4C,0x8B,0xDC,0x49,0x89,0x6B,0x20,0x56,0x41,0x54,0x41,0x55,0x41,0x56}),&createEquipped)) return false;
        if (!hook<0x1AED30>("Player inventory owner 1AED30",std::to_array<std::uint8_t>({0x48,0x83,0xEC,0x48,0x48,0x8B,0x01,0x33,0xD2,0x48,0x89,0x54,0x24,0x20}),
            &scoped<0x1AED30,Owner::ItemFirst,std::uint64_t,void*>)) return false;
        if (!hook<0x1AED00>("Player inventory owner 1AED00",std::to_array<std::uint8_t>({0x40,0x53,0x48,0x83,0xEC,0x30,0x48,0x8B,0xDA,0x48,0x89,0x54,0x24,0x28}),
            &scoped<0x1AED00,Owner::ItemFirst,std::uint32_t,void*,void*>)) return false;
        if (!hook<0x1B1740>("Player inventory owner 1B1740",std::to_array<std::uint8_t>({0x48,0x89,0x5C,0x24,0x18,0x48,0x89,0x74,0x24,0x20,0x57,0x48,0x83,0xEC,0x30}),
            &scoped<0x1B1740,Owner::ItemFirst,void,void*,void*>)) return false;
        if (!hook<0x1B15B0>("Player inventory owner 1B15B0",std::to_array<std::uint8_t>({0x48,0x89,0x5C,0x24,0x08,0x48,0x89,0x6C,0x24,0x18,0x48,0x89,0x74,0x24,0x20}),
            &scoped<0x1B15B0,Owner::ItemFirst,std::uint64_t,void*,void*>)) return false;
        if (!hook<0xE650E0>("Player inventory owner E650E0",std::to_array<std::uint8_t>({0x48,0x89,0x5C,0x24,0x18,0x89,0x54,0x24,0x10,0x57,0x48,0x83,0xEC,0x60}),
            &scoped<0xE650E0,Owner::ActorFirst,std::uintptr_t,RE::Actor*,std::uint32_t>)) return false;
        if (!hook<0xE651D0>("Player inventory owner E651D0",std::to_array<std::uint8_t>({0x48,0x89,0x5C,0x24,0x18,0x89,0x54,0x24,0x10,0x57,0x48,0x83,0xEC,0x60}),
            &scoped<0xE651D0,Owner::ActorFirst,std::uintptr_t,RE::Actor*,std::uint32_t>)) return false;
        if (!hook<0xE66960>("Player inventory owner E66960",std::to_array<std::uint8_t>({0x44,0x89,0x44,0x24,0x18,0x57,0x41,0x56,0x48,0x83,0xEC,0x68,0x48,0x83,0x3A,0x00}),
            &scoped<0xE66960,Owner::ActorFirst,void,RE::Actor*,const Item*,std::uint32_t>)) return false;
        if (!hook<0x3E3910>("Player inventory owner 3E3910",std::to_array<std::uint8_t>({0x48,0x89,0x5C,0x24,0x08,0x48,0x89,0x6C,0x24,0x10,0x48,0x89,0x74,0x24,0x18}),
            &scoped<0x3E3910,Owner::ActorFirst,bool,RE::TESObjectREFR*,const Item*,const Slot*>)) return false;
        if (!hook<0xEDB910>("Player inventory owner EDB910",std::to_array<std::uint8_t>({0x48,0x8B,0xC4,0x48,0x81,0xEC,0xA8,0x00,0x00,0x00,0x48,0x89,0x58,0x08}),
            &scoped<0xEDB910,Owner::DoubleFirst,std::uint64_t,void*,void*>)) return false;
        if (!hook<0xEDBAD0>("Player inventory owner EDBAD0",std::to_array<std::uint8_t>({0x48,0x89,0x5C,0x24,0x08,0x48,0x89,0x6C,0x24,0x10,0x48,0x89,0x74,0x24,0x18}),
            &scoped<0xEDBAD0,Owner::DoubleFirst,bool,void*,void*>)) return false;
        if (!hook<0x1B3D20>("Player inventory owner 1B3D20",std::to_array<std::uint8_t>({0x48,0x89,0x5C,0x24,0x10,0x48,0x89,0x6C,0x24,0x18,0x48,0x89,0x74,0x24,0x20}),
            &scoped<0x1B3D20,Owner::InventoryFirst,void,void*,void*,std::uint32_t,void*,std::uint32_t,bool>)) return false;
        if (!hook<0x1B5C90>("Player inventory owner 1B5C90",std::to_array<std::uint8_t>({0x40,0x55,0x41,0x56,0x48,0x83,0xEC,0x48,0x48,0x8B,0x01,0x48,0x8B,0xEA}),
            &scoped<0x1B5C90,Owner::InventoryCallback,std::uint64_t,void*,void*>)) return false;
        if (!hook<0xE0F920>("Player inventory owner E0F920",std::to_array<std::uint8_t>({0x48,0x89,0x5C,0x24,0x08,0x55,0x56,0x57,0x41,0x54,0x41,0x55,0x41,0x56}),
            &scoped<0xE0F920,Owner::ActorFirst,void,RE::Actor*,const Item*,void*>)) return false;
        if (!hook<0xE70280>("Player inventory owner E70280",std::to_array<std::uint8_t>({0x48,0x8B,0xC4,0x48,0x89,0x58,0x18,0x55,0x57,0x41,0x56,0x48,0x83,0xEC,0x70}),
            &scoped<0xE70280,Owner::ActorSecond,bool,void*,RE::Actor*,const Item*,std::int32_t,const Slot*,std::uint32_t,bool,bool,bool,bool,void*>)) return false;
        if (!hook<0x407650>("Player inventory owner 407650",std::to_array<std::uint8_t>({0x40,0x53,0x55,0x56,0x41,0x55,0x41,0x56,0x41,0x57,0x48,0x83,0xEC,0x38}),
            &scoped<0x407650,Owner::Rebuild,std::uint64_t,void*,void*>)) return false;
        if (!hook<0x4078C0>("Player inventory owner 4078C0",std::to_array<std::uint8_t>({0x40,0x53,0x55,0x56,0x41,0x55,0x41,0x56,0x41,0x57,0x48,0x83,0xEC,0x38}),
            &scoped<0x4078C0,Owner::Rebuild,std::uint64_t,void*,void*>)) return false;
        if (!hook<0xE27B10>("Player inventory owner E27B10",std::to_array<std::uint8_t>({0x48,0x89,0x5C,0x24,0x08,0x48,0x89,0x6C,0x24,0x10,0x48,0x89,0x74,0x24,0x18}),
            &scoped<0xE27B10,Owner::DoubleSecond,std::uint64_t,void*,void*>)) return false;
        if (!hook<0x40C680>("Player inventory owner 40C680",std::to_array<std::uint8_t>({0x48,0x8B,0xC4,0x41,0x54,0x41,0x55,0x41,0x57,0x48,0x83,0xEC,0x60,0x4C,0x8B,0x79,0x08}),
            &scoped<0x40C680,Owner::SingleSecond,void,void*,void*>)) return false;
        if (!hook<0x402210>("Player inventory owner 402210",std::to_array<std::uint8_t>({0x48,0x89,0x5C,0x24,0x08,0x48,0x89,0x74,0x24,0x10,0x57,0x48,0x83,0xEC,0x30}),
            &scoped<0x402210,Owner::DoubleFirst,std::uint64_t,void*,std::uint32_t>)) return false;
        if (!hook<0xE2F790>("Player inventory owner E2F790",std::to_array<std::uint8_t>({0x48,0x8B,0xC4,0x55,0x41,0x56,0x41,0x57,0x48,0x81,0xEC,0x90,0x00,0x00,0x00}),
            &scoped<0xE2F790,Owner::SingleSecond,void,void*,void*>)) return false;
        const auto rebuildPrefix=std::to_array<std::uint8_t>({0x48,0x89,0x5C,0x24,0x10,0x48,0x89,0x6C,0x24,0x20,0x56,0x57,0x41,0x56});
        if (!hook<0x3FFAD0>("Player inventory model reconstruction",rebuildPrefix,
            &scoped<0x3FFAD0,Owner::RebuildFirst,std::uint64_t,void*,std::uint32_t>) ||
            !hook<0x3FFC70>("Player inventory model reconstruction alternate",rebuildPrefix,
            &scoped<0x3FFC70,Owner::RebuildFirst,std::uint64_t,void*,std::uint32_t>)) return false;
        if (!hook<0x407800>("Player inventory clear mask",std::to_array<std::uint8_t>({0x48,0x89,0x5C,0x24,0x20,0x41,0x54,0x41,0x56,0x41,0x57,0x48,0x83,0xEC,0x30}),
            &scoped<0x407800,Owner::DoubleSecond,std::uint64_t,void*,void*>)) return false;
        if (!hook<0x1B0BA0>("Player inventory stack slot collector",std::to_array<std::uint8_t>({0x40,0x57,0x41,0x56,0x48,0x83,0xEC,0x48,0xF6,0x42,0x24,0x07,0x48,0x8B,0xFA}),
            &scoped<0x1B0BA0,Owner::ItemCallback,std::uint64_t,void*,void*>)) return false;
        if (!hook<0xE5F580>("Player inventory equipped condition",std::to_array<std::uint8_t>({0x48,0x89,0x6C,0x24,0x18,0x48,0x89,0x74,0x24,0x20,0x41,0x56,0x48,0x83,0xEC,0x50}),
            &scoped<0xE5F580,Owner::ItemFirst,std::int32_t,void*,void*>)) return false;
        if (!installIndexedUnequip()) return false;
        s_installed=true;
        ROCK_LOG_INFO(Init,"Native player inventory hand mapping installed: physical worn bits, exact player ownership, canonical primary construction");
        return true;
    }
}
