#include "physics-interaction/weapon/NativeEquippedBoneOwnership.h"
#include "physics-interaction/weapon/NativeHandEquipment.h"
#include "physics-interaction/weapon/NativeModelAncestry.h"
#include "physics-interaction/native/EntryTrampolineHook.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/PhysicsLog.h"
#include "rock_support/Fo4VrRuntime.h"
#include "RE/Bethesda/PlayerCharacter.h"
#include "RE/Bethesda/TESBoundObjects.h"
#include <array>
#include <Windows.h>

namespace rock::native_equipped_bone_ownership
{
    namespace
    {
        // 1785010 retains this node and writes index+8; 1785340 consumes the
        // same 0x10-byte-stride table. 17901D0 constructs/destructs its entries.
        struct Binding { RE::NiPointer<RE::NiAVObject> node; std::int32_t index{-1}; std::uint32_t pad{}; };
        static_assert(sizeof(Binding)==0x10 && offsetof(Binding,index)==8);
        using Bind=void(*)(void*,void*,RE::NiAVObject*,RE::BSTArray<Binding>*);
        using Find=bool(*)(RE::NiAVObject*,const RE::BSFixedString*,Binding*);
        Bind s_bind{};
        bool s_installed{};
        using native_model_ancestry::Relation;
        using native_model_ancestry::within;

        struct Model { RE::BGSObjectInstance item{nullptr,nullptr}; RE::NiPointer<RE::NiAVObject> root; };
        bool model(RE::PlayerCharacter* player,const void* biped,std::uint32_t hand,Model& out)
        {
            using Read=RE::BGSObjectInstance*(*)(RE::Actor*,RE::BGSObjectInstance*,std::uint32_t);
            reinterpret_cast<Read>(REL::Offset(0xE50DA0).address())(player,&out.item,hand);
            if (!out.item.object || !out.item.object->IsWeapon()) return false;
            const auto* data=out.item.instanceData?static_cast<const RE::TESObjectWEAP::InstanceData*>(out.item.instanceData.get()):
                &static_cast<const RE::TESObjectWEAP*>(out.item.object)->weaponData;
            if (data->type!=RE::WEAPON_TYPE::kGun) return false;
            using Index=std::uint32_t(*)(RE::Actor*,RE::TESBoundObject*,std::uint32_t);
            const auto index=reinterpret_cast<Index>(REL::Offset(0x1CAFC0).address())(player,static_cast<RE::TESObjectWEAP*>(out.item.object),hand);
            if (!biped || index>=44) return false;
            const auto* record=static_cast<const std::byte*>(biped)+0x10+index*0x58;
            RE::TESBoundObject* object{}; RE::TBO_InstanceData* instance{}; RE::NiAVObject* root{};
            if (!native_memory::tryReadField(record,0,object) || object!=out.item.object ||
                !native_memory::tryReadField(record,8,instance) || instance!=out.item.instanceData.get() ||
                !native_memory::tryReadField(record,0x30,root) || !root) return false;
            out.root.reset(root);
            return true;
        }
        void bind(void* mapper,void* skeleton,RE::NiAVObject* scene,RE::BSTArray<Binding>* output) noexcept
        {
            s_bind(mapper,skeleton,scene,output);
            try {
            auto* player=s_installed?f4vr::getPlayer():nullptr;
            // Private samplers and NPCs never enter this correction. The
            // native graph/scene binding owner already retains this root;
            // the biped models are retained only within the same callback.
            if (!player || !scene || scene!=player->firstPerson3D.get() || !output || output->size()>768) return;
            Model right,left;
            const auto* biped=native_hand_equipment::playerBiped(true);
            if (!model(player,biped,0,right) || !model(player,biped,1,left) || right.root==left.root ||
                within(right.root.get(),scene)!=Relation::Inside || within(left.root.get(),scene)!=Relation::Inside) return;
            std::size_t crossed=0,repaired=0,unresolved=0,rightCount=0,flatCount=0,missingCount=0;
            for (auto& binding:*output) {
                if (!binding.node) ++missingCount;
                else if (binding.index>=0) ++flatCount;
                if (binding.node && within(binding.node.get(),right.root.get())==Relation::Inside) ++rightCount;
                if (!binding.node || within(binding.node.get(),left.root.get())!=Relation::Inside ||
                    within(binding.node.get(),right.root.get())==Relation::Inside) continue;
                ++crossed;
                // Model-local NiNodes are the conflicting targets. A flattened
                // tree is not a model bone; reject an unexpected indexed match
                // rather than reinterpret or write its palette.
                Binding replacement;
                const auto name=binding.node->name;
                const bool found=binding.index<0 && reinterpret_cast<Find>(REL::Offset(0x1C20840).address())(right.root.get(),&name,&replacement) &&
                    replacement.node && within(replacement.node.get(),right.root.get())==Relation::Inside;
                if (crossed<=12) ROCK_LOG_INFO(Animation,
                    "AKIMBO primary bone owner name='{}' right={:08X} left={:08X} old={:#x} new={:#x} matched={}",
                    name.c_str(),right.item.object->formID,left.item.object->formID,
                    reinterpret_cast<std::uintptr_t>(binding.node.get()),reinterpret_cast<std::uintptr_t>(replacement.node.get()),found);
                if (found) { binding=std::move(replacement); ++repaired; }
                else { binding.node.reset(); binding.index=-1; ++unresolved; }
            }
            ROCK_LOG_INFO(Animation,"AKIMBO primary binding ownership right={:08X} left={:08X} bones={} rightTargets={} flatTargets={} missingTargets={} crossed={} repaired={} unresolved={}",
                right.item.object->formID,left.item.object->formID,output->size(),rightCount,flatCount,missingCount,crossed,repaired,unresolved);
            } catch (...) { OutputDebugStringA("ROCK: equipped bone ownership correction failed\n"); }
        }
    }
    bool install() noexcept
    {
        if (s_installed) return true;
        if (!REL::Module::IsVR() || REL::Module::get().version()!=F4SE::RUNTIME_VR_1_2_72) return false;
        constexpr auto prefix=std::to_array<std::uint8_t>({0x40,0x55,0x57,0x41,0x56,0x41,0x57,0x48,0x83,0xEC,0x48,0x8B,0x7A,0x30});
        void* original{};
        if (!entry_trampoline_hook::install("Equipped graph bone ownership",0x1785010,prefix.data(),prefix.size(),reinterpret_cast<void*>(&bind),original)) return false;
        s_bind=reinterpret_cast<Bind>(original);
        s_installed=true;
        return true;
    }
}
