#include "physics-interaction/weapon/NativeSecondaryActions.h"
#include "physics-interaction/weapon/NativeEquippedActionGraph.h"
#include "physics-interaction/weapon/NativeHandEquipment.h"
#include "physics-interaction/weapon/NativeSecondaryBoneBindings.h"
#include "physics-interaction/native/EntryTrampolineHook.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/native/NativeNiNodeFactory.h"
#include "physics-interaction/weapon/recoil/RecoilMath.h"
#include "physics-interaction/weapon/WeaponInstanceClassification.h"
#include "physics-interaction/weapon/HoldingGripAdapter.h"
#include "RockConfig.h"
#include "physics-interaction/input/InputRemapRuntime.h"
#include "physics-interaction/input/InputRemapPolicy.h"
#include "physics-interaction/input/NativeSecondaryTrigger.h"
#include "physics-interaction/core/RockRuntimeState.h"
#include "physics-interaction/PhysicsLog.h"
#include "rock_support/Fo4VrRuntime.h"
#include "RE/Bethesda/PlayerCharacter.h"
#include "RE/Bethesda/TESBoundObjects.h"
#include "RE/Bethesda/BGSDefaultObjectManager.h"
#include <Windows.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <memory>
#include <utility>

namespace rock::native_secondary_actions
{
    namespace
    {
        using Graph=native_equipped_action_graph::Graph;
        using View=native_hand_equipment::EquippedView;
        using ManagerRef=RE::BSTSmartPointer<RE::BSAnimationGraphManager>;
        using GetManager=bool(*)(void*,ManagerRef*);
        using IdleRoots=bool(*)(void*,RE::Actor*,RE::BSScrapArray<void*>*);
        using DefaultWeapon=std::uint32_t*(*)(RE::Actor*,std::uint32_t*,RE::BGSObjectInstance*);
        using PrepareShot=bool(*)(void*);
        using Muzzle=RE::NiAVObject*(*)(RE::Actor*,std::uint32_t);
        using ModelInt=bool(*)(void*,std::uint32_t,const RE::BSFixedString*,int);
        using Pulse=void(*)(std::uint32_t,float,float);
        using Pattern=void(*)(std::uint32_t,float,float,std::uint32_t,std::uint32_t);
        using SetLoaded=void(*)(void*,std::uint32_t,std::uint32_t);
        GetManager s_getManager{};
        IdleRoots s_idleRoots{};
        DefaultWeapon s_defaultWeapon{};
        PrepareShot s_prepareShot{};
        Muzzle s_muzzle{};
        ModelInt s_modelInt{};
        Pulse s_pulse{};
        Pattern s_pattern{};
        SetLoaded s_setLoaded{};
        bool s_installed{};

        struct NativeKick { float duration{},elapsed{},distance{}; };
        static_assert(sizeof(NativeKick)==12);

        template<class F> F native(std::uintptr_t rva) { return reinterpret_cast<F>(REL::Offset(rva).address()); }
        template<std::size_t N> bool matches(std::uintptr_t rva,const std::array<std::uint8_t,N>& expected)
        {
            std::array<std::uint8_t,N> actual{};
            return native_memory::guardedCopyFromMemory(reinterpret_cast<void*>(REL::Offset(rva).address()),actual.data(),N) && actual==expected;
        }
        struct State
        {
            std::unique_ptr<Graph> graph;
            BoneBindings bindings;
            std::array<ManagerRef,2> modelManagers;
            ManagerRef originalParent;
            RE::BGSObjectInstance item{nullptr,nullptr};
            RE::NiPointer<RE::NiAVObject> model;
            std::uintptr_t data{};
            std::uint64_t frame{};
            DWORD thread{};
            bool retiring{},failed{},ready{},reload{},reloading{};
            native_secondary_trigger::Intent trigger;
            std::uint32_t actions{},events{},shots{};
            std::uint32_t inputMask{UINT32_MAX},inputTraces{};
            const char* tracedStage{};
            NativeKick kick;
            bool kickActive{};
            RE::NiPointer<RE::NiNode> recoilNode;
            std::uint64_t recoilFrame{UINT64_MAX};
            weapon_recoil_policy::Family recoilFamily{weapon_recoil_policy::Family::Default};
            std::uint32_t recoilShots{},recoilSamples{};
            struct TraceWindow { std::uint64_t second{UINT64_MAX}; std::uint32_t count{},suppressed{}; };
            std::array<TraceWindow,6> traces{};
        };
        enum class Trace : std::size_t { Action,Event,Lifecycle,Fire,Recoil,Input };
        bool trace(State& s,Trace channel)
        {
            auto& window=s.traces[static_cast<std::size_t>(channel)];
            const auto second=GetTickCount64()/1000;
            if (window.second!=second) {
                if (window.suppressed) ROCK_LOG_INFO(Weapon,"AKIMBO diagnostic sampling form={:08X} channel={} suppressed={} windowSeconds=1",
                    s.item.object?s.item.object->formID:0,static_cast<std::size_t>(channel),window.suppressed);
                window={second,0,0};
            }
            if (window.count<8) { ++window.count; return true; }
            ++window.suppressed;
            return false;
        }
        State& state()
        {
            // No engine calls during DLL static destruction. Native I/O may
            // still be running then; normal frame retirement owns all cleanup.
            static auto* instance=new State();
            return *instance;
        }
        struct Context {
            State* state;
            RE::PlayerCharacter* player;
            bool shot{};
            RE::NiAVObject* muzzle{};
            bool prepared{};
            std::uint32_t managerReads{},idleQueries{},idleMatches{};
            bool recoil{};
        };
        thread_local Context* t_context{};
        struct Scope
        {
            Context* previous;
            explicit Scope(Context& context):previous(std::exchange(t_context,&context)) {}
            ~Scope() { t_context=previous; }
        };
        bool same(const State& s,const View& view)
        {
            return view.weapon && view.weapon==s.item.object && view.instance==s.item.instanceData.get() &&
                view.data==s.data && view.root==s.model.get();
        }
        void releaseModelGraphs(State& s,bool restore)
        {
            for (auto& manager:s.modelManagers) if (manager) {
                if (s.graph && s.graph->manager())
                    native<void(*)(RE::BSAnimationGraphManager*,ManagerRef*)>(0x16906A0)(s.graph->manager(),&manager);
                if (restore && s.originalParent)
                    native<void(*)(RE::BSAnimationGraphManager*,ManagerRef*,bool)>(0x1690440)(s.originalParent.get(),&manager,true);
                manager.reset();
            }
            s.originalParent.reset();
        }
        bool isolateModelGraphs(State& s)
        {
            auto* player=f4vr::getPlayer();
            ManagerRef parent;
            if (!s_getManager(static_cast<RE::IAnimationGraphManagerHolder*>(player),&parent) || !parent) return false;
            const auto slot=native<std::uint32_t(*)(RE::Actor*,RE::TESBoundObject*,std::uint32_t)>(0x1CAFC0)(player,static_cast<RE::TESObjectWEAP*>(s.item.object),1);
            if (slot>=44) return false;
            if (s.originalParent && s.originalParent!=parent) releaseModelGraphs(s,false);
            s.originalParent=parent;
            for (std::size_t view=0;view<2;++view) {
                const auto* biped=native_hand_equipment::playerBiped(view==0);
                if (!biped) continue;
                const auto* record=static_cast<const std::byte*>(biped)+slot*0x58;
                RE::TESBoundObject* form{}; RE::TBO_InstanceData* instance{}; void* holder{};
                if (!native_memory::tryReadField(record,0x10,form) || form!=s.item.object ||
                    !native_memory::tryReadField(record,0x18,instance) || instance!=s.item.instanceData.get() ||
                    !native_memory::tryReadField(record,0x50,holder)) return false;
                ManagerRef manager;
                if (holder) {
                    std::uintptr_t vtable{};
                    if (!native_memory::tryReadField(holder,0,vtable) || vtable!=REL::Offset(0x2D3A8D8).address()) return false;
                    native<bool(*)(void*,ManagerRef*)>(0x812EA0)(holder,&manager);
                }
                if (view==1 && manager && manager==s.modelManagers[0]) continue;
                if (manager==s.modelManagers[view]) continue;
                if (s.modelManagers[view]) native<void(*)(RE::BSAnimationGraphManager*,ManagerRef*)>(0x16906A0)(s.graph->manager(),&s.modelManagers[view]);
                s.modelManagers[view]=manager;
                if (!manager) continue; // A native model graph is optional.
                native<void(*)(RE::BSAnimationGraphManager*,ManagerRef*)>(0x16906A0)(parent.get(),&s.modelManagers[view]);
                native<void(*)(RE::BSAnimationGraphManager*,ManagerRef*,bool)>(0x1690440)(s.graph->manager(),&s.modelManagers[view],true);
                ROCK_LOG_INFO(Animation,"AKIMBO model graph isolated form={:08X} view={} manager={:#x} oldParent={:#x} newParent={:#x}",
                    s.item.object->formID,view,reinterpret_cast<std::uintptr_t>(manager.get()),reinterpret_cast<std::uintptr_t>(parent.get()),reinterpret_cast<std::uintptr_t>(s.graph->manager()));
            }
            return true;
        }
        bool validContext()
        {
            return s_installed && t_context && t_context->player==f4vr::getPlayer() &&
                t_context->state->ready && t_context->state->thread==GetCurrentThreadId() &&
                same(*t_context->state,native_hand_equipment::equippedViews()[1]);
        }
        // EF96B0 and EFD1C0 share only these three scalar fields and bit 1.
        // Execute those synchronous native leaf operations on the scene owner,
        // restoring the primary record before returning to any engine caller.
        // No equipment, graph, inventory or player node pointer is exchanged.
        class RecoilScope
        {
            State& _state;
            RE::PlayerCharacter* _player;
            NativeKick _primary{};
            std::uint8_t _flags{};
            bool _active{};
        public:
            explicit RecoilScope(State& s,RE::PlayerCharacter* player):_state(s),_player(player)
            {
                if (!player || !native_memory::tryReadField(player,0x8FC,_primary) ||
                    !native_memory::tryReadField(player,0x12A4,_flags)) return;
                auto* bytes=reinterpret_cast<std::byte*>(player);
                if (!native_memory::tryWriteValue(reinterpret_cast<NativeKick*>(bytes+0x8FC),s.kick)) return;
                const auto flags=static_cast<std::uint8_t>((_flags&~2u)|(s.kickActive?2u:0u));
                if (!native_memory::tryWriteValue(reinterpret_cast<std::uint8_t*>(bytes+0x12A4),flags)) {
                    (void)native_memory::tryWriteValue(reinterpret_cast<NativeKick*>(bytes+0x8FC),_primary);
                    return;
                }
                _active=true;
            }
            ~RecoilScope()
            {
                if (!_active) return;
                std::uint8_t current{};
                const bool read=native_memory::tryReadField(_player,0x8FC,_state.kick) &&
                    native_memory::tryReadField(_player,0x12A4,current);
                _state.kickActive=read && (current&2)!=0;
                auto* bytes=reinterpret_cast<std::byte*>(_player);
                const auto restoreFlags=static_cast<std::uint8_t>((read?current:_flags)&~2u)|(_flags&2u);
                const bool restored=native_memory::tryWriteValue(reinterpret_cast<NativeKick*>(bytes+0x8FC),_primary) &&
                    native_memory::tryWriteValue(reinterpret_cast<std::uint8_t*>(bytes+0x12A4),static_cast<std::uint8_t>(restoreFlags));
                if (!read || !restored) ROCK_LOG_ERROR(Weapon,"AKIMBO native recoil record restore failed read={} restored={}",read,restored);
            }
            explicit operator bool() const { return _active; }
            RecoilScope(const RecoilScope&)=delete;
            RecoilScope& operator=(const RecoilScope&)=delete;
        };
        void* recoilAim(RE::Actor* actor,std::uint32_t index)
        {
            if (t_context && actor==t_context->player && (t_context->shot || t_context->recoil)) {
                if (!validContext()) return nullptr;
                index=1;
            }
            return native<void*(*)(RE::Actor*,std::uint32_t)>(0xE526A0)(actor,index);
        }
        void startRecoil(RE::PlayerCharacter* player)
        {
            if (!t_context || !t_context->shot || player!=t_context->player) {
                native<void(*)(RE::PlayerCharacter*)>(0xEF96B0)(player);
                return;
            }
            if (!validContext()) return;
            auto& s=*t_context->state;
            NativeKick primaryBefore{},primaryAfter{};
            std::uint8_t flagsBefore{},flagsAfter{};
            const bool before=native_memory::tryReadField(player,0x8FC,primaryBefore) && native_memory::tryReadField(player,0x12A4,flagsBefore);
            {
                RecoilScope record(s,player);
                if (!record) {
                    ROCK_LOG_SAMPLE_WARN(Weapon,1000,"AKIMBO secondary recoil record unavailable");
                    return;
                }
                native<void(*)(RE::PlayerCharacter*)>(0xEF96B0)(player);
            }
            const bool primaryPreserved=before && native_memory::tryReadField(player,0x8FC,primaryAfter) &&
                native_memory::tryReadField(player,0x12A4,flagsAfter) &&
                std::memcmp(&primaryBefore,&primaryAfter,sizeof(NativeKick))==0 && ((flagsBefore^flagsAfter)&2)==0;
            ++s.recoilShots;
            if (trace(s,Trace::Recoil) || !primaryPreserved) ROCK_LOG_INFO(Weapon,
                "AKIMBO native recoil shot form={:08X} index=1 duration={} distance={} active={} primaryPreserved={} primaryElapsed={}->{} sequence={}",
                s.item.object->formID,s.kick.duration,s.kick.distance,s.kickActive,primaryPreserved,primaryBefore.elapsed,primaryAfter.elapsed,s.recoilShots);
            if (!primaryPreserved) ROCK_LOG_ERROR(Weapon,"AKIMBO secondary recoil changed primary native state form={:08X}",s.item.object->formID);
        }
        bool installRecoilCalls()
        {
            constexpr std::array<std::uintptr_t,4> calls{0x333E26,0xEF96DC,0xEFD0D0,0xEFD1E7};
            constexpr std::array<std::uintptr_t,4> targets{0xEF96B0,0xE526A0,0xE526A0,0xE526A0};
            std::array<std::array<std::uint8_t,5>,4> originals{};
            for (std::size_t i=0;i<calls.size();++i) {
                originals[i][0]=0xE8;
                const auto displacement=static_cast<std::int32_t>(targets[i]-calls[i]-5);
                std::memcpy(originals[i].data()+1,&displacement,4);
                if (!matches(calls[i],originals[i])) return false;
            }
            const std::array<std::uintptr_t,4> hooks{reinterpret_cast<std::uintptr_t>(&startRecoil),
                reinterpret_cast<std::uintptr_t>(&recoilAim),reinterpret_cast<std::uintptr_t>(&recoilAim),reinterpret_cast<std::uintptr_t>(&recoilAim)};
            try {
                auto& trampoline=F4SE::GetTrampoline();
                if (trampoline.free_size()<64) return false;
                for (std::size_t i=0;i<calls.size();++i) {
                    const auto address=REL::Offset(calls[i]).address();
                    (void)trampoline.write_call<5>(address,hooks[i]);
                    std::array<std::uint8_t,5> call{};
                    std::array<std::uint8_t,14> relay{};
                    std::int32_t displacement{}; std::uintptr_t destination{};
                    bool valid=native_memory::guardedCopyFromMemory(reinterpret_cast<void*>(address),call.data(),call.size()) && call[0]==0xE8;
                    std::memcpy(&displacement,call.data()+1,4);
                    valid=valid && native_memory::guardedCopyFromMemory(reinterpret_cast<void*>(address+5+displacement),relay.data(),relay.size());
                    std::memcpy(&destination,relay.data()+6,8);
                    constexpr std::array<std::uint8_t,6> jump{0xFF,0x25,0,0,0,0};
                    if (!valid || std::memcmp(relay.data(),jump.data(),jump.size()) || destination!=hooks[i])
                        throw std::runtime_error("native recoil call redirect failed");
                }
                if (!FlushInstructionCache(GetCurrentProcess(),nullptr,0)) throw std::runtime_error("native recoil instruction cache failed");
                return true;
            } catch (...) {
                for (std::size_t i=0;i<calls.size();++i) REL::safe_write(REL::Offset(calls[i]).address(),originals[i].data(),originals[i].size());
                FlushInstructionCache(GetCurrentProcess(),nullptr,0);
                return false;
            }
        }
        bool getManager(void* holder,ManagerRef* out)
        {
            if (t_context && holder==static_cast<RE::IAnimationGraphManagerHolder*>(t_context->player) && out) {
                if (!validContext()) { out->reset(); return false; }
                ++t_context->managerReads;
                *out=t_context->state->graph->holder()->animationGraphManager;
                return bool(*out);
            }
            return s_getManager(holder,out);
        }
        bool idleRoots(void* database,RE::Actor* actor,RE::BSScrapArray<void*>* out)
        {
            if (!t_context || actor!=t_context->player || !out) return s_idleRoots(database,actor,out);
            if (!validContext()) return false;
            ++t_context->idleQueries;
            // Native 595C50 still adds the race/base root after this collector.
            // Only the weapon-specific roots come from this hand's exact native
            // subgraph request; none are inserted into the player's primary list.
            for (const auto& name:t_context->state->graph->idleRoots()) {
                auto* root=native<void*(*)(void*,const char*)>(0x595760)(database,name.c_str());
                if (root && std::find(out->begin(),out->end(),root)==out->end()) {
                    out->push_back(root);
                    ++t_context->idleMatches;
                }
            }
            return !out->empty();
        }
        std::uint32_t* defaultWeapon(RE::Actor* actor,std::uint32_t* index,RE::BGSObjectInstance* item)
        {
            if (!t_context || actor!=t_context->player || !index || !item) return s_defaultWeapon(actor,index,item);
            if (!validContext()) { *item=RE::BGSObjectInstance{nullptr,nullptr}; *index=UINT32_MAX; return index; }
            native<RE::BGSObjectInstance*(*)(RE::Actor*,RE::BGSObjectInstance*,std::uint32_t)>(0xE50DA0)(actor,item,1);
            *index=1;
            return index;
        }
        bool prepareShot(void* launch)
        {
            // The native player path chooses the indexed muzzle, but then aims
            // toward the PRIMARY controller's target. Keep native origin/cell
            // preparation and use this gun's muzzle direction for index one.
            if (t_context && t_context->shot && !validContext()) return false;
            const bool result=s_prepareShot(launch);
            if (!result || !validContext() || !t_context->shot) return result;
            RE::Actor* actor{}; std::uint32_t index{};
            if (!native_memory::tryReadField(launch,0x20,actor) || actor!=t_context->player ||
                !native_memory::tryReadField(launch,0x48,index) || index!=1) return result;
            auto* muzzle=native<RE::NiAVObject*(*)(RE::Actor*,std::uint32_t)>(0xDE85F0)(actor,1);
            auto* primary=native<RE::NiAVObject*(*)(RE::Actor*,std::uint32_t)>(0xDE85F0)(actor,0);
            if (!muzzle || muzzle==primary) {
                ROCK_LOG_SAMPLE_WARN(Weapon,1000,"AKIMBO left shot rejected: independent native muzzle unavailable");
                return false;
            }
            // Native 104F661/104F70F read muzzle+80: stored ROW one.
            // Use ROCK's established stored-basis conversion, never column one.
            const auto direction=transform_math::rotateLocalVectorToWorld(muzzle->world.rotate,RE::NiPoint3{0,1,0});
            float heading{},pitch{};
            if (!std::isfinite(direction.x) || !std::isfinite(direction.y) || !std::isfinite(direction.z)) return false;
            native<void(*)(const RE::NiPoint3*,float*,float*)>(0x113A6D0)(&direction,&heading,&pitch);
            const bool prepared=std::isfinite(heading) && std::isfinite(pitch) &&
                native_memory::tryWriteValue(reinterpret_cast<float*>(static_cast<std::byte*>(launch)+0x4C),heading) &&
                native_memory::tryWriteValue(reinterpret_cast<float*>(static_cast<std::byte*>(launch)+0x50),pitch);
            t_context->prepared=prepared;
            return prepared;
        }
        RE::NiAVObject* muzzle(RE::Actor* actor,std::uint32_t index)
        {
            if (t_context && t_context->shot && actor==t_context->player && index==1) return validContext()?t_context->muzzle:nullptr;
            return s_muzzle(actor,index);
        }
        bool modelInt(void* biped,std::uint32_t slot,const RE::BSFixedString* name,int value)
        {
            if (t_context && !validContext()) return false;
            if (validContext() && slot==0x29 && name &&
                (biped==native_hand_equipment::playerBiped(true) || biped==native_hand_equipment::playerBiped(false))) {
                slot=native<std::uint32_t(*)(RE::Actor*,RE::TESBoundObject*,std::uint32_t)>(0x1CAFC0)(
                    t_context->player,static_cast<RE::TESObjectWEAP*>(t_context->state->item.object),1);
                if (slot>=44) return false;
                if (*name==RE::BSFixedString("iWeaponCharge"))
                    value=static_cast<int>(native<std::uint32_t(*)(void*,std::uint32_t)>(0xEC48C0)(t_context->player->currentProcess,1));
                t_context->state->graph->setInt(*name,value);
            } else if (s_installed && slot==0x29 && name && *name==RE::BSFixedString("iWeaponCharge") && native_hand_equipment::hasSecondaryGun() &&
                (biped==native_hand_equipment::playerBiped(true) || biped==native_hand_equipment::playerBiped(false))) {
                if (auto* player=f4vr::getPlayer(); player && player->currentProcess)
                    value=static_cast<int>(native<std::uint32_t(*)(void*,std::uint32_t)>(0xEC48C0)(player->currentProcess,0));
            }
            return s_modelInt(biped,slot,name,value);
        }
        std::uint32_t pulseHand(std::uint32_t hand)
        {
            if (validContext() && t_context->shot) {
                std::uint32_t secondary{};
                if (native_memory::tryReadField(t_context->player,0x8D0,secondary) && secondary<2) return secondary;
            }
            return hand;
        }
        void pulse(std::uint32_t hand,float duration,float strength)
        {
            if (t_context && t_context->shot && !validContext()) return;
            s_pulse(pulseHand(hand),duration,strength);
        }
        void pattern(std::uint32_t hand,float duration,float strength,std::uint32_t interval,std::uint32_t count)
        {
            if (t_context && t_context->shot && !validContext()) return;
            s_pattern(pulseHand(hand),duration,strength,interval,count);
        }
        const RE::TESObjectWEAP::InstanceData* weaponData(const RE::BGSObjectInstance& item)
        {
            if (!item.object || !item.object->IsWeapon()) return nullptr;
            const auto* data=item.instanceData?static_cast<const RE::TESObjectWEAP::InstanceData*>(item.instanceData.get()):
                &static_cast<const RE::TESObjectWEAP*>(item.object)->weaponData;
            return data->type==RE::WEAPON_TYPE::kGun?data:nullptr;
        }
        void setLoaded(void* process,std::uint32_t index,std::uint32_t amount)
        {
            auto* player=s_installed?f4vr::getPlayer():nullptr;
            if (player && process==player->currentProcess && index<2) {
                RE::BGSObjectInstance item{nullptr,nullptr},other{nullptr,nullptr};
                const auto read=native<RE::BGSObjectInstance*(*)(RE::Actor*,RE::BGSObjectInstance*,std::uint32_t)>(0xE50DA0);
                read(player,&item,index); read(player,&other,1-index);
                const auto* data=weaponData(item); const auto* otherData=weaponData(other);
                // FO4's ordinary ammo inventory count includes loaded rounds.
                // Native EC4B90 is the single indexed loaded-count setter used
                // by equip, reload and shooting. A refill cannot allocate rounds
                // already owned by the other loaded magazine. Charged ammo has
                // native per-item health accounting and is not a round count.
                if (data && otherData && data->ammo && data->ammo==otherData->ammo && data->ammo->data.health==0 &&
                    native<bool(*)(RE::Actor*,const RE::BGSObjectInstance*)>(0xE4EF20)(player,&item)) {
                    const auto current=native<std::uint32_t(*)(void*,std::uint32_t)>(0xEC48C0)(process,index);
                    if (amount>current) {
                        const auto reserved=native<std::uint32_t(*)(void*,std::uint32_t)>(0xEC48C0)(process,1-index);
                        const auto total=native<int(*)(RE::TESObjectREFR*,RE::TESBoundObject*)>(0x3E3750)(player,data->ammo);
                        const auto available=total>0 && static_cast<std::uint32_t>(total)>reserved?static_cast<std::uint32_t>(total)-reserved:0;
                        amount=(std::min)(amount,available);
                    }
                }
            }
            s_setLoaded(process,index,amount);
        }

        struct Action
        {
            alignas(16) std::array<std::byte,0x60> data{};
            explicit Action(RE::PlayerCharacter* player,RE::TESForm* form)
            {
                // 1715E0, E5AD50 and 1016D20 agree on this stack-owned native
                // ActionInput/TESActionData layout. Priority Try is synchronous.
                native<void*(*)(void*,int,RE::Actor*,RE::TESForm*,RE::TESObjectREFR*,std::uint32_t)>(0x1715E0)(data.data(),2,player,form,nullptr,1);
                new(data.data()+0x28) RE::BSFixedString();
                new(data.data()+0x30) RE::BSFixedString();
                *reinterpret_cast<std::uintptr_t*>(data.data())=REL::Offset(0x2C80B98).address();
            }
            ~Action()
            {
                reinterpret_cast<RE::BSFixedString*>(data.data()+0x30)->~BSFixedString();
                reinterpret_cast<RE::BSFixedString*>(data.data()+0x28)->~BSFixedString();
                native<void(*)(void*)>(0x1714D0)(data.data());
            }
            const RE::BSFixedString& event() const { return *reinterpret_cast<const RE::BSFixedString*>(data.data()+0x28); }
        };
        bool dispatch(State& s,RE::DEFAULT_OBJECT object,bool conditional=false)
        {
            auto* player=f4vr::getPlayer();
            auto* defaults=RE::BGSDefaultObjectManager::GetSingleton();
            auto* form=defaults?defaults->GetDefaultObject(object):nullptr;
            if (!player || !form || !same(s,native_hand_equipment::equippedViews()[1])) return false;
            Context context{&s,player}; Scope scope{context};
            Action action{player,form};
            const bool accepted=native<bool(*)(void*)>(0xF9BEF0)(action.data.data());
            ++s.actions;
            if (accepted || trace(s,Trace::Action)) ROCK_LOG_INFO(Weapon,"AKIMBO native action form={:08X} hand=left action={} event='{}' accepted={} actualIndex=1 managerReads={} idleQueries={} idleMatches={} conditional={} attempt={}",
                s.item.object->formID,std::to_underlying(object),action.event().c_str(),accepted,context.managerReads,context.idleQueries,context.idleMatches,conditional,s.actions);
            return accepted;
        }
        void attackState(State& s,std::uint32_t value)
        {
            if (same(s,native_hand_equipment::equippedViews()[1]))
                native<void(*)(RE::Actor*,std::uint32_t,std::uint32_t)>(0xE51910)(f4vr::getPlayer(),1,value);
        }
        std::uint32_t loaded(const View& view)
        {
            std::uint32_t value{};
            return view.data && native_memory::tryReadField(reinterpret_cast<void*>(view.data),0x18,value)?value:0;
        }
        bool emptyTrigger(State& s)
        {
            Context context{&s,f4vr::getPlayer()}; Scope scope(context);
            // Native FC0D18..FC0D99: an empty magazine with an available
            // indexed ammo selection requests reload. With no ammo, attempt
            // the optional FireEmpty action (e.g. a minigun shredder), then
            // play this exact instance's attack-fail sound independently.
            auto* ammo=native<RE::TESAmmo*(*)(RE::Actor*,std::uint32_t)>(0xE05BA0)(context.player,1);
            if (ammo) {
                const bool accepted=dispatch(s,RE::DEFAULT_OBJECT::kActionReload);
                if (accepted) s.reloading=true;
                if (accepted || trace(s,Trace::Lifecycle)) ROCK_LOG_INFO(Weapon,"AKIMBO empty trigger form={:08X} response=native-reload accepted={}",s.item.object->formID,accepted);
                return accepted;
            }
            const bool conditional=dispatch(s,RE::DEFAULT_OBJECT::kActionFireEmpty,true);
            const auto* data=weaponData(s.item);
            auto* sound=data?data->attackFailSound:nullptr;
            if (sound) native<void(*)(RE::BGSSoundDescriptorForm*,std::uint32_t,const RE::NiPoint3*,RE::NiAVObject*)>(0x81FBF0)(sound,0,nullptr,nullptr);
            if (trace(s,Trace::Lifecycle)) ROCK_LOG_INFO(Weapon,"AKIMBO empty trigger form={:08X} response=native-dry-fire conditionalAction={} sound={:08X}",
                s.item.object->formID,conditional,sound?sound->formID:0);
            return true;
        }
        void drain(State& s,bool effects)
        {
            Graph::Events events;
            const auto count=s.graph->takeEvents(events);
            for (std::size_t i=0;i<count;++i) {
                if (!same(s,native_hand_equipment::equippedViews()[1])) return;
                const auto& event=events[i];
                ++s.events;
                const bool lifecycle=event.tag==RE::BSFixedString("reloadComplete") || event.tag==RE::BSFixedString("reloadEnd") ||
                    event.tag==RE::BSFixedString("reloadStart") || event.tag==RE::BSFixedString("weaponDraw");
                if (trace(s,lifecycle?Trace::Lifecycle:Trace::Event)) ROCK_LOG_INFO(Animation,"AKIMBO private native event form={:08X} tag='{}' payload='{}' gameplay={} sequence={}",
                    s.item.object->formID,event.tag.c_str(),event.payload.c_str(),effects,s.events);
                if (!effects) continue;
                if (event.tag==RE::BSFixedString("weaponFire")) {
                    const auto views=native_hand_equipment::equippedViews();
                    const auto before=loaded(views[1]),rightBefore=loaded(views[0]);
                    const auto* data=s.item.instanceData?static_cast<const RE::TESObjectWEAP::InstanceData*>(s.item.instanceData.get()):
                        &static_cast<RE::TESObjectWEAP*>(s.item.object)->weaponData;
                    if ((data->ammo && before==0) || s.reloading) continue;
                    if (data->ammo && data->ammo->data.health==0 &&
                        native<bool(*)(RE::Actor*,const RE::BGSObjectInstance*)>(0xE4EF20)(f4vr::getPlayer(),&s.item) &&
                        native<int(*)(RE::TESObjectREFR*,RE::TESBoundObject*)>(0x3E3750)(f4vr::getPlayer(),data->ammo)<=0) continue;
                    Context context{&s,f4vr::getPlayer(),true}; Scope scope{context};
                    // Native 3331F0 searches the supplied attachment first. Both
                    // roots are this exact equipped model, so its fallback can
                    // never find a same-named muzzle on the other firearm.
                    context.muzzle=native<RE::NiAVObject*(*)(RE::TESObjectWEAP*,RE::NiAVObject*,RE::NiAVObject*,bool)>(0x3331F0)(
                        static_cast<RE::TESObjectWEAP*>(s.item.object),views[1].root,views[1].root,true);
                    if (!context.muzzle) {
                        ROCK_LOG_SAMPLE_WARN(Weapon,1000,"AKIMBO left shot rejected: native model has no independent muzzle form={:08X}",s.item.object->formID);
                        continue;
                    }
                    const RE::BSFixedString index{"1"};
                    native<bool(*)(void*,RE::Actor*,const RE::BSFixedString*)>(0xFF2A40)(nullptr,context.player,&index);
                    const auto after=native_hand_equipment::equippedViews();
                    ++s.shots;
                    if (trace(s,Trace::Fire) || loaded(after[1])==0 || loaded(after[0])!=rightBefore || !context.prepared) ROCK_LOG_INFO(Weapon,
                        "AKIMBO native fire event form={:08X} hand=left prepared={} loaded={}->{} rightLoaded={}->{} index=1 sequence={}",
                        s.item.object->formID,context.prepared,before,loaded(after[1]),rightBefore,loaded(after[0]),s.shots);
                } else if (event.tag==RE::BSFixedString("reloadComplete")) {
                    const auto before=native_hand_equipment::equippedViews();
                    const auto leftBefore=loaded(before[1]),rightBefore=loaded(before[0]);
                    Context context{&s,f4vr::getPlayer()}; Scope scope{context};
                    const RE::BSFixedString index{"1"};
                    native<bool(*)(void*,RE::Actor*,const RE::BSFixedString*)>(0xFF2C10)(nullptr,context.player,&index);
                    const auto after=native_hand_equipment::equippedViews();
                    if (loaded(after[1])!=leftBefore || loaded(after[0])!=rightBefore || trace(s,Trace::Lifecycle)) ROCK_LOG_INFO(Weapon,
                        "AKIMBO native reload complete form={:08X} loaded={}->{} rightLoaded={}->{} index=1",
                        s.item.object->formID,leftBefore,loaded(after[1]),rightBefore,loaded(after[0]));
                } else if (event.tag==RE::BSFixedString("reloadEnd")) {
                    if (s.reloading) ROCK_LOG_INFO(Weapon,"AKIMBO native reload ended form={:08X} loaded={} eventSequence={}",
                        s.item.object->formID,loaded(native_hand_equipment::equippedViews()[1]),s.events);
                    s.reloading=false;
                } else if (event.tag==RE::BSFixedString("attackStop")) {
                    attackState(s,0);
                }
            }
        }
        void cancel(State& s)
        {
            if (s.trigger.cancel() && s.ready && s.graph) { (void)s.graph->notify(RE::BSFixedString("attackRelease")); attackState(s,0); }
            s.reload=false;
            if (s.graph) drain(s,false);
        }
    }

    bool install() noexcept
    {
        if (s_installed) return true;
        if (!Graph::validate()) return false;
        std::uintptr_t getter{};
        const auto slot=REL::Offset(0x2D819E8).address();
        if (!native_memory::tryReadValue(reinterpret_cast<const std::uintptr_t*>(slot),getter) || getter!=REL::Offset(0xE14FD0).address() ||
            !matches(0xE50DA0,std::to_array<std::uint8_t>({0x40,0x53,0x48,0x83,0xEC,0x20,0x48,0x8B,0xDA})) ||
            !matches(0xFF2A40,std::to_array<std::uint8_t>({0x48,0x89,0x5C,0x24,0x08})) ||
            !matches(0xFF2C10,std::to_array<std::uint8_t>({0x48,0x89,0x5C,0x24,0x08})) ||
            !matches(0x595760,std::to_array<std::uint8_t>({0x40,0x56,0x48,0x83,0xEC,0x20,0x48,0x8B,0xF2})) ||
            !matches(0xDE85F0,std::to_array<std::uint8_t>({0x48,0x89,0x5C,0x24,0x20,0x89,0x54,0x24,0x10})) ||
            !matches(0x3331F0,std::to_array<std::uint8_t>({0x48,0x89,0x5C,0x24,0x20,0x57,0x48,0x83,0xEC,0x40})) ||
            !matches(0x64DE20,std::to_array<std::uint8_t>({0x48,0x89,0x5C,0x24,0x08,0x57,0x48,0x83,0xEC,0x30})) ||
            !matches(0x113A6D0,std::to_array<std::uint8_t>({0x48,0x89,0x5C,0x24,0x08,0x48,0x89,0x74,0x24,0x10})) ||
            !matches(0x1715E0,std::to_array<std::uint8_t>({0x48,0x8D,0x05,0xD1,0xFC,0xB1,0x02,0x48,0x89,0x01})) ||
            !matches(0x1714D0,std::to_array<std::uint8_t>({0x48,0x89,0x5C,0x24,0x08,0x57,0x48,0x83,0xEC,0x20})) ||
            !matches(0xF9BEF0,std::to_array<std::uint8_t>({0x48,0x8B,0xD1,0x48,0x8B,0x0D,0x76,0x88,0xB6,0x04,0xE9,0x01,0x93,0xED,0xFF})) ||
            !matches(0x812EA0,std::to_array<std::uint8_t>({0x40,0x53,0x48,0x83,0xEC,0x20,0x48,0x8B,0x41,0x18})) ||
            !matches(0x1690440,std::to_array<std::uint8_t>({0x4C,0x8D,0x49,0x10,0xE9,0x97,0x10,0,0})) ||
            !matches(0x16906A0,std::to_array<std::uint8_t>({0x4C,0x8D,0x41,0x10,0xE9,0xD7,0x25,0,0})) ||
            !matches(0xEC48C0,std::to_array<std::uint8_t>({0x40,0x53,0x48,0x83,0xEC,0x20,0x4C,0x8D,0x44,0x24,0x48})) ||
            !matches(0x3E3750,std::to_array<std::uint8_t>({0x40,0x53,0x48,0x83,0xEC,0x20,0x48,0x8B,0xDA})) ||
            !matches(0xE4EF20,std::to_array<std::uint8_t>({0x48,0x89,0x5C,0x24,0x08,0x57,0x48,0x83,0xEC,0x20})) ||
            !matches(0xE05BA0,std::to_array<std::uint8_t>({0x48,0x83,0xEC,0x28,0x48,0x8B,0x89,0,3,0,0})) ||
            !matches(0x81FBF0,std::to_array<std::uint8_t>({0x48,0x83,0xEC,0x48,0x4C,0x89,0x4C,0x24,0x20,0x4D,0x8B,0xC8,0x44,0x8B,0xC2})) ||
            !matches(0xE51910,std::to_array<std::uint8_t>({0x89,0x54,0x24,0x10,0x53,0x55,0x56,0x57,0x41,0x56}))) return false;
        void* original{};
        constexpr auto rootsPrefix=std::to_array<std::uint8_t>({0x48,0x8B,0xC4,0x53,0x55,0x57,0x41,0x57,0x48,0x83,0xEC,0x58,0x48,0x89,0x70,0x08});
        if (!entry_trampoline_hook::install("Equipped secondary native idle roots",0x595800,rootsPrefix.data(),rootsPrefix.size(),reinterpret_cast<void*>(&idleRoots),original)) return false;
        s_idleRoots=reinterpret_cast<IdleRoots>(original);
        constexpr auto indexPrefix=std::to_array<std::uint8_t>({0x48,0x89,0x74,0x24,0x20,0x57,0x41,0x56,0x41,0x57,0x48,0x83,0xEC,0x50});
        if (!entry_trampoline_hook::install("Equipped secondary default action index",0xE50E70,indexPrefix.data(),indexPrefix.size(),reinterpret_cast<void*>(&defaultWeapon),original)) return false;
        s_defaultWeapon=reinterpret_cast<DefaultWeapon>(original);
        constexpr auto shotPrefix=std::to_array<std::uint8_t>({0x48,0x8B,0xC4,0x55,0x57,0x41,0x54,0x41,0x55,0x48,0x8D,0xA8,0x58,0xFF,0xFF,0xFF});
        if (!entry_trampoline_hook::install("Equipped secondary native muzzle direction",0x104F3A0,shotPrefix.data(),shotPrefix.size(),reinterpret_cast<void*>(&prepareShot),original)) return false;
        s_prepareShot=reinterpret_cast<PrepareShot>(original);
        constexpr auto muzzlePrefix=std::to_array<std::uint8_t>({0x48,0x89,0x5C,0x24,0x20,0x89,0x54,0x24,0x10,0x57,0x48,0x83,0xEC,0x50});
        if (!entry_trampoline_hook::install("Equipped secondary native muzzle ownership",0xDE85F0,muzzlePrefix.data(),muzzlePrefix.size(),reinterpret_cast<void*>(&muzzle),original)) return false;
        s_muzzle=reinterpret_cast<Muzzle>(original);
        constexpr auto modelPrefix=std::to_array<std::uint8_t>({0x48,0x63,0xC2,0x48,0x83,0xC1,0x10,0x4D,0x8B,0xD0,0x48,0x6B,0xC0,0x58});
        if (!entry_trampoline_hook::install("Equipped secondary model variable ownership",0x1C9A90,modelPrefix.data(),modelPrefix.size(),reinterpret_cast<void*>(&modelInt),original)) return false;
        s_modelInt=reinterpret_cast<ModelInt>(original);
        constexpr auto pulsePrefix=std::to_array<std::uint8_t>({0x40,0x57,0x48,0x83,0xEC,0x50,0x0F,0x29,0x74,0x24,0x40,0x0F,0x57,0xC0});
        if (!entry_trampoline_hook::install("Equipped secondary firing haptics",0x1BA9BF0,pulsePrefix.data(),pulsePrefix.size(),reinterpret_cast<void*>(&pulse),original)) return false;
        s_pulse=reinterpret_cast<Pulse>(original);
        constexpr auto patternPrefix=std::to_array<std::uint8_t>({0x48,0x8B,0xC4,0x55,0x57,0x48,0x83,0xEC,0x78,0x0F,0x29,0x70,0xD8,0x0F,0x57,0xC0});
        if (!entry_trampoline_hook::install("Equipped secondary firing haptic patterns",0x1BA9D20,patternPrefix.data(),patternPrefix.size(),reinterpret_cast<void*>(&pattern),original)) return false;
        s_pattern=reinterpret_cast<Pattern>(original);
        constexpr auto ammoPrefix=std::to_array<std::uint8_t>({0x40,0x53,0x48,0x83,0xEC,0x30,0x41,0x8B,0xD8,0x4C,0x8D,0x44,0x24,0x20});
        if (!entry_trampoline_hook::install("Equipped magazines shared reserve",0xEC4B90,ammoPrefix.data(),ammoPrefix.size(),reinterpret_cast<void*>(&setLoaded),original)) return false;
        s_setLoaded=reinterpret_cast<SetLoaded>(original);
        if (!matches(0xEF96B0,std::to_array<std::uint8_t>({0x40,0x53,0x48,0x83,0xEC,0x20})) ||
            !matches(0xEFD0B0,std::to_array<std::uint8_t>({0x40,0x53,0x48,0x83,0xEC,0x50})) ||
            !matches(0xEFD1C0,std::to_array<std::uint8_t>({0x48,0x89,0x5C,0x24,0x08,0x57,0x48,0x83,0xEC,0x20})) ||
            !matches(0xE526A0,std::to_array<std::uint8_t>({0x48,0x83,0xEC,0x28,0x48,0x8B,0x89,0,3,0,0})) ||
            !installRecoilCalls()) {
            ROCK_LOG_ERROR(Init,"AKIMBO native recoil routing validation failed");
            return false;
        }
        s_getManager=reinterpret_cast<GetManager>(getter);
        const auto replacement=reinterpret_cast<std::uintptr_t>(&getManager);
        REL::safe_write(slot,replacement);
        std::uintptr_t readback{};
        if (!native_memory::tryReadValue(reinterpret_cast<const std::uintptr_t*>(slot),readback) || readback!=replacement) {
            REL::safe_write(slot,getter); return false;
        }
        s_installed=true;
        ROCK_LOG_INFO(Init,"Native secondary equipped actions installed: independent behavior, native item index, scoped player graph, native muzzle, per-item recoil");
        return true;
    }

    void update(RE::NiNode* attachment,float dt) noexcept
    {
        if (!s_installed) return;
        auto& s=state();
        if (!s.thread) s.thread=GetCurrentThreadId();
        if (s.thread!=GetCurrentThreadId()) return;
        try {
            const auto& frame=runtime_state::currentFrame();
            if (s.frame==frame.frameIndex) return;
            s.frame=frame.frameIndex;
            const auto view=native_hand_equipment::equippedViews()[1];
            input_remap_runtime::setSecondaryEquippedTriggerOwned(view.weapon && frame.weaponDrawn);
            if (s.graph && !same(s,view)) { cancel(s); s.ready=false; s.retiring=true; }
            if (s.retiring) {
                if (!s.graph->canDestroy()) return;
                releaseModelGraphs(s,false);
                s.bindings.restore();
                s.graph.reset(); s.model.reset(); s.item=RE::BGSObjectInstance{nullptr,nullptr};
                s.recoilNode.reset(); s.kick={}; s.kickActive=false; s.recoilFrame=UINT64_MAX;
                s.retiring=false; s.failed=false; s.reloading=false;
            }
            if (!attachment || !view.weapon || !view.root || frame.localMenuBlocking || !frame.weaponDrawn || !frame.visualAuthorityAvailable) { cancel(s); return; }
            const auto trigger=input_remap_runtime::peekRawButtonState(true,input_remap_policy::kOpenVrSteamVrTriggerButtonId);
            const bool providerSuppressed=input_remap_runtime::isProviderOpenVrGameInputSuppressedForHand(true);
            const bool menu=input_remap_runtime::isMenuInputActive();
            const bool fresh=trigger.sampleAgeMilliseconds<=100;
            const bool allowed=trigger.available && fresh && !providerSuppressed && !menu;
            if (s.trigger.observe(trigger.held,allowed) && s.ready && s.graph) {
                (void)s.graph->notify(RE::BSFixedString("attackRelease"));
                if (!allowed) attackState(s,0);
            }
            if (!s.graph) {
                s.item=RE::BGSObjectInstance(view.weapon,view.instance); s.model.reset(view.root); s.data=view.data;
                s.graph=std::make_unique<Graph>(); s.actions=s.events=s.shots=s.inputTraces=0;
                s.inputMask=UINT32_MAX; s.tracedStage=nullptr;
                s.recoilShots=s.recoilSamples=0;
                s.traces={};
                const auto* slot=view.weapon->GetEquipSlot(view.instance);
                const auto classification=weapon_instance_classification::classifyEquippedWeapon(view.weapon,view.instance,slot?slot->formID:0);
                s.recoilFamily=weapon_recoil_policy::classifyFamily({view.weapon->formID,classification.keywordFlags,
                    classification.sizeClass,classification.source,classification.resolved});
                if (!s.graph->start(f4vr::getPlayer(),s.item)) {
                    s.failed=true;
                    ROCK_LOG_ERROR(Animation,"AKIMBO secondary native graph start failed form={:08X} stage={}",view.weapon->formID,s.graph->stage());
                }
            }
            if (s.failed) return;
            if (!s.ready) {
                const auto loading=s.graph->advance();
                if (s.tracedStage!=s.graph->stage()) {
                    s.tracedStage=s.graph->stage();
                    ROCK_LOG_INFO(Animation,"AKIMBO secondary initialization form={:08X} stage={} result={}",
                        view.weapon->formID,s.tracedStage,static_cast<int>(loading));
                }
                if (loading==Graph::Load::Pending) return;
                if (loading==Graph::Load::Failed) {
                    s.failed=true;
                    ROCK_LOG_ERROR(Animation,"AKIMBO secondary native graph preparation failed form={:08X} stage={}; left actions rejected",view.weapon->formID,s.graph->stage());
                    return;
                }
                s.ready=true;
                s.recoilNode=native_scene::createEngineNiNode(0);
                if (!s.recoilNode) {
                    s.failed=true; s.ready=false;
                    ROCK_LOG_ERROR(Animation,"AKIMBO private recoil node creation failed form={:08X}",view.weapon->formID);
                    return;
                }
                if (!dispatch(s,RE::DEFAULT_OBJECT::kActionForceEquip)) {
                    s.failed=true; s.ready=false;
                    ROCK_LOG_ERROR(Animation,"AKIMBO private graph rejected native force-equip form={:08X}",view.weapon->formID);
                    return;
                }
                s.graph->update(0.0f);
                drain(s,false);
                const bool placed=s.graph->place(attachment->world);
                const bool bound=placed && s.bindings.bind(s.model.get(),*s.graph);
                const bool isolated=bound && isolateModelGraphs(s);
                if (!isolated) {
                    s.bindings.restore(); releaseModelGraphs(s,true);
                    s.failed=true; s.ready=false;
                    ROCK_LOG_ERROR(Animation,"AKIMBO private pose/graph ownership failed form={:08X} placed={} bound={} isolated={}; actions rejected",
                        view.weapon->formID,placed,bound,isolated);
                    return;
                }
                ROCK_LOG_INFO(Weapon,"AKIMBO secondary firearm ready form={:08X} instance={:#x} loaded={} graph={:#x} authority=equipped-index-1",
                    view.weapon->formID,reinterpret_cast<std::uintptr_t>(view.instance),loaded(view),reinterpret_cast<std::uintptr_t>(s.graph->manager()));
            }
            if (!isolateModelGraphs(s)) { cancel(s); return; }
            const auto* data=s.item.instanceData?static_cast<const RE::TESObjectWEAP::InstanceData*>(s.item.instanceData.get()):&view.weapon->weaponData;
            const float speed=native<float(*)(RE::Actor*,RE::BGSObjectInstance*,std::uint32_t)>(0x64DE20)(f4vr::getPlayer(),&s.item,1);
            if (!std::isfinite(speed) || speed<=0) { cancel(s); return; }
            s.graph->setFloat(RE::BSFixedString("weaponSpeedMult"),speed);
            s.graph->setFloat(RE::BSFixedString("ReloadSpeedMult"),data->reloadSpeed);
            const std::uint32_t inputMask=static_cast<std::uint32_t>(trigger.available) | (static_cast<std::uint32_t>(fresh)<<1) |
                (static_cast<std::uint32_t>(providerSuppressed)<<2) | (static_cast<std::uint32_t>(menu)<<3) |
                (static_cast<std::uint32_t>(trigger.held)<<4) | (static_cast<std::uint32_t>(s.trigger.armed())<<5) |
                (static_cast<std::uint32_t>(s.reloading)<<6);
            if (inputMask!=s.inputMask) {
                s.inputMask=inputMask;
                ++s.inputTraces;
                if (trace(s,Trace::Input)) ROCK_LOG_INFO(Weapon,
                    "AKIMBO secondary input form={:08X} available={} ageMs={} providerSuppressed={} menu={} held={} armed={} reloading={} allowed={}",
                    view.weapon->formID,trigger.available,trigger.sampleAgeMilliseconds,providerSuppressed,menu,trigger.held,s.trigger.armed(),s.reloading,allowed);
            }
            if (!allowed) { cancel(s); return; }
            if (s.reload) {
                s.reload=false;
                if (!s.reloading && dispatch(s,RE::DEFAULT_OBJECT::kActionReload)) { s.reloading=true; (void)s.trigger.cancel(); }
            }
            if (s.trigger.wantsStart() && !s.reloading) {
                if (data->ammo && !loaded(view) && !data->flags.any(RE::WEAPON_FLAGS::kChargingReload)) {
                    if (emptyTrigger(s)) s.trigger.accepted(false);
                } else {
                    auto action=RE::DEFAULT_OBJECT::kActionFireSingle;
                    if (data->flags.any(RE::WEAPON_FLAGS::kChargingAttack)) action=RE::DEFAULT_OBJECT::kActionFireCharge;
                    else if (data->flags.any(RE::WEAPON_FLAGS::kAutomatic)) action=RE::DEFAULT_OBJECT::kActionFireAuto;
                    if (dispatch(s,action)) { attackState(s,15); s.trigger.accepted(true); }
                }
            }
            s.graph->update(dt);
            if (!s.graph->place(attachment->world) || s.graph->eventOverflow()) {
                cancel(s); s.bindings.restore(); releaseModelGraphs(s,true); s.failed=true; s.ready=false;
                ROCK_LOG_ERROR(Animation,"AKIMBO secondary graph stopped: invalid pose or event overflow form={:08X}",view.weapon->formID);
                return;
            }
            if (!s.bindings.present()) {
                cancel(s);
                ROCK_LOG_SAMPLE_WARN(Animation,1000,"AKIMBO secondary part transport unavailable form={:08X}",view.weapon->formID);
                return;
            }
            drain(s,true);
        } catch (...) {
            s.bindings.restore(); releaseModelGraphs(s,true);
            s.failed=true; s.ready=false; (void)s.trigger.cancel();
            OutputDebugStringA("ROCK: independent equipped secondary action failed\n");
        }
    }
    void carryPresentation(const RE::NiTransform& previous,const RE::NiTransform& target) noexcept
    {
        if (!s_installed) return;
        auto& s=state();
        if (s.thread!=GetCurrentThreadId() || !s.ready || !same(s,native_hand_equipment::equippedViews()[1])) return;
        s.bindings.carryPublished(weapon_visual_authority_math::makePresentationWorldDelta(previous,target));
    }
    void present(RE::NiNode* attachment) noexcept
    {
        auto& s=state();
        if (s.thread!=GetCurrentThreadId() || !s.ready || !s.graph || !attachment || !same(s,native_hand_equipment::equippedViews()[1])) return;
        if (s.graph->place(attachment->world) && !s.bindings.present())
            ROCK_LOG_SAMPLE_WARN(Animation,1000,"AKIMBO secondary late part transport unavailable form={:08X}",s.item.object->formID);
    }
    void suspend() noexcept
    {
        if (!s_installed) return;
        auto& s=state();
        if (s.thread==GetCurrentThreadId()) {
            try { cancel(s); update(nullptr,0.0f); } catch (...) { (void)s.trigger.cancel(); }
        }
    }
    bool requestReload() noexcept
    {
        if (!s_installed) return false;
        auto& s=state();
        if (s.thread!=GetCurrentThreadId() || !s.ready || !same(s,native_hand_equipment::equippedViews()[1])) return false;
        s.reload=true;
        return true;
    }
    void applyRecoil(RE::NiTransform& weapon,RE::NiTransform& hand,const RE::NiTransform& controller) noexcept
    {
        if (!s_installed) return;
        auto& s=state();
        if (s.thread!=GetCurrentThreadId() || !s.ready || !s.recoilNode || !same(s,native_hand_equipment::equippedViews()[1])) return;
        const auto& frame=runtime_state::currentFrame();
        if (frame.localMenuBlocking || !frame.weaponDrawn) return;
        const auto* nodes=f4vr::getPlayerNodes();
        const auto* kick=nodes?nodes->primaryWeaponKickbackRecoilNode:nullptr;
        const auto* parent=kick?kick->parent:nullptr;
        if (!parent || !nodes->primaryWandNode) return;
        try {
            if (s.recoilFrame!=frame.frameIndex) {
                s.recoilFrame=frame.frameIndex;
                s.recoilNode->local.MakeIdentity();
                Context context{&s,f4vr::getPlayer()}; context.recoil=true;
                Scope scope(context);
                RecoilScope record(s,context.player);
                if (!record) {
                    ROCK_LOG_SAMPLE_WARN(Weapon,1000,"AKIMBO secondary native recoil evaluation unavailable");
                    return;
                }
                // Native callers EF616D/EF618F pass a separate owned reference
                // into each function; the callee consumes that reference.
                for (const auto entry:{0xEFD0B0,0xEFD1C0}) {
                    auto* node=s.recoilNode.get();
                    node->IncRefCount();
                    native<void(*)(RE::PlayerCharacter*,RE::NiNode**)>(entry)(context.player,&node);
                }
            }
            if (g_rockConfig.rockEmergencyDisableRecoil) return;
            using namespace weapon_recoil_policy;
            float percent=100.0f;
            switch (s.recoilFamily) {
            case Family::Laser: percent=g_rockConfig.rockLaserRecoilPercent; break;
            case Family::Pistol: percent=g_rockConfig.rockPistolOneHandRecoilPercent; break;
            case Family::Rifle: percent=g_rockConfig.rockRifleOneHandRecoilPercent; break;
            case Family::Shotgun: percent=g_rockConfig.rockShotgunOneHandRecoilPercent; break;
            case Family::Heavy: percent=g_rockConfig.rockHeavyOneHandRecoilPercent; break;
            default: percent=g_rockConfig.rockDefaultOneHandRecoilPercent; break;
            }
            const auto profile=selectProfile(f4vr::isInPowerArmor(),false,false);
            const auto gains=g_rockConfig.rockImmersiveRecoil?effectiveGains(s.recoilFamily,profile,percent):ProfileGains{1,1};
            RE::NiTransform controlled{};
            if (!weapon_recoil_authority_math::tryBuildControlledKick(s.recoilNode->local,gains,controlled)) {
                ROCK_LOG_SAMPLE_WARN(Weapon,1000,"AKIMBO secondary native recoil pose invalid");
                return;
            }
            const auto* handed=f4vr::getIniSetting("bLeftHandedMode:VR");
            if (!handed) return;
            const auto carrier=weapon_recoil_authority_math::resolveCarrierLocalDelta(
                controlled,parent->world,nodes->primaryWandNode->world,!handed->GetBinary());
            const auto delta=weapon_recoil_authority_math::resolveCarrierWorldDelta(carrier,controller);
            if (!holding_grip_math::valid(holding_grip_math::fromNative(delta))) return;
            weapon_recoil_authority_math::applyOneHandKick(delta,weapon,hand,weapon,hand);
            if (weapon_recoil_authority_math::hasKick(controlled) && (++s.recoilSamples,trace(s,Trace::Recoil)))
                ROCK_LOG_INFO(Weapon,"AKIMBO recoil delivered form={:08X} physicalHand=left profile={} family={} nativeT=({:.4f},{:.4f},{:.4f}) primaryRecord=restored",
                    s.item.object->formID,name(profile),name(s.recoilFamily),s.recoilNode->local.translate.x,s.recoilNode->local.translate.y,s.recoilNode->local.translate.z);
        } catch (...) {
            ROCK_LOG_SAMPLE_WARN(Weapon,1000,"AKIMBO secondary recoil delivery failed");
        }
    }
}
