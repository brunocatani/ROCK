#include "physics-interaction/weapon/NativeSecondaryActions.h"
#include "physics-interaction/weapon/NativeEquippedActionGraph.h"
#include "physics-interaction/weapon/NativeHandEquipment.h"
#include "physics-interaction/weapon/NativeSecondaryBoneBindings.h"
#include "physics-interaction/weapon/NativeEquippedAnimationAudio.h"
#include "physics-interaction/weapon/NativeInventoryHandSlots.h"
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
#include "RE/Bethesda/BSAnimationGraph.h"
#include "RE/Bethesda/PlayerCharacter.h"
#include "RE/Bethesda/TESBoundObjects.h"
#include "RE/Bethesda/BGSDefaultObjectManager.h"
#include <Windows.h>
#include <algorithm>
#include <array>
#include <atomic>
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
        using SelectIdle=RE::TESForm*(*)(RE::TESForm*,void*,void*);
        GetManager s_getManager{};
        IdleRoots s_idleRoots{};
        DefaultWeapon s_defaultWeapon{};
        PrepareShot s_prepareShot{};
        Muzzle s_muzzle{};
        ModelInt s_modelInt{};
        Pulse s_pulse{};
        Pattern s_pattern{};
        // Physical hand of the index-1 weapon. Frame writer (update), read by
        // the native haptic queue hooks on the game's calling thread.
        std::atomic<bool> s_handLeft{true};
        std::atomic<bool> s_drawn{false}; // Same writer/readers: index 1 equipped and drawn.
        const char* handName() noexcept { return s_handLeft.load(std::memory_order_relaxed) ? "left" : "right"; }
        SetLoaded s_setLoaded{};
        SelectIdle s_selectIdle{};
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
            native_equipped_animation_audio::Channel audio;
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
            bool rootsTraced{};
            struct TraceWindow { std::uint64_t second{UINT64_MAX}; std::uint32_t count{},suppressed{}; };
            std::array<TraceWindow,7> traces{};
        };
        // Refusal: condition/idle dumps of refused actions. Separate from
        // Lifecycle so a held trigger on a refused gun cannot suppress the
        // reload and draw events logged in the same second.
        enum class Trace : std::size_t { Action,Event,Lifecycle,Fire,Recoil,Input,Refusal };
        bool trace(State& s,Trace channel,std::uint32_t limit=8)
        {
            auto& window=s.traces[static_cast<std::size_t>(channel)];
            const auto second=GetTickCount64()/1000;
            if (window.second!=second) {
                if (window.suppressed) ROCK_LOG_INFO(Weapon,"AKIMBO diagnostic sampling form={:08X} channel={} suppressed={} windowSeconds=1",
                    s.item.object?s.item.object->formID:0,static_cast<std::size_t>(channel),window.suppressed);
                window={second,0,0};
            }
            if (window.count<limit) { ++window.count; return true; }
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
            std::uint32_t managerReads{},idleQueries{},idleMatches{},cacheReads{};
            bool recoil{};
            struct Query {
                RE::BSFixedString name;
                std::uint32_t id{UINT32_MAX};
                double value{};
                bool accepted{};
                bool actorState{};
            };
            std::array<Query,16> queries{};
            std::size_t queryCount{};
            struct IdleQuery { std::uint32_t candidate{},selected{}; std::size_t queryBegin{},queryEnd{}; };
            std::array<IdleQuery,32> idles{};
            std::size_t idleCount{};
            bool traceReload{};
            // Equipment identity resolved once per synchronous native call.
            bool identityChecked{},identityMatched{};
        };
        thread_local Context* t_context{};
        // Set only while one actor-level variable getter is re-run against the
        // player's own AIProcess cache (see variableQuery).
        thread_local bool t_actorVariableLookup{};
        struct Scope
        {
            Context* previous;
            native_inventory_hand_slots::Scope inventoryOwner;
            explicit Scope(Context& context):previous(std::exchange(t_context,&context)),inventoryOwner(context.player) {}
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
                if (manager==s.modelManagers[view]) {
                    // Rebuilding the other hand can register this unchanged
                    // native model manager with the actor again. Identity is
                    // not proof of exclusive event ownership. The native
                    // detach lookup is allocation-free when it is absent.
                    if (manager && native<bool(*)(RE::BSAnimationGraphManager*,ManagerRef*)>(0x16906A0)(parent.get(),&manager)) {
                        native<void(*)(RE::BSAnimationGraphManager*,ManagerRef*,bool)>(0x1690440)(s.graph->manager(),&manager,true);
                        ROCK_LOG_SAMPLE_INFO(Animation,1000,"AKIMBO model graph reattachment isolated form={:08X} view={} manager={:#x}",
                            s.item.object->formID,view,reinterpret_cast<std::uintptr_t>(manager.get()));
                    }
                    continue;
                }
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
            if (!s_installed || !t_context || t_context->player!=f4vr::getPlayer() ||
                !t_context->state->ready || t_context->state->thread!=GetCurrentThreadId()) return false;
            // A scope spans one synchronous native action, shot, reload or
            // recoil call, and ROCK never equips inside it. Every hooked lookup
            // in that call (variable getters fire dozens of times) shares the
            // identity resolved at its first lookup instead of re-reading the
            // equipment and biped records through guarded page queries.
            if (!t_context->identityChecked) {
                t_context->identityMatched=same(*t_context->state,native_hand_equipment::equippedViews()[1]);
                t_context->identityChecked=true;
            }
            return t_context->identityMatched;
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
        RE::BSAnimationGraphVariableCache* variableCache(void* process)
        {
            if (t_context && !t_actorVariableLookup && process==t_context->player->currentProcess) {
                if (!validContext()) return nullptr;
                auto* manager=t_context->state->graph->manager();
                if (!manager || manager->activeGraph>=manager->graph.size() ||
                    manager->variableCache.graphToCacheFor!=manager->graph[manager->activeGraph] ||
                    !manager->variableCache.lock || manager->variableCache.variableQuickLookup.empty()) return nullptr;
                ++t_context->cacheReads;
                return &manager->variableCache;
            }
            return native<RE::BSAnimationGraphVariableCache*(*)(void*)>(0xEC5590)(process);
        }
        template<std::uintptr_t Rva,class Key,class Value>
        bool variableQuery(void* holder,Key key,Value* output)
        {
            bool accepted=native<bool(*)(void*,Key,Value*)>(Rva)(holder,key,output);
            const bool player=t_context && holder==static_cast<RE::IAnimationGraphManagerHolder*>(t_context->player);
            bool actorState=false;
            if (!accepted && player && validContext()) {
                // Actor getters read AIProcess+8 -> +268, the player's own
                // graph cache. The private weapon graph does not define every
                // actor-level variable native conditions read: a left reload
                // idle requires isReloading, which only the actor cache
                // answers (verified: 18:48 reloads selected 000BDDA6 with
                // isReloading from the actor; without this, nothing selects).
                // Weapon variables still come from this hand's graph first.
                t_actorVariableLookup=true;
                accepted=native<bool(*)(void*,Key,Value*)>(Rva)(holder,key,output);
                t_actorVariableLookup=false;
                actorState=accepted;
            }
            if (player && t_context->queryCount<t_context->queries.size()) {
                auto& query=t_context->queries[t_context->queryCount++];
                if constexpr (std::is_integral_v<Key>) query.id=key;
                else if (key) query.name=*key;
                query.accepted=accepted;
                query.actorState=actorState;
                if (accepted && output) query.value=static_cast<double>(*output);
            }
            return accepted;
        }
        bool tracingPrimaryReload() noexcept;
        bool primaryReloadTraceLine() noexcept;
        RE::TESForm* selectIdle(RE::TESForm* candidate,void* conditions,void* history)
        {
            // Trace the native decision tree without changing its conditions,
            // cooldown list or result. Keep the complete parent/child query
            // spans for a rejected reload, including non-variable conditions.
            auto* context=t_context;
            Context::IdleQuery* query=nullptr;
            if (context && context->traceReload && context->idleCount<context->idles.size()) {
                query=&context->idles[context->idleCount++];
                query->candidate=candidate?candidate->formID:0;
                query->queryBegin=context->queryCount;
            }
            auto* result=s_selectIdle(candidate,conditions,history);
            if (query) { query->selected=result?result->formID:0; query->queryEnd=context->queryCount; }
            else if (!context && tracingPrimaryReload() && primaryReloadTraceLine())
                ROCK_LOG_INFO(Animation,"AKIMBO primary reload trace idle candidate={:08X} selected={:08X}",
                    candidate?candidate->formID:0,result?result->formID:0);
            return result;
        }
        bool installVariableQueryTrace()
        {
            constexpr std::array<std::uintptr_t,6> originals{0xE15290,0xE152F0,0xE15350,0xE15230,0xE151D0,0xE15170};
            const std::array<std::uintptr_t,6> hooks{
                reinterpret_cast<std::uintptr_t>(&variableQuery<0xE15290,std::uint32_t,float>),
                reinterpret_cast<std::uintptr_t>(&variableQuery<0xE152F0,std::uint32_t,bool>),
                reinterpret_cast<std::uintptr_t>(&variableQuery<0xE15350,std::uint32_t,std::int32_t>),
                reinterpret_cast<std::uintptr_t>(&variableQuery<0xE15230,const RE::BSFixedString*,float>),
                reinterpret_cast<std::uintptr_t>(&variableQuery<0xE151D0,const RE::BSFixedString*,std::int32_t>),
                reinterpret_cast<std::uintptr_t>(&variableQuery<0xE15170,const RE::BSFixedString*,bool>)};
            const auto table=REL::Offset(0x2D819C8).address()+0x80;
            for (std::size_t i=0;i<originals.size();++i) {
                std::uintptr_t value{};
                if (!native_memory::tryReadValue(reinterpret_cast<const std::uintptr_t*>(table+i*8),value) || value!=REL::Offset(originals[i]).address()) return false;
            }
            for (std::size_t i=0;i<hooks.size();++i) {
                REL::safe_write(table+i*8,hooks[i]);
                std::uintptr_t value{};
                if (!native_memory::tryReadValue(reinterpret_cast<const std::uintptr_t*>(table+i*8),value) || value!=hooks[i]) {
                    for (std::size_t j=0;j<=i;++j) REL::safe_write(table+j*8,REL::Offset(originals[j]).address());
                    return false;
                }
            }
            return true;
        }
        bool installVariableCacheCalls()
        {
            // Actor's six variable getters bypass GetAnimationGraphManagerImpl
            // and read AIProcess+8 -> +268 directly. Native graph activation
            // builds the SAME named/fixed-ID cache on our private manager.
            constexpr std::array<std::uintptr_t,6> calls{0xE1518C,0xE151EC,0xE1524C,0xE152AB,0xE1530B,0xE1536B};
            std::array<std::array<std::uint8_t,5>,calls.size()> originals{};
            for (std::size_t i=0;i<calls.size();++i) {
                const auto displacement=static_cast<std::int32_t>(0xEC5590-calls[i]-5);
                originals[i][0]=0xE8;
                std::memcpy(originals[i].data()+1,&displacement,4);
                if (!matches(calls[i],originals[i])) return false;
            }
            std::size_t patched=0;
            try {
                auto& trampoline=F4SE::GetTrampoline();
                if (trampoline.free_size()<calls.size()*14) return false;
                for (std::size_t i=0;i<calls.size();++i) {
                    const auto address=REL::Offset(calls[i]).address();
                    ++patched;
                    (void)trampoline.write_call<5>(address,&variableCache);
                    std::array<std::uint8_t,5> call{};
                    std::array<std::uint8_t,14> relay{};
                    std::int32_t displacement{}; std::uintptr_t destination{};
                    bool valid=native_memory::guardedCopyFromMemory(reinterpret_cast<void*>(address),call.data(),call.size()) && call[0]==0xE8;
                    std::memcpy(&displacement,call.data()+1,4);
                    valid=valid && native_memory::guardedCopyFromMemory(reinterpret_cast<void*>(address+5+displacement),relay.data(),relay.size());
                    std::memcpy(&destination,relay.data()+6,8);
                    constexpr std::array<std::uint8_t,6> jump{0xFF,0x25,0,0,0,0};
                    if (!valid || std::memcmp(relay.data(),jump.data(),jump.size()) || destination!=reinterpret_cast<std::uintptr_t>(&variableCache))
                        throw std::runtime_error("native variable cache redirect failed");
                }
                if (!FlushInstructionCache(GetCurrentProcess(),nullptr,0)) throw std::runtime_error("native variable cache instruction flush failed");
                return true;
            } catch (...) {
                for (std::size_t i=0;i<patched;++i) REL::safe_write(REL::Offset(calls[i]).address(),originals[i].data(),originals[i].size());
                FlushInstructionCache(GetCurrentProcess(),nullptr,0);
                return false;
            }
        }
        // Diagnostic: per primary weapon identity, the idle roots the native
        // collector returns for the player's own (right-hand) actions and the
        // player's idle-root list, at the change and again once 3 s later
        // (subgraph registration is asynchronous), for comparison with
        // weapons whose roots do not resolve in the left graph.
        std::atomic<std::uint32_t> s_tracedPrimaryRoots{UINT32_MAX}; // Any native idle thread.
        std::atomic<std::uint64_t> s_settledPrimaryTrace{};
        bool idleRoots(void* database,RE::Actor* actor,RE::BSScrapArray<void*>* out)
        {
            if (!t_context && out && s_installed && actor==f4vr::getPlayer()) {
                const bool result=s_idleRoots(database,actor,out);
                RE::BGSObjectInstance right{nullptr,nullptr};
                native<RE::BGSObjectInstance*(*)(RE::Actor*,RE::BGSObjectInstance*,std::uint32_t)>(0xE50DA0)(actor,&right,0);
                const auto form=right.object?right.object->formID:0;
                const auto now=GetTickCount64();
                const bool changed=s_tracedPrimaryRoots.exchange(form,std::memory_order_relaxed)!=form;
                if (changed) s_settledPrimaryTrace.store(now+3000,std::memory_order_relaxed);
                auto settleAt=s_settledPrimaryTrace.load(std::memory_order_relaxed);
                const bool settled=!changed && settleAt && now>=settleAt &&
                    s_settledPrimaryTrace.compare_exchange_strong(settleAt,0,std::memory_order_relaxed);
                if (changed || settled) {
                    const char* stage=changed?"right-weapon-change":"right-weapon-settled";
                    for (std::uint32_t i=0;i<out->size() && i<16;++i)
                        ROCK_LOG_INFO(Animation,"AKIMBO native primary idle root stage={} weapon={:08X} index={} root={:#x}",
                            stage,form,i,reinterpret_cast<std::uintptr_t>((*out)[i]));
                    ROCK_LOG_INFO(Animation,"AKIMBO native primary idle roots stage={} weapon={:08X} count={} result={}",stage,form,out->size(),result);
                    native_equipped_action_graph::logPlayerIdleRoots(actor,form,stage);
                }
                return result;
            }
            if (!t_context || actor!=t_context->player || !out) return s_idleRoots(database,actor,out);
            if (!validContext()) return false;
            ++t_context->idleQueries;
            // Native 595C50 still adds the race/base root after this collector.
            // The roots come from this hand's own weapon subgraph request in
            // stored order; none enter the player's primary list.
            const bool traceRoots=!t_context->state->rootsTraced;
            t_context->state->rootsTraced=true;
            for (const auto& entry:t_context->state->graph->idleRoots()) {
                // Native EC5C80 copies each root name and rewrites it through
                // the global behavior alias table (2BB10) before the idle
                // database lookup. Wrapping behaviors (Tactical Reload's
                // Pistol_GunWrappingBehavior) are not idle-database roots; only
                // the aliased name is.
                RE::BSFixedString name{entry.name};
                native<bool(*)(RE::BSFixedString*)>(0x2BB10)(&name);
                auto* root=native<void*(*)(void*,const char*)>(0x595760)(database,name.c_str());
                if (traceRoots) ROCK_LOG_INFO(Animation,"AKIMBO idle root lookup form={:08X} name='{}' aliased='{}' resolved={:#x}",
                    t_context->state->item.object->formID,entry.name.c_str(),name.c_str(),reinterpret_cast<std::uintptr_t>(root));
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
                ROCK_LOG_SAMPLE_WARN(Weapon,1000,"AKIMBO second weapon shot rejected: independent native muzzle unavailable");
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
        // 1BA9BF0/1BA9D20 only queue a pulse per native controller index
        // (+0x50 records at 145BBCA08); another thread plays it later, so the
        // physical hand must be chosen here, not at the OpenVR call. Player
        // +0x8D0 holds the native offhand (left) index; native left-handed
        // mode is forced off.
        std::uint32_t pulseHand(std::uint32_t hand)
        {
            auto* player=s_installed?f4vr::getPlayer():nullptr;
            std::uint32_t left{};
            if (!player || !native_memory::tryReadField(player,0x8D0,left) || left>1) return hand;
            const std::uint32_t right=1-left;
            const bool secondaryLeft=s_handLeft.load(std::memory_order_relaxed);
            if (validContext() && t_context->shot) return secondaryLeft?left:right;
            // The second weapon holds the right hand, so the primary fires
            // from the left: its pulses, queued for the native primary
            // (right) index, belong to the left hand. Input's OpenVR retarget
            // stands down in this layout.
            if (!secondaryLeft && hand==right && s_drawn.load(std::memory_order_relaxed) &&
                input_remap_runtime::publishedWeaponTriggerRouting().leftFiring) return left;
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
            // Condition and idle-selection traces cover reloads and every
            // refused action (rate-limited below), so a rejected attack
            // reports the variable or idle that refused it.
            context.traceReload=true;
            Action action{player,form};
            const bool accepted=native<bool(*)(void*)>(0xF9BEF0)(action.data.data());
            ++s.actions;
            if (accepted || trace(s,Trace::Action)) ROCK_LOG_INFO(Weapon,"AKIMBO native action form={:08X} hand={} action={} event='{}' accepted={} actualIndex=1 managerReads={} idleQueries={} idleMatches={} cacheReads={} conditional={} attempt={}",
                s.item.object->formID,handName(),std::to_underlying(object),action.event().c_str(),accepted,context.managerReads,context.idleQueries,context.idleMatches,context.cacheReads,conditional,s.actions);
            if ((object==RE::DEFAULT_OBJECT::kActionReload || !accepted) && (accepted || trace(s,Trace::Refusal,2))) {
                for (std::size_t i=0;i<context.queryCount;++i) {
                    const auto& query=context.queries[i];
                    ROCK_LOG_INFO(Animation,"AKIMBO action condition form={:08X} action={} attempt={} name='{}' id={} readable={} source={} value={}",
                        s.item.object->formID,std::to_underlying(object),s.actions,query.name.c_str(),query.id,query.accepted,
                        query.actorState?"actor":"weapon-graph",query.value);
                }
                for (std::size_t i=0;i<context.idleCount;++i) {
                    const auto& query=context.idles[i];
                    ROCK_LOG_INFO(Animation,"AKIMBO action selection form={:08X} action={} attempt={} candidate={:08X} selected={:08X} variableQueryRange=[{},{})",
                        s.item.object->formID,std::to_underlying(object),s.actions,query.candidate,query.selected,query.queryBegin,query.queryEnd);
                }
            }
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
                // Graph lifecycle state follows every drained event, including
                // teardown drains; otherwise a discarded reloadEnd leaves this
                // hand permanently "reloading" and refusing fire and reload.
                if (event.tag==RE::BSFixedString("reloadStart")) {
                    // Native graphs can enter reload themselves after the last
                    // shot. Their lifecycle, not just our button dispatch,
                    // owns whether further attacks are admitted.
                    s.reloading=true;
                    (void)s.trigger.cancel();
                    continue;
                }
                // Every reload leaves the graph's reload state with
                // reloadStateExit; reloadEnd is not universal (Tactical Reload's
                // empty-magazine reloads exit without it, which left this hand
                // refusing every later attack).
                if (event.tag==RE::BSFixedString("reloadEnd") || event.tag==RE::BSFixedString("reloadStateExit")) {
                    if (s.reloading) ROCK_LOG_INFO(Weapon,"AKIMBO native reload ended form={:08X} tag='{}' loaded={} eventSequence={}",
                        s.item.object->formID,event.tag.c_str(),loaded(native_hand_equipment::equippedViews()[1]),s.events);
                    s.reloading=false;
                    continue;
                }
                if (event.tag==RE::BSFixedString("attackStop")) { attackState(s,0); continue; }
                if (!effects) continue;
                if (s.audio.event(f4vr::getPlayer(),s.model.get(),event.tag,event.payload,s.frame)) continue;
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
                        ROCK_LOG_SAMPLE_WARN(Weapon,1000,"AKIMBO second weapon shot rejected: native model has no independent muzzle form={:08X}",s.item.object->formID);
                        continue;
                    }
                    const RE::BSFixedString index{"1"};
                    native<bool(*)(void*,RE::Actor*,const RE::BSFixedString*)>(0xFF2A40)(nullptr,context.player,&index);
                    const auto after=native_hand_equipment::equippedViews();
                    ++s.shots;
                    if (trace(s,Trace::Fire) || loaded(after[1])==0 || loaded(after[0])!=rightBefore || !context.prepared) ROCK_LOG_INFO(Weapon,
                        "AKIMBO native fire event form={:08X} hand={} prepared={} loaded={}->{} primaryLoaded={}->{} index=1 sequence={}",
                        s.item.object->formID,handName(),context.prepared,before,loaded(after[1]),rightBefore,loaded(after[0]),s.shots);
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
                }
            }
        }
        void cancel(State& s)
        {
            if (s.trigger.cancel() && s.ready && s.graph) { (void)s.graph->notify(RE::BSFixedString("attackRelease")); attackState(s,0); }
            s.reload=false;
            s.audio.stop();
            if (s.graph) drain(s,false);
        }
    }

    namespace
    {
        // Diagnostic for the restored-save report: the primary (index 0)
        // reloads without animation until it is re-equipped. One window per
        // dispatched primary reload records which of three stages is silent:
        // the native idle choice, the player graph's events, and the primary
        // model's own graph events and node motion.
        constexpr std::uint64_t kPrimaryReloadTraceMs=3000;
        constexpr std::uint32_t kPrimaryReloadTraceLines=48;
        std::atomic<std::uint64_t> s_primaryReloadTraceUntil{0}; // Input writer, frame/idle readers.
        std::atomic<std::uint32_t> s_primaryReloadTraceLines{0};
        bool tracingPrimaryReload() noexcept
        {
            return GetTickCount64()<s_primaryReloadTraceUntil.load(std::memory_order_acquire);
        }
        bool primaryReloadTraceLine() noexcept
        {
            return s_primaryReloadTraceLines.fetch_add(1,std::memory_order_relaxed)<kPrimaryReloadTraceLines;
        }
        using EventSource=RE::BSTEventSource<RE::BSAnimationGraphEvent>;
        EventSource* graphEvents(RE::BShkbAnimationGraph* graph) noexcept
        {
            // Same verified layout as NativeEquippedActionGraph (graph+0x68).
            return reinterpret_cast<EventSource*>(reinterpret_cast<std::byte*>(graph)+0x68);
        }
        struct PrimaryReloadTrace final : RE::BSTEventSink<RE::BSAnimationGraphEvent>
        {
            const char* label{};
            // The window's ManagerRef keeps the manager alive. Graphs are
            // compared by address only and unregistered only while the
            // manager still holds them.
            RE::BSAnimationGraphManager* manager{};
            std::vector<RE::BShkbAnimationGraph*> graphs;
            std::uint32_t events{};
            void attach(RE::BSAnimationGraphManager* owner,const char* name)
            {
                label=name; manager=owner;
                if (!manager) return;
                for (const auto& graph:manager->graph) if (graph) {
                    graphs.push_back(graph.get());
                    graphEvents(graph.get())->RegisterSink(this);
                }
            }
            void detach()
            {
                if (manager) for (const auto& graph:manager->graph)
                    if (graph && std::find(graphs.begin(),graphs.end(),graph.get())!=graphs.end())
                        graphEvents(graph.get())->UnregisterSink(this);
                graphs.clear(); manager=nullptr;
            }
            RE::BSEventNotifyControl ProcessEvent(const RE::BSAnimationGraphEvent& event,EventSource*) override
            {
                ++events;
                if (primaryReloadTraceLine())
                    ROCK_LOG_INFO(Animation,"AKIMBO primary reload trace graph={} event='{}' payload='{}'",
                        label,event.tag.c_str()?event.tag.c_str():"",event.payload.c_str()?event.payload.c_str():"");
                return RE::BSEventNotifyControl::kContinue;
            }
        };
        struct PrimaryReloadWindow
        {
            bool active{};
            std::uint32_t form{};
            ManagerRef playerManager,modelManager;
            PrimaryReloadTrace player,model;
            std::vector<RE::NiPointer<RE::NiAVObject>> nodes;
            std::vector<RE::NiPoint3> initial;
            float maxMotion{};
            std::size_t maxNode{};
        };
        PrimaryReloadWindow s_primaryReload{}; // Frame thread only.

        // Which models a player graph's bones are bound to. FO4VR 1.2.72 raw
        // disassembly of 1416A9570 (graph root setter): the bound root is a
        // refcounted node at graph+0x388; the bone table is a BSTArray at
        // graph+0x2A0 (data +0, size +0x10, 16-byte entries, node at +0),
        // filled by name lookup under that root (141785010 -> 141C20840,
        // sized by 1417901D0). Names shared by two weapon models can bind
        // one weapon's bones to the other's nodes.
        void reportGraphBinding(std::size_t index, RE::BShkbAnimationGraph* graph, const RE::NiAVObject* primary, const RE::NiAVObject* secondary)
        {
            RE::NiAVObject* root{};
            const RE::NiAVObject* bone{};
            std::uintptr_t bones{};
            std::uint32_t count{};
            if (!native_memory::tryReadField(graph, 0x388, root) || !native_memory::tryReadField(graph, 0x2A0, bones) ||
                !native_memory::tryReadField(graph, 0x2B0, count)) {
                ROCK_LOG_INFO(Animation, "AKIMBO primary reload trace binding graph={} unreadable", index);
                return;
            }
            std::uint32_t inPrimary{}, inSecondary{}, other{}, missing{};
            const char* secondaryBone = "";
            const char* otherBone = "";
            for (std::uint32_t i = 0; bones && i < count && i < 512; ++i) {
                if (!native_memory::tryReadValue(reinterpret_cast<const RE::NiAVObject* const*>(bones + i * 0x10), bone) || !bone) { ++missing; continue; }
                int owner = 0;
                for (auto* node = bone; node && owner == 0; node = node->parent) {
                    if (node == primary) owner = 1;
                    else if (node == secondary) owner = 2;
                }
                if (owner == 1) ++inPrimary;
                else if (owner == 2) { if (!inSecondary++) secondaryBone = bone->name.c_str() ? bone->name.c_str() : ""; }
                else if (!other++) otherBone = bone->name.c_str() ? bone->name.c_str() : "";
            }
            ROCK_LOG_INFO(Animation,
                "AKIMBO primary reload trace binding graph={} root={:#x} rootName='{}' bones={} inPrimaryModel={} inSecondaryModel={} elsewhere={} missing={} firstSecondaryBone='{}' firstElsewhereBone='{}'",
                index, reinterpret_cast<std::uintptr_t>(root), root && root->name.c_str() ? root->name.c_str() : "", count,
                inPrimary, inSecondary, other, missing, secondaryBone, otherBone);
        }

        void endPrimaryReloadTrace()
        {
            auto& w=s_primaryReload;
            if (!w.active) return;
            w.player.detach(); w.model.detach();
            const char* node=w.maxNode<w.nodes.size() && w.nodes[w.maxNode] && w.nodes[w.maxNode]->name.c_str()?
                w.nodes[w.maxNode]->name.c_str():"<none>";
            ROCK_LOG_INFO(Animation,"AKIMBO primary reload trace end weapon={:08X} playerEvents={} modelEvents={} sampledNodes={} maxNodeMotion={:.4f} node='{}'",
                w.form,w.player.events,w.model.events,w.nodes.size(),w.maxMotion,node);
            w=PrimaryReloadWindow{};
        }

        void beginPrimaryReloadTrace()
        {
            auto& w=s_primaryReload;
            auto* player=f4vr::getPlayer();
            const auto view=native_hand_equipment::equippedViews()[0];
            if (!player || !view.weapon) return;
            w.active=true;
            w.form=view.weapon->formID;
            (void)s_getManager(static_cast<RE::IAnimationGraphManagerHolder*>(player),&w.playerManager);
            w.player.attach(w.playerManager.get(),"player");
            // The primary model's own graph, read from its biped record the
            // same way the secondary model graph is isolated.
            const auto slot=native<std::uint32_t(*)(RE::Actor*,RE::TESBoundObject*,std::uint32_t)>(0x1CAFC0)(player,view.weapon,0);
            const auto* biped=native_hand_equipment::playerBiped(true);
            void* holder{};
            std::uintptr_t vtable{};
            if (biped && slot<44 && native_memory::tryReadField(static_cast<const std::byte*>(biped)+slot*0x58,0x50,holder) && holder &&
                native_memory::tryReadField(holder,0,vtable) && vtable==REL::Offset(0x2D3A8D8).address())
                native<bool(*)(void*,ManagerRef*)>(0x812EA0)(holder,&w.modelManager);
            w.model.attach(w.modelManager.get(),"primary-model");
            auto& s=state();
            const bool sharedWithSecondary=w.modelManager &&
                (w.modelManager==s.modelManagers[0] || w.modelManager==s.modelManagers[1]);
            // Up to 32 named descendant nodes of the primary first-person model.
            if (view.root) {
                std::vector<RE::NiAVObject*> pending{view.root};
                for (std::size_t i=0;i<pending.size() && pending.size()<256 && w.nodes.size()<32;++i) {
                    auto* node=pending[i]->IsNode();
                    if (!node) continue;
                    for (const auto& child:node->GetRuntimeData().children) {
                        if (!child) continue;
                        pending.push_back(child.get());
                        if (child->IsNode() && w.nodes.size()<32) {
                            w.nodes.emplace_back(child.get());
                            w.initial.push_back(child->local.translate);
                        }
                    }
                }
            }
            const auto secondaryView=native_hand_equipment::equippedViews()[1];
            for (std::size_t i=0;i<w.player.graphs.size();++i)
                reportGraphBinding(i,w.player.graphs[i],view.root,secondaryView.root);
            ROCK_LOG_INFO(Animation,"AKIMBO primary reload trace begin weapon={:08X} playerManager={:#x} playerGraphs={} modelHolder={:#x} modelManager={:#x} modelGraphs={} modelSharedWithSecondary={} sampledNodes={}",
                w.form,reinterpret_cast<std::uintptr_t>(w.playerManager.get()),w.player.graphs.size(),reinterpret_cast<std::uintptr_t>(holder),
                reinterpret_cast<std::uintptr_t>(w.modelManager.get()),w.model.graphs.size(),sharedWithSecondary,w.nodes.size());
        }

        void tracePrimaryReloadFrame()
        {
            auto& w=s_primaryReload;
            if (!tracingPrimaryReload()) { endPrimaryReloadTrace(); return; }
            if (!w.active) beginPrimaryReloadTrace();
            for (std::size_t i=0;i<w.nodes.size();++i) {
                if (!w.nodes[i]) continue;
                const auto d=w.nodes[i]->local.translate-w.initial[i];
                const float motion=std::sqrt(d.x*d.x+d.y*d.y+d.z*d.z);
                if (motion>w.maxMotion) { w.maxMotion=motion; w.maxNode=i; }
            }
        }
    }

    void tracePrimaryReload() noexcept
    {
        s_primaryReloadTraceLines.store(0,std::memory_order_relaxed);
        s_primaryReloadTraceUntil.store(GetTickCount64()+kPrimaryReloadTraceMs,std::memory_order_release);
    }

    bool readAmmo(const std::uint32_t index, equipped_ammo::Facts& out) noexcept
    {
        out = {};
        auto* player = s_installed && index < 2 ? f4vr::getPlayer() : nullptr;
        if (!player || !player->currentProcess) return false;
        // The producer's helpers: indexed item (E50DA0), indexed ammo (E05BA0),
        // indexed loaded rounds (EC48C0, which its DDF690 forwards to),
        // inventory count (3E3750) and the special ammo forms (2F30E0).
        const auto itemAt = native<RE::BGSObjectInstance*(*)(RE::Actor*,RE::BGSObjectInstance*,std::uint32_t)>(0xE50DA0);
        const auto ammoAt = native<RE::TESAmmo*(*)(RE::Actor*,std::uint32_t)>(0xE05BA0);
        const auto loadedAt = native<std::uint32_t(*)(void*,std::uint32_t)>(0xEC48C0);
        const auto inventoryCount = native<int(*)(RE::TESObjectREFR*,RE::TESBoundObject*)>(0x3E3750);
        const auto countsAsLoaded = native<bool(*)(const RE::TESAmmo*)>(0x2F30E0);
        RE::BGSObjectInstance item{ nullptr, nullptr };
        itemAt(player, &item, index);
        if (!item.object || !item.object->IsWeapon()) return true;
        // Type on the base form; flags and capacity on the instance.
        const auto* weapon = static_cast<const RE::TESObjectWEAP*>(item.object);
        if (static_cast<std::uint8_t>(weapon->weaponData.type.underlying()) < 7) return true;
        const auto* data = item.instanceData ?
            static_cast<const RE::TESObjectWEAP::InstanceData*>(item.instanceData.get()) : &weapon->weaponData;
        out.weapon = true;
        out.notPlayable = data->flags.all(RE::WEAPON_FLAGS::kNotPlayable);
        auto* ammo = ammoAt(player, index);
        if (!ammo) return true;
        out.hasAmmo = true;
        out.loaded = loadedAt(player->currentProcess, index);
        out.charge = ammo->data.health;
        out.countAsLoaded = countsAsLoaded(ammo);
        out.nonPlayableAmmo = (ammo->data.flags & 0x2) != 0;
        out.capacity = data->ammoCapacity;
        if (!out.countAsLoaded && !out.nonPlayableAmmo) out.inventoryCount = inventoryCount(player, ammo);
        if (out.charge == 0 && ammoAt(player, 1 - index) == ammo) out.otherLoadedSameAmmo = loadedAt(player->currentProcess, 1 - index);
        return true;
    }

    bool install() noexcept
    {
        if (s_installed) return true;
        if (!Graph::validate() || !native_equipped_animation_audio::Channel::validate()) {
            ROCK_LOG_ERROR(Init,"AKIMBO native graph/audio dependency validation failed");
            return false;
        }
        std::uintptr_t getter{};
        const auto slot=REL::Offset(0x2D819E8).address();
        if (!native_memory::tryReadValue(reinterpret_cast<const std::uintptr_t*>(slot),getter) || getter!=REL::Offset(0xE14FD0).address() ||
            !matches(0xE50DA0,std::to_array<std::uint8_t>({0x40,0x53,0x48,0x83,0xEC,0x20,0x48,0x8B,0xDA})) ||
            !matches(0xFF2A40,std::to_array<std::uint8_t>({0x48,0x89,0x5C,0x24,0x08})) ||
            !matches(0xFF2C10,std::to_array<std::uint8_t>({0x48,0x89,0x5C,0x24,0x08})) ||
            !matches(0x595760,std::to_array<std::uint8_t>({0x40,0x56,0x48,0x83,0xEC,0x20,0x48,0x8B,0xF2})) ||
            !matches(0x2BB10,std::to_array<std::uint8_t>({0x48,0x89,0x5C,0x24,0x08,0x48,0x89,0x74,0x24,0x18,0x57,0x48,0x83,0xEC,0x20,0x48,0x8B,0xF9})) ||
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
            !matches(0x2F30E0,std::to_array<std::uint8_t>({0x48,0x85,0xC9,0x74,0x15,0x48,0x3B,0x0D})) ||
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
        constexpr auto idleTracePrefix=std::to_array<std::uint8_t>({0x48,0x89,0x5C,0x24,0x08,0x48,0x89,0x6C,0x24,0x10,0x48,0x89,0x74,0x24,0x18});
        if (!entry_trampoline_hook::install("Equipped reload native selection trace",0x593D90,idleTracePrefix.data(),idleTracePrefix.size(),reinterpret_cast<void*>(&selectIdle),original)) return false;
        s_selectIdle=reinterpret_cast<SelectIdle>(original);
        if (!matches(0xEF96B0,std::to_array<std::uint8_t>({0x40,0x53,0x48,0x83,0xEC,0x20})) ||
            !matches(0xEFD0B0,std::to_array<std::uint8_t>({0x40,0x53,0x48,0x83,0xEC,0x50})) ||
            !matches(0xEFD1C0,std::to_array<std::uint8_t>({0x48,0x89,0x5C,0x24,0x08,0x57,0x48,0x83,0xEC,0x20})) ||
            !matches(0xE526A0,std::to_array<std::uint8_t>({0x48,0x83,0xEC,0x28,0x48,0x8B,0x89,0,3,0,0})) ||
            !installRecoilCalls()) {
            ROCK_LOG_ERROR(Init,"AKIMBO native recoil routing validation failed");
            return false;
        }
        if (!installVariableCacheCalls() || !installVariableQueryTrace()) {
            ROCK_LOG_ERROR(Init,"AKIMBO native variable cache routing validation failed");
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
            tracePrimaryReloadFrame();
            s.frame=frame.frameIndex;
            const auto view=native_hand_equipment::equippedViews()[1];
            const bool handLeft=native_hand_equipment::secondaryHandIsLeft();
            s_handLeft.store(handLeft,std::memory_order_relaxed);
            s_drawn.store(view.weapon && frame.weaponDrawn,std::memory_order_relaxed);
            input_remap_runtime::setSlotOneTrigger(view.weapon != nullptr, view.weapon && frame.weaponDrawn, handLeft);
            if (s.graph && !same(s,view)) { cancel(s); s.ready=false; s.retiring=true; }
            if (s.retiring) {
                if (!s.graph->canDestroy()) return;
                releaseModelGraphs(s,false);
                s.bindings.restore();
                s.graph.reset(); s.model.reset(); s.item=RE::BGSObjectInstance{nullptr,nullptr};
                s.recoilNode.reset(); s.kick={}; s.kickActive=false; s.recoilFrame=UINT64_MAX;
                s.retiring=false; s.failed=false; s.reloading=false; s.rootsTraced=false;
            }
            if (!attachment || !view.weapon || !view.root || frame.localMenuBlocking || !frame.weaponDrawn || !frame.visualAuthorityAvailable) { cancel(s); return; }
            const auto trigger=input_remap_runtime::peekRawButtonState(handLeft,input_remap_policy::kOpenVrSteamVrTriggerButtonId);
            const bool providerSuppressed=input_remap_runtime::isProviderOpenVrGameInputSuppressedForHand(handLeft);
            const bool menu=input_remap_runtime::isMenuInputActive();
            // A sample older than one normal input period (a frame hitch)
            // neither starts nor ends a press: the held state is kept. Only a
            // sample too old to describe the controller releases the trigger.
            constexpr std::uint64_t kFreshInputMs=100,kLostInputMs=500;
            const bool fresh=trigger.sampleAgeMilliseconds<=kFreshInputMs;
            const bool lost=trigger.sampleAgeMilliseconds>kLostInputMs;
            const bool allowed=trigger.available && !lost && !providerSuppressed && !menu;
            if ((fresh || !allowed) && s.trigger.observe(trigger.held,allowed) && s.ready && s.graph) {
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
                (static_cast<std::uint32_t>(s.reloading)<<6) | (static_cast<std::uint32_t>(lost)<<7);
            if (inputMask!=s.inputMask) {
                s.inputMask=inputMask;
                ++s.inputTraces;
                if (trace(s,Trace::Input)) ROCK_LOG_INFO(Weapon,
                    "AKIMBO secondary input form={:08X} available={} ageMs={} lost={} providerSuppressed={} menu={} held={} armed={} reloading={} allowed={}",
                    view.weapon->formID,trigger.available,trigger.sampleAgeMilliseconds,lost,providerSuppressed,menu,trigger.held,s.trigger.armed(),s.reloading,allowed);
            }
            // Input admission gates only NEW actions. The graph, its part
            // presentation and its events keep running every drawn frame, so
            // a suppressed or stale trigger never freezes a reload, drops its
            // reloadComplete/reloadEnd, or cuts its sounds.
            if (s.reload) {
                s.reload=false;
                if (!menu && !s.reloading && dispatch(s,RE::DEFAULT_OBJECT::kActionReload)) { s.reloading=true; (void)s.trigger.cancel(); }
            }
            if (allowed && fresh && s.trigger.wantsStart() && !s.reloading) {
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
            // A frame without part transport still owns this graph's gameplay
            // events (shots, magazine refill, reload end).
            if (!s.bindings.present(s.frame))
                ROCK_LOG_SAMPLE_WARN(Animation,1000,"AKIMBO secondary part transport unavailable form={:08X}",view.weapon->formID);
            drain(s,true);
        } catch (...) {
            s.audio.stop();
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
        if (s.graph->place(attachment->world) && !s.bindings.present(runtime_state::currentFrame().frameIndex))
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
                ROCK_LOG_INFO(Weapon,"AKIMBO recoil delivered form={:08X} physicalHand={} profile={} family={} nativeT=({:.4f},{:.4f},{:.4f}) primaryRecord=restored",
                    s.item.object->formID,handName(),name(profile),name(s.recoilFamily),s.recoilNode->local.translate.x,s.recoilNode->local.translate.y,s.recoilNode->local.translate.z);
        } catch (...) {
            ROCK_LOG_SAMPLE_WARN(Weapon,1000,"AKIMBO secondary recoil delivery failed");
        }
    }
}
