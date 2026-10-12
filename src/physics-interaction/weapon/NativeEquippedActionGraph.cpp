#include "physics-interaction/weapon/NativeEquippedActionGraph.h"
#include "physics-interaction/weapon/NativeSubgraphMetadata.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/native/NativeNiNodeFactory.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/weapon/HoldingGripRecords.h"
#include "physics-interaction/weapon/NativeSecondaryPose.h"
#include "physics-interaction/PhysicsLog.h"
#include "rock_support/Fo4VrRuntime.h"
#include "RE/Bethesda/Actor.h"
#include "RE/Bethesda/PlayerCharacter.h"
#include "RE/Bethesda/BSStringT.h"
#include "RE/Bethesda/MemoryManager.h"
#include <Windows.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

namespace rock::native_equipped_action_graph
{
    namespace
    {
        template<class F> F native(std::uintptr_t rva) { return reinterpret_cast<F>(REL::Offset(rva).address()); }
        template<std::size_t N> bool matches(std::uintptr_t rva, const std::array<std::uint8_t,N>& bytes)
        {
            std::array<std::uint8_t,N> actual{};
            return native_memory::guardedCopyFromMemory(reinterpret_cast<void*>(REL::Offset(rva).address()),actual.data(),N) && actual==bytes;
        }
        bool s_validated = false;
        using Handles = RE::BSTSmallArray<RE::SubgraphHandle,2>;
        using Identifiers = RE::BSTSmallArray<RE::SubgraphIdentifier,2>;
        using Keywords = RE::BSScrapArray<RE::IKeywordFormBase*>;
        using ManagerRef = RE::BSTSmartPointer<RE::BSAnimationGraphManager>;
        using GraphRef = RE::BSTSmartPointer<RE::BShkbAnimationGraph>;
        using Source = RE::BSTEventSource<RE::BSAnimationGraphEvent>;
        constexpr std::size_t kMaxBones = 768;

        using Metadata=native_subgraph_metadata::List;

        struct alignas(16) Qs { float translation[4]; float rotation[4]; float scale[4]; };
        bool transform(const Qs& input, RE::NiTransform& out)
        {
            float norm=0;
            for (auto f:input.rotation) { if (!std::isfinite(f)) return false; norm+=f*f; }
            for (auto f:input.translation) if (!std::isfinite(f)) return false;
            for (auto f:input.scale) if (!std::isfinite(f)) return false;
            if (norm<1e-6f || std::abs(input.scale[0])<1e-6f ||
                std::abs(input.scale[0]-input.scale[1])>.001f || std::abs(input.scale[0]-input.scale[2])>.001f) return false;
            out.translate={input.translation[0],input.translation[1],input.translation[2]};
            out.rotate=transform_math::transposeRotation(transform_math::havokQuaternionToNiRows<RE::NiMatrix3>(input.rotation));
            out.scale=input.scale[0];
            return holding_grip_math::valid(holding_grip_math::fromNative(out));
        }
    }

    struct Graph::Impl final : RE::BSTEventSink<RE::BSAnimationGraphEvent>
    {
        alignas(16) std::array<std::byte,0x18> storage{};
        RE::BGSObjectInstance item{nullptr,nullptr};
        RE::Actor* player{}; // Persistent player; request/callback owner validates identity.
        RE::TESRace* race{};
        Handles handles;
        Identifiers identifiers;
        Handles baseHandles;
        Identifiers baseIdentifiers;
        std::vector<RE::BSFixedString> roots;
        std::array<RE::NiPointer<RE::NiNode>,kMaxBones> bones{};
        RE::NiPointer<RE::NiNode> root;
        RE::NiNode* weapon{}; // Owned by bones/root.
        std::size_t boneCount{};
        Events events{};
        std::size_t eventCount{};
        DWORD thread{GetCurrentThreadId()};
        std::uint8_t phase{};
        bool constructed{}, sinks{}, overflow{}, updateRejected{};
        const char* stage{"not-started"};

        RE::SimpleAnimationGraphManagerHolder* holder() const { return reinterpret_cast<RE::SimpleAnimationGraphManagerHolder*>(const_cast<std::byte*>(storage.data())); }
        Source* source(RE::BShkbAnimationGraph* graph) { return reinterpret_cast<Source*>(reinterpret_cast<std::byte*>(graph)+0x68); }
        RE::BSEventNotifyControl ProcessEvent(const RE::BSAnimationGraphEvent& event, Source*) override
        {
            if (GetCurrentThreadId()!=thread || eventCount==events.size()) { overflow=true; return RE::BSEventNotifyControl::kContinue; }
            events[eventCount++]={event.tag,event.payload};
            return RE::BSEventNotifyControl::kContinue;
        }
        ~Impl() override
        {
            if (!constructed) return;
            auto* h=holder();
            if (h->animationGraphManager) {
                if (sinks) for (const auto& graph:h->animationGraphManager->graph) if (graph) source(graph.get())->UnregisterSink(this);
                if (!handles.empty()) native<void(*)(ManagerRef*,Handles*)>(0x7F43C0)(&h->animationGraphManager,&handles);
                if (!baseHandles.empty()) native<void(*)(ManagerRef*,Handles*)>(0x7F43C0)(&h->animationGraphManager,&baseHandles);
            }
            // Caller checks readiness before destruction; native 812450 may
            // otherwise synchronously wait for its background loader.
            native<void(*)(void*)>(0x811F50)(storage.data());
        }
        bool buildBones(RE::BShkbAnimationGraph* graph)
        {
            stage="skeleton-array-layout";
            void* owner{}; void* skeleton{}; const std::int16_t* parents{};
            const Qs* reference{}; const std::uintptr_t* names{};
            std::uint32_t count{}, parentCount{}, poseCount{};
            if (!graph || !native_memory::tryReadField(graph,0x240,owner) || !owner ||
                !native_memory::tryReadField(owner,0x20,skeleton) || !skeleton ||
                !native_memory::tryReadField(skeleton,0x18,parents) || !parents ||
                !native_memory::tryReadField(skeleton,0x20,parentCount) ||
                !native_memory::tryReadField(skeleton,0x28,names) || !names ||
                !native_memory::tryReadField(skeleton,0x30,count) || count==0 || count>kMaxBones || parentCount!=count ||
                !native_memory::tryReadField(skeleton,0x38,reference) || !reference ||
                !native_memory::tryReadField(skeleton,0x40,poseCount) || poseCount!=count) return false;
            std::array<std::int16_t,kMaxBones> ancestry{};
            if (!native_memory::guardedCopyFromMemory(parents,ancestry.data(),count*sizeof(std::int16_t))) return false;
            stage="skeleton-root-allocation";
            root=native_scene::createEngineNiNode(4);
            if (!root) return false;
            root->name=RE::BSFixedString("ROCK Secondary Native Behavior");
            for (std::uint32_t i=0;i<count;++i) {
                stage="skeleton-bone-data";
                // hkaBone is {hkStringPtr,lockTranslation,padding}, stride 10.
                std::uintptr_t name{}; Qs pose{};
                if (!native_memory::tryReadField(names,i*0x10,name) || !(name&~std::uintptr_t{1}) ||
                    !native_memory::tryReadValue(reference+i,pose)) return false;
                std::array<char,260> text{};
                const auto* p=reinterpret_cast<const char*>(name&~std::uintptr_t{1});
                bool terminated=false;
                for (std::size_t j=0;j<text.size();++j) {
                    if (!native_memory::tryReadValue(p+j,text[j])) return false;
                    if (!text[j]) { terminated=true; break; }
                }
                if (!terminated || !text[0]) return false;
                stage="skeleton-bone-allocation";
                auto node=native_scene::createEngineNiNode(4);
                if (!node) return false;
                stage="skeleton-reference-transform";
                if (!transform(pose,node->local)) {
                    ROCK_LOG_ERROR(Animation,"AKIMBO invalid native reference pose bone={} name='{}'",i,text.data());
                    return false;
                }
                node->name=RE::BSFixedString(text.data());
                if (node->name==RE::BSFixedString("Weapon")) weapon=node.get();
                bones[i]=std::move(node);
                auto parent=ancestry[i];
                stage="skeleton-parent-topology";
                for (std::uint32_t depth=0;parent>=0;++depth) {
                    if (static_cast<std::uint32_t>(parent)>=count || static_cast<std::uint32_t>(parent)==i || depth>=count) return false;
                    parent=ancestry[parent];
                }
            }
            stage="skeleton-native-child-attachment";
            for (std::uint32_t i=0;i<count;++i) {
                const auto parent=ancestry[i];
                auto* parentNode=parent<0?root.get():bones[parent].get();
                parentNode->AttachChild(bones[i].get(),true);
                if (bones[i]->parent!=parentNode) return false;
            }
            boneCount=count;
            f4vr::updateTransformsDown(root.get(),true);
            stage="skeleton-weapon-anchor";
            ROCK_LOG_INFO(Animation,"AKIMBO private skeleton constructed form={:08X} bones={} root={:#x} weapon={:#x} heap=engine-live",
                item.object->formID,boneCount,reinterpret_cast<std::uintptr_t>(root.get()),reinterpret_cast<std::uintptr_t>(weapon));
            return weapon!=nullptr;
        }
    };

    Graph::Graph() : _impl(std::make_unique<Impl>()) {}
    Graph::~Graph()=default;

    bool Graph::validate() noexcept
    {
        if (s_validated) return true;
        if (!REL::Module::IsVR() || REL::Module::get().version()!=F4SE::RUNTIME_VR_1_2_72) return false;
        // Constructors/destructors and load/subgraph entries are shared with the
        // verified preharvest path. Bind/update/activation use the native manager,
        // not hka clip sampling or an independent simulation clock.
        s_validated=matches(0x811F10,std::to_array<std::uint8_t>({0x40,0x53,0x48,0x83,0xEC,0x20})) &&
            matches(0x811F50,std::to_array<std::uint8_t>({0x48,0x89,0x5C,0x24,0x10})) &&
            matches(0x811FE0,std::to_array<std::uint8_t>({0x48,0x89,0x5C,0x24,0x18})) &&
            matches(0x8122C0,std::to_array<std::uint8_t>({0x48,0x8B,0x41,0x10,0x48,0x85,0xC0,0x74,0x0C})) &&
            matches(0x1016340,std::to_array<std::uint8_t>({0x48,0x89,0x5C,0x24,0x08,0x48,0x89,0x74,0x24,0x10})) &&
            matches(0x7F4320,std::to_array<std::uint8_t>({0x48,0x89,0x5C,0x24,0x08})) &&
            matches(0x7F43C0,std::to_array<std::uint8_t>({0x48,0x83,0xEC,0x28,0x83,0x7A,0x18})) &&
            matches(0xEDA2F0,std::to_array<std::uint8_t>({0x48,0x89,0x5C,0x24,0x08,0x57,0x48,0x83,0xEC,0x30})) &&
            matches(0x168F940,std::to_array<std::uint8_t>({0x40,0x53,0x41,0x57,0x48,0x83,0xEC,0x38})) &&
            matches(0x1690240,std::to_array<std::uint8_t>({0x48,0x8B,0xC4,0x53,0x57,0x41,0x55,0x41,0x57})) &&
            matches(0x169036E,std::to_array<std::uint8_t>({0x48,0x8D,0x8F,0x88,0,0,0,0xE8,0x96,0x1A,0x11,0})) &&
            matches(0x1C23777,std::to_array<std::uint8_t>({0x8B,0x43,0x70,0xF3,0x0F,0x10,0x53,0x7C})) &&
            matches(0x1C2381F,std::to_array<std::uint8_t>({0x8B,0x83,0xA0,0,0,0,0x89,0x83,0xF0,0,0,0})) &&
            matches(0x80D6C0,std::to_array<std::uint8_t>({0x48,0x89,0x5C,0x24,0x08,0x57,0x48,0x83,0xEC,0x40})) &&
            matches(0x80DC20,std::to_array<std::uint8_t>({0x44,0x89,0x44,0x24,0x18,0x48,0x83,0xEC,0x28})) &&
            matches(0x80DC40,std::to_array<std::uint8_t>({0xF3,0x0F,0x11,0x54,0x24,0x18,0x48,0x83,0xEC,0x28})) &&
            matches(0xF2ED40,std::to_array<std::uint8_t>({0x48,0x89,0x5C,0x24,0x10,0x57,0x48,0x81,0xEC,0x60,0x02,0,0}));
        return s_validated;
    }

    bool Graph::start(RE::Actor* player,const RE::BGSObjectInstance& item)
    {
        auto& s=*_impl;
        s.stage="native-holder-construction";
        if (!s_validated || s.constructed || !player || player!=f4vr::getPlayer() || !item.object || !item.object->IsWeapon()) return false;
        s.player=player; s.race=player->GetVisualsRace(); s.item=item;
        if (!s.race) return false;
        // Native routines consume inline capacity bits; reserve the two real
        // project results using the same storage correction as preharvest.
        s.handles.reserve(2); s.identifiers.reserve(2);
        s.baseHandles.reserve(2); s.baseIdentifiers.reserve(2);
        const auto inlineStorage=[](auto& array) {
            return static_cast<void*>(array.data())==reinterpret_cast<std::byte*>(&array)+8 && array.capacity()==2;
        };
        if (!inlineStorage(s.handles) || !inlineStorage(s.identifiers) || !inlineStorage(s.baseHandles) || !inlineStorage(s.baseIdentifiers)) return false;
        if (native<void*(*)(void*)>(0x811F10)(s.storage.data())!=s.storage.data()) return false;
        s.constructed=true;
        RE::BSScrapArray<RE::BSStaticStringT<260>> projects;
        s.stage="player-behavior-projects";
        auto* root=player->Get3D();
        if (!root || !player->PopulateGraphProjectsToLoad(root,projects) || projects.size()<2 || projects.size()>8) return false;
        s.phase=1;
        s.stage="background-project-loading";
        return native<bool(*)(void*,decltype(projects)*,int)>(0x811FE0)(s.storage.data(),&projects,3);
    }

    Graph::Load Graph::advance()
    {
        auto& s=*_impl;
        if (!s.constructed || s.player!=f4vr::getPlayer() || s.race!=s.player->GetVisualsRace()) return Load::Failed;
        if (!canDestroy()) return Load::Pending;
        auto* manager=s.holder()->animationGraphManager.get();
        if (!manager || manager->graph.size()<2 || manager->graph.size()>8) return Load::Failed;
        if (s.phase==1) {
            s.stage="private-base-subgraph-request";
            // Full native evaluation needs role zero as well as the weapon
            // role. The old sampler needed only role one because it sampled an
            // individual clip, which is insufficient for a running graph.
            Keywords baseKeywords;
            int baseRole=0,basePriority=3;
            Metadata baseMetadata;
            using Request=bool(*)(RE::Actor*,RE::TESRace*,RE::BSAnimationGraphManager*,int*,Keywords*,int*,Handles*,Identifiers*,Metadata*);
            // ExtraAnimGraphPreload permits an empty optional base subgraph;
            // the loaded behavior project remains the base in that case.
            (void)native<Request>(0x1016340)(s.player,s.race,manager,&baseRole,&baseKeywords,&basePriority,
                &s.baseHandles,&s.baseIdentifiers,&baseMetadata);
            Keywords keywords;
            s.stage="exact-weapon-keywords";
            native<void(*)(RE::BGSObjectInstance*,Keywords*)>(0xEDA2F0)(&s.item,&keywords);
            if (keywords.empty()) return Load::Failed;
            int role=1,priority=3;
            Metadata metadata;
            s.stage="private-weapon-subgraph-request";
            if (!native<Request>(0x1016340)(s.player,s.race,manager,&role,&keywords,&priority,&s.handles,&s.identifiers,&metadata) ||
                s.handles.empty() || s.identifiers.empty() || metadata.count>1024) return Load::Failed;
            auto* block=metadata.head; auto* record=metadata.read;
            for (std::uint32_t i=0;i<metadata.count;++i) {
                if (!record) return Load::Failed;
                if (std::find(s.roots.begin(),s.roots.end(),record->name)==s.roots.end()) s.roots.push_back(record->name);
                ++record;
                if (block && reinterpret_cast<std::byte*>(record)>=block->records.data()+0x300) {
                    block=block->next; record=block?reinterpret_cast<Metadata::Record*>(block->records.data()):nullptr;
                }
            }
            s.phase=2;
            s.stage="native-subgraph-loading";
            return Load::Pending;
        }
        if (s.phase==2) {
            int priority=3;
            if (!native<bool(*)(ManagerRef*,Handles*,int*)>(0x7F4320)(&s.holder()->animationGraphManager,&s.baseHandles,&priority) ||
                !native<bool(*)(ManagerRef*,Handles*,int*)>(0x7F4320)(&s.holder()->animationGraphManager,&s.handles,&priority)) return Load::Pending;
            s.stage="native-skeleton-reference";
            if (!s.buildBones(manager->graph[1].get())) return Load::Failed;
            RE::BSScrapArray<RE::NiAVObject*> roots;
            s.stage="native-graph-variable-initialization";
            for (const auto& graph:manager->graph) {
                if (!graph) return Load::Failed;
                // Variable initialization receives this private graph directly.
                // Do not register the actor mediator: our indexed owner consumes
                // its native events after the native update returns.
                if (!native<bool(*)(void*,const GraphRef*)>(0xF2ED40)(static_cast<RE::IAnimationGraphManagerHolder*>(s.player),&graph)) return Load::Failed;
                roots.push_back(s.root.get());
            }
            s.stage="native-scene-binding";
            if (!native<bool(*)(RE::BSAnimationGraphManager*,decltype(roots)*,bool)>(0x168F940)(manager,&roots,true)) return Load::Failed;
            s.stage="native-first-person-activation";
            if (!native<bool(*)(RE::BSAnimationGraphManager*,std::uint32_t)>(0x1690240)(manager,1)) return Load::Failed;
            s.stage="private-variable-cache";
            const auto& cache=manager->variableCache;
            if (cache.graphToCacheFor!=manager->graph[1] || cache.lock!=&manager->updateLock ||
                cache.variableQuickLookup.empty() || cache.variableQuickLookup.size()>4096) return Load::Failed;
            for (const auto& graph:manager->graph) s.source(graph.get())->RegisterSink(&s);
            s.sinks=true; s.phase=3;
            s.stage="ready";
            ROCK_LOG_INFO(Animation,"AKIMBO private native graph ready form={:08X} instance={:#x} manager={:#x} graphs={} bones={} idleRoots={} authority=equipped-index-1",
                s.item.object->formID,reinterpret_cast<std::uintptr_t>(s.item.instanceData.get()),reinterpret_cast<std::uintptr_t>(manager),manager->graph.size(),s.boneCount,s.roots.size());
        }
        return s.phase==3?Load::Ready:Load::Failed;
    }
    bool Graph::canDestroy() const noexcept { return !_impl->constructed || native<bool(*)(void*)>(0x8122C0)(_impl->storage.data()); }
    RE::SimpleAnimationGraphManagerHolder* Graph::holder() const noexcept { return _impl->constructed?_impl->holder():nullptr; }
    RE::BSAnimationGraphManager* Graph::manager() const noexcept { auto* h=holder(); return h?h->animationGraphManager.get():nullptr; }
    bool Graph::notify(const RE::BSFixedString& event) { return _impl->phase==3 && _impl->holder()->NotifyAnimationGraphImpl(event); }
    void Graph::setInt(const RE::BSFixedString& name,std::int32_t value) { if (_impl->phase==3) native<void(*)(void*,const RE::BSFixedString*,int)>(0x80DC20)(holder(),&name,value); }
    void Graph::setFloat(const RE::BSFixedString& name,float value) { if (_impl->phase==3) native<void(*)(void*,const RE::BSFixedString*,float)>(0x80DC40)(holder(),&name,value); }
    void Graph::update(float dt)
    {
        auto& s=*_impl;
        if (s.phase!=3 || !std::isfinite(dt) || dt<0) return;
        const bool accepted=native<bool(*)(void*,float)>(0x80D6C0)(holder(),dt);
        if (!accepted && !s.updateRejected)
            ROCK_LOG_WARN(Animation,"AKIMBO native graph update rejected form={:08X} manager={:#x} dt={}",
                s.item.object->formID,reinterpret_cast<std::uintptr_t>(manager()),dt);
        s.updateRejected=!accepted;
    }
    std::size_t Graph::takeEvents(Events& output)
    {
        auto& s=*_impl; const auto count=s.eventCount;
        for (std::size_t i=0;i<count;++i) { output[i]=std::move(s.events[i]); s.events[i]={}; }
        s.eventCount=0; return count;
    }
    bool Graph::eventOverflow() const noexcept { return _impl->overflow; }
    const char* Graph::stage() const noexcept { return _impl->stage; }
    RE::NiNode* Graph::bone(const RE::BSFixedString& name) const noexcept
    {
        for (std::size_t i=0;i<_impl->boneCount;++i) if (_impl->bones[i]->name==name) return _impl->bones[i].get();
        return nullptr;
    }
    bool Graph::place(const RE::NiTransform& target) noexcept
    {
        auto& s=*_impl;
        if (s.phase!=3 || !s.weapon || !holding_grip_math::valid(holding_grip_math::fromNative(target))) return false;
        // This is a detached scene root. updateTransformsDown deliberately
        // leaves parentless nodes alone, so publish its world explicitly.
        s.root->local.MakeIdentity();
        s.root->world=s.root->local;
        f4vr::updateTransformsDown(s.root.get(),false);
        if (!native_secondary_pose::resolveBehaviorRootWorld(s.weapon->world,target,s.root->local)) return false;
        s.root->world=s.root->local;
        f4vr::updateTransformsDown(s.root.get(),false);
        return true;
    }
    std::span<const RE::BSFixedString> Graph::idleRoots() const noexcept { return _impl->roots; }
}
