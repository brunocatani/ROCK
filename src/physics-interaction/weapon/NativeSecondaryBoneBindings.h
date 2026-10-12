#pragma once

#include "physics-interaction/weapon/NativeEquippedActionGraph.h"
#include "physics-interaction/grab/SkinnedBoneOwner.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/weapon/WeaponScenePresentation.h"
#include "physics-interaction/weapon/NativeSecondaryPoseTransport.h"
#include "physics-interaction/weapon/NativePresentedPoseHistory.h"
#include "rock_support/Fo4VrRuntime.h"
#include "RE/Bethesda/BSSkin.h"
#include "RE/Bethesda/BSGeometry.h"
#include "RE/NetImmerse/NiCloningProcess.h"
#include <Windows.h>
#include <vector>

namespace rock::native_secondary_actions
{
    // Scene-owner lifetime. The native equipped model remains the rendered
    // model; only its animation targets are private to its equipped hand.
    class BoneBindings
    {
        struct OriginalBone
        {
            RE::NiPointer<RE::NiAVObject> node;
            RE::NiPointer<RE::NiAVObject> tree;
            RE::BSFixedString name;
            std::int32_t flatIndex{-1};
        };
        struct Slot { OriginalBone original; RE::NiNode* target{}; std::uint32_t index{}; };
        struct Skin
        {
            RE::NiPointer<RE::BSGeometry> geometry;
            RE::NiPointer<RE::BSSkin::Instance> original, instance;
            std::vector<Slot> slots;
        };
        struct Node {
            RE::NiPointer<RE::NiAVObject> actual;
            RE::NiNode* target{};
            std::size_t parentIndex{},anchorIndex{};
            RE::NiTransform originalLocal, beforeLocal, beforeWorld, presentedWorld, presentedLocal;
            native_secondary_pose_transport::EvaluatedPose transport;
            native_secondary_pose_transport::PresentedHistory history;
            RE::NiTransform lastNativeLocal{};
            bool sampledNative{};
        };
        std::vector<Skin> _skins;
        std::vector<Node> _nodes;
        std::uint64_t _sampleFrame{UINT64_MAX},_nextReport{},_nextBoneReport{};
        std::uint32_t _sampleFrames{},_movingSamples{};
        float _nativeTravel{},_nativeRotation{},_nativeScale{},_overwrite{};
        RE::BSFixedString _movingName;

        void sampleNative(std::uint64_t frame) noexcept
        {
            if (_sampleFrame==frame) return;
            _sampleFrame=frame;
            ++_sampleFrames;
            for (auto& node:_nodes) if (node.target) {
                const auto& current=node.target->local;
                if (node.sampledNative) {
                    const float travel=(current.translate-node.lastNativeLocal.translate).Length();
                    float matrix=0;
                    for (int row=0;row<3;++row) for (int column=0;column<3;++column)
                        matrix=(std::max)(matrix,std::abs(current.rotate.entry[row][column]-node.lastNativeLocal.rotate.entry[row][column]));
                    const float scale=std::abs(current.scale-node.lastNativeLocal.scale);
                    if (travel>0.0001f || matrix>0.0001f || scale>0.0001f) ++_movingSamples;
                    if (travel>_nativeTravel || matrix>_nativeRotation || scale>_nativeScale) _movingName=node.actual->name;
                    _nativeTravel=(std::max)(_nativeTravel,travel);
                    _nativeRotation=(std::max)(_nativeRotation,matrix);
                    _nativeScale=(std::max)(_nativeScale,scale);
                }
                node.lastNativeLocal=current;
                node.sampledNative=true;
            }
        }

        static bool flatBone(RE::NiAVObject* candidate,const RE::NiTransform* world,OriginalBone& out)
        {
            std::uintptr_t vtable{};
            if (!candidate || !native_memory::tryReadField(candidate,0,vtable) || vtable!=REL::Offset(0x2E580A8).address()) return false;
            auto* tree=static_cast<f4vr::BSFlattenedBoneTree*>(candidate);
            if (tree->numTransforms<=0 || tree->numTransforms>768 || !tree->transforms) return false;
            for (int i=0;i<tree->numTransforms;++i) {
                const auto& bone=tree->transforms[i];
                if (world==&bone.world || (bone.refNode && world==&bone.refNode->world)) {
                    out.tree.reset(candidate); out.node.reset(bone.refNode); out.name=bone.name; out.flatIndex=i;
                    return true;
                }
            }
            return false;
        }
        static bool originalBone(RE::NiAVObject* node,RE::NiAVObject* root,const RE::NiTransform* world,OriginalBone& out)
        {
            if (node && world==&node->world) { out.node.reset(node); out.name=node->name; return true; }
            if (flatBone(root,world,out)) return true;
            if (root) if (auto* parent=root->IsNode()) {
                if (parent->GetRuntimeData().children.size()>256) return false;
                for (const auto& child:parent->GetRuntimeData().children) if (flatBone(child.get(),world,out)) return true;
            }
            return false;
        }
        static bool arrays(RE::BSSkin::Instance* skin,RE::NiAVObject**& nodes,const RE::NiTransform**& worlds,std::uint32_t& count)
        {
            return native_memory::tryReadField(skin,0x10,nodes) && nodes &&
                native_memory::tryReadField(skin,0x28,worlds) && worlds &&
                native_memory::tryReadField(skin,0x38,count) && count<=768;
        }
    public:
        ~BoneBindings() { restore(); }
        BoneBindings()=default;
        BoneBindings(const BoneBindings&)=delete;
        BoneBindings& operator=(const BoneBindings&)=delete;
        void restore() noexcept
        {
            for (auto& skin:_skins) {
                auto& current=skin.geometry->GetRuntimeData().skinInstance;
                if (current.get()==skin.instance.get()) current=skin.original;
            }
            for (auto& node:_nodes) if (node.target) node.actual->local=node.originalLocal;
            _skins.clear(); _nodes.clear();
            _sampleFrame=UINT64_MAX; _nextReport=0; _nextBoneReport=0; _sampleFrames=_movingSamples=0;
            _nativeTravel=_nativeRotation=_nativeScale=_overwrite=0; _movingName={};
        }
        bool bind(RE::NiAVObject* root,native_equipped_action_graph::Graph& graph)
        {
            restore();
            if (!root || !nativeSkinLayoutVerified()) return false;
            constexpr auto prefix=std::to_array<std::uint8_t>({0x48,0x89,0x5C,0x24,0x08,0x48,0x89,0x74,0x24,0x10,0x57,0x48,0x83,0xEC,0x20});
            std::array<std::uint8_t,prefix.size()> live{};
            if (!native_memory::guardedCopyFromMemory(reinterpret_cast<void*>(REL::Offset(0x1C13FF0).address()),live.data(),live.size()) || live!=prefix) return false;
            // Native binding resolves each graph bone with 1C20840, a
            // pre-order depth-first search (1C220E0) that stops at the first
            // same-named node. Models can repeat a bone name (the 1911 has a
            // WeaponMagazine with its own rounds under both P-Receiver and
            // P-Mag); only the first one is animated, the others keep their
            // authored pose. Driving every duplicate pushed the second set of
            // rounds up out of the slide.
            std::vector<std::pair<RE::BSFixedString,RE::NiAVObject*>> owners;
            {
                std::vector<RE::NiAVObject*> stack{root};
                std::size_t visited=0;
                while (!stack.empty()) {
                    if (++visited>8192) return false;
                    auto* object=stack.back(); stack.pop_back();
                    if (object!=root && object->IsNode() && graph.bone(object->name) &&
                        std::none_of(owners.begin(),owners.end(),[&](const auto& owner) { return owner.first==object->name; }))
                        owners.emplace_back(object->name,object);
                    if (auto* node=object->IsNode()) {
                        const auto& children=node->GetRuntimeData().children;
                        // Reverse push so children pop in native (forward) order.
                        for (auto i=children.size();i>0;--i) if (children[i-1]) stack.push_back(children[i-1].get());
                    }
                }
            }
            const auto nativeOwner=[&](RE::NiAVObject* object) {
                return std::any_of(owners.begin(),owners.end(),[&](const auto& owner) { return owner.second==object; });
            };
            std::vector<RE::NiAVObject*> objects;
            objects.reserve(256); objects.push_back(root);
            _nodes.push_back({RE::NiPointer<RE::NiAVObject>{root},nullptr,0,0,root->local,{},{},{}});
            std::size_t boundNodes=0;
            for (std::size_t i=0;i<objects.size();++i) {
                auto* object=objects[i];
                if (auto* node=object->IsNode()) {
                    for (const auto& child:node->GetRuntimeData().children) if (child) {
                        if (objects.size()>=8192) return false;
                        objects.push_back(child.get());
                        auto* target=child->IsNode() && nativeOwner(child.get())?graph.bone(child->name):nullptr;
                        if (target) ++boundNodes;
                        const auto anchor=_nodes[i].target?i:_nodes[i].anchorIndex;
                        _nodes.push_back({RE::NiPointer<RE::NiAVObject>{child.get()},target,i,anchor,child->local,{},{},{}});
                    }
                }
                auto* geometry=object->IsGeometry();
                if (!geometry) geometry=object->IsTriShape();
                auto* instance=geometry?geometry->GetRuntimeData().skinInstance.get():nullptr;
                if (!instance) continue;
                RE::NiAVObject** nodes{}; const RE::NiTransform** worlds{}; std::uint32_t count{}; RE::NiAVObject* skinRoot{};
                if (!arrays(instance,nodes,worlds,count) || !native_memory::tryReadField(instance,0x48,skinRoot)) return false;
                Skin skin; skin.geometry.reset(geometry); skin.original.reset(instance); skin.slots.reserve(count);
                for (std::uint32_t j=0;j<count;++j) {
                    OriginalBone original;
                    if (!originalBone(nodes[j],skinRoot,worlds[j],original)) continue;
                    if (auto* target=graph.bone(original.name)) skin.slots.push_back({std::move(original),target,j});
                }
                // Native skin cloning gives this geometry independent palette
                // arrays while retaining its original bind matrices/materials.
                // The original instance stays intact for exact restoration.
                RE::NiCloningProcess cloning{};
                cloning.copyType=RE::NiCloningProcess::CopyType::kCopyExact;
                cloning.scale={1.0f,1.0f,1.0f};
                using Clone=RE::NiObject*(*)(RE::NiObject*,RE::NiCloningProcess*);
                auto* copy=reinterpret_cast<Clone>(REL::Offset(0x1C13FF0).address())(instance,&cloning);
                skin.instance.reset(static_cast<RE::BSSkin::Instance*>(copy));
                RE::NiAVObject** copyNodes{}; const RE::NiTransform** copyWorlds{}; std::uint32_t copyCount{};
                if (!copy || copy==instance || !arrays(skin.instance.get(),copyNodes,copyWorlds,copyCount) ||
                    copyCount!=count || copyNodes==nodes || copyWorlds==worlds) return false;
                _skins.push_back(std::move(skin));
            }
            // Preparation is read-only. Commit after every shape was inspected.
            for (auto& skin:_skins) {
                RE::NiAVObject** nodes{}; const RE::NiTransform** worlds{}; std::uint32_t count{};
                if (!arrays(skin.instance.get(),nodes,worlds,count)) { restore(); return false; }
                for (const auto& slot:skin.slots) {
                    if (slot.index>=count) { restore(); return false; }
                    nodes[slot.index]=slot.target; worlds[slot.index]=&slot.target->world;
                }
                skin.geometry->GetRuntimeData().skinInstance=skin.instance;
            }
            ROCK_LOG_INFO(Animation,"AKIMBO independent bone binding objects={} skins={} nodes={} descendantTransport=complete",
                objects.size(),_skins.size(),boundNodes);
            // First pose sample after the graph has evaluated, not the bind pose.
            _nextBoneReport=GetTickCount64()+1000;
            return true;
        }
        void carryPublished(const RE::NiTransform& delta) noexcept
        {
            for (auto& node:_nodes) node.transport.carryPublished(delta);
        }
        bool present(std::uint64_t frame) noexcept
        {
            if (_nodes.empty()) return false;
            sampleNative(frame);
            // Snapshot evaluated worlds before moving any parent. Animated
            // descendants can differ from their local transforms; rigidly carry
            // that output with the corrected bone instead of discarding it.
            for (std::size_t i=0;i<_nodes.size();++i) {
                auto& node=_nodes[i];
                if (i && node.actual->parent!=_nodes[node.parentIndex].actual.get()) return false;
                node.beforeWorld=node.actual->world;
                node.beforeLocal=node.actual->local;
                if (!weapon_scene_presentation::isFiniteTransform(node.beforeWorld)) return false;
            }
            _nodes[0].presentedWorld=_nodes[0].beforeWorld;
            for (std::size_t i=1;i<_nodes.size();++i) {
                auto& node=_nodes[i];
                const auto& parent=_nodes[node.parentIndex];
                if (node.target) {
                    // Native graph binding (1785010) drives a same-named model
                    // node with the bone's LOCAL transform under the model's
                    // own parent. Model hierarchies differ from the skeleton
                    // (BAS rounds hang under WeaponMagazineHelper, the skeleton
                    // has them under WeaponMagazine); copying the bone's WORLD
                    // dropped that intermediate offset and moved the rounds
                    // out of the magazine.
                    node.presentedLocal=node.target->local;
                    node.presentedWorld=transform_math::composeTransforms(parent.presentedWorld,node.presentedLocal);
                    _overwrite=(std::max)(_overwrite,(node.beforeWorld.translate-node.presentedWorld.translate).Length());
                } else {
                    const auto& anchor=_nodes[node.anchorIndex];
                    if (!node.transport.reframe(anchor.beforeWorld,anchor.presentedWorld,node.beforeWorld,node.presentedWorld)) {
                        ROCK_LOG_SAMPLE_WARN(Animation,1000,"AKIMBO part pose rejected stage=descendant-frame name='{}' anchor='{}'",
                            node.actual->name.c_str(),anchor.actual->name.c_str());
                        return false;
                    }
                }
                if (!weapon_scene_presentation::isFiniteTransform(node.presentedWorld)) return false;
            }
            for (std::size_t i=1;i<_nodes.size();++i) {
                auto& node=_nodes[i];
                node.actual->world=node.presentedWorld;
                if (node.target) node.actual->local=node.presentedLocal;
            }
            // 1C23777..3849 copies current world (+70) to previous (+C0)
            // before native placement. The shader accumulator (281BE35..BE4D)
            // and shader setup (28796C0..97AE) consume that prior transform.
            // Give them the prior presented pose, not the transient melee arm
            // pose or this frame's earlier private graph placement. True hand
            // movement and native part animation remain in the history.
            float historyError=0;
            for (auto& node:_nodes) {
                const auto& previous=node.history.publish(frame,node.actual->world);
                historyError=(std::max)(historyError,(node.actual->previousWorld.translate-previous.translate).Length());
                node.actual->previousWorld=previous;
            }
            ROCK_LOG_SAMPLE_INFO(Animation,1000,"AKIMBO secondary motion history frame={} nodes={} correctedPreviousMaxGame={:.3f} source=previous-presentation",
                frame,_nodes.size(),historyError);
            const auto now=GetTickCount64();
            // Diagnostic: per bound model bone, its authored rest local, the
            // local the model's own graph/native code left this frame, and
            // the private skeleton's local that replaced it. Every 5 seconds.
            if (now>=_nextBoneReport) {
                _nextBoneReport=now+5000;
                std::size_t reported=0;
                for (const auto& node:_nodes) {
                    if (!node.target || ++reported>24) continue;
                    const auto& rest=node.originalLocal.translate;
                    const auto& model=node.beforeLocal.translate;
                    const auto& skeleton=node.target->local.translate;
                    ROCK_LOG_INFO(Animation,"AKIMBO bound bone pose name='{}' parent='{}' rest=({:.3f},{:.3f},{:.3f}) modelFrame=({:.3f},{:.3f},{:.3f}) skeleton=({:.3f},{:.3f},{:.3f}) skeletonParent='{}'",
                        node.actual->name.c_str(),node.actual->parent?node.actual->parent->name.c_str():"",
                        rest.x,rest.y,rest.z,model.x,model.y,model.z,skeleton.x,skeleton.y,skeleton.z,
                        node.target->parent?node.target->parent->name.c_str():"");
                }
            }
            if (now>=_nextReport) {
                ROCK_LOG_INFO(Animation,"AKIMBO native part evaluation frame={} frames={} changedSamples={} nativeLocalTravelGame={:.5f} nativeLocalMatrixDelta={:.5f} nativeLocalScaleDelta={:.5f} part='{}' presentationRepairGame={:.3f} reference=bone-parent-local",
                    frame,_sampleFrames,_movingSamples,_nativeTravel,_nativeRotation,_nativeScale,_movingName.c_str(),_overwrite);
                _nextReport=now+1000; _sampleFrames=_movingSamples=0;
                _nativeTravel=_nativeRotation=_nativeScale=_overwrite=0; _movingName={};
            }
            return true;
        }
    };
}
