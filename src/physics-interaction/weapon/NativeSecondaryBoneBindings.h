#pragma once

#include "physics-interaction/weapon/NativeEquippedActionGraph.h"
#include "physics-interaction/grab/SkinnedBoneOwner.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/weapon/WeaponScenePresentation.h"
#include "physics-interaction/weapon/NativeSecondaryPoseTransport.h"
#include "rock_support/Fo4VrRuntime.h"
#include "RE/Bethesda/BSSkin.h"
#include "RE/Bethesda/BSGeometry.h"
#include "RE/NetImmerse/NiCloningProcess.h"
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
            RE::NiTransform originalLocal, beforeWorld, presentedWorld, presentedLocal;
            native_secondary_pose_transport::EvaluatedPose transport;
        };
        std::vector<Skin> _skins;
        std::vector<Node> _nodes;

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
        }
        bool bind(RE::NiAVObject* root,native_equipped_action_graph::Graph& graph)
        {
            restore();
            if (!root || !nativeSkinLayoutVerified()) return false;
            constexpr auto prefix=std::to_array<std::uint8_t>({0x48,0x89,0x5C,0x24,0x08,0x48,0x89,0x74,0x24,0x10,0x57,0x48,0x83,0xEC,0x20});
            std::array<std::uint8_t,prefix.size()> live{};
            if (!native_memory::guardedCopyFromMemory(reinterpret_cast<void*>(REL::Offset(0x1C13FF0).address()),live.data(),live.size()) || live!=prefix) return false;
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
                        auto* target=child->IsNode()?graph.bone(child->name):nullptr;
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
            return true;
        }
        void carryPublished(const RE::NiTransform& delta) noexcept
        {
            for (auto& node:_nodes) node.transport.carryPublished(delta);
        }
        bool present() noexcept
        {
            if (_nodes.empty()) return false;
            // Snapshot evaluated worlds before moving any parent. Animated
            // descendants can differ from their local transforms; rigidly carry
            // that output with the corrected bone instead of discarding it.
            for (std::size_t i=0;i<_nodes.size();++i) {
                auto& node=_nodes[i];
                if (i && node.actual->parent!=_nodes[node.parentIndex].actual.get()) return false;
                node.beforeWorld=node.actual->world;
                if (!weapon_scene_presentation::isFiniteTransform(node.beforeWorld)) return false;
            }
            _nodes[0].presentedWorld=_nodes[0].beforeWorld;
            for (std::size_t i=1;i<_nodes.size();++i) {
                auto& node=_nodes[i];
                const auto& parent=_nodes[node.parentIndex];
                if (node.target) {
                    node.presentedWorld=node.target->world;
                    if (!native_secondary_pose_transport::relative(parent.presentedWorld,node.presentedWorld,node.presentedLocal)) {
                        ROCK_LOG_SAMPLE_WARN(Animation,1000,"AKIMBO part pose rejected stage=bound-local name='{}'",node.actual->name.c_str());
                        return false;
                    }
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
            return true;
        }
    };
}
