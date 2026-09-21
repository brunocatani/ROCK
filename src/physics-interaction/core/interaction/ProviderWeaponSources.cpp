#include "physics-interaction/core/PhysicsInteractionInternal.h"
#include "physics-interaction/weapon/WeaponHierarchy.h"
#include <functional>
#include <algorithm>
#include <cstring>

namespace rock {
namespace {
    void copySourceTransform(api::Transform& out, const RE::NiTransform& in) {
        for (std::size_t row=0;row<3;++row) for (std::size_t col=0;col<3;++col)
            out.rotate[row*3+col]=in.rotate.entry[row][col];
        out.translate[0]=in.translate.x; out.translate[1]=in.translate.y;
        out.translate[2]=in.translate.z; out.scale=in.scale;
    }
}
void PhysicsInteraction::refreshProviderWeaponSources() {
    auto* root=resolveEquippedWeaponInteractionNode();
    const auto generation=_weaponCollision.getCurrentWeaponGenerationKey();
    auto& catalog=_providerSources;
    if (root!=catalog.root || generation!=catalog.generation) catalog.clear();
    if (!root || !generation) { catalog.clear(); return; }
    catalog.root=root; catalog.generation=generation; catalog.overflow=false;

    // Check the live hierarchy once per API batch. Weapon collision generation
    // does not change for every scene edit; a generation-only cache misses those.
    std::array<RE::NiAVObject*,provider::WeaponSourceCatalog::Capacity> nodes{};
    std::size_t count=0;
    const auto visit=[&](auto&& self,RE::NiAVObject* object,std::uint32_t depth)->bool {
        if (depth>64 || count==nodes.size()) return false;
        nodes[count++]=object;
        if (auto* node=object->IsNode()) {
            auto& children=node->GetRuntimeData().children;
            for (decltype(children.size()) i=0;i<children.size();++i) {
                auto* child=children[i].get();
                if (child && (child->parent!=node || !self(self,child,depth+1))) return false;
            }
        }
        return true;
    };
    const auto less=std::less<RE::NiAVObject*>{};
    const bool complete=visit(visit,root,0);
    std::sort(nodes.begin(),nodes.begin()+count,less);
    if (!complete || std::adjacent_find(nodes.begin(),nodes.begin()+count)!=nodes.begin()+count) {
        catalog.clear(); catalog.root=root; catalog.generation=generation; catalog.overflow=true;
        return; // No partial catalog may turn an ambiguous name into a unique one.
    }

    // Keep surviving identities, including their captured local transform. Pins
    // keep a removed allocation from being recycled as a different keyed node.
    std::uint32_t retained=0;
    for (std::uint32_t i=0;i<catalog.count;++i) {
        auto& entry=catalog.entries[i];
        if (std::binary_search(nodes.begin(),nodes.begin()+count,entry.node.get(),less)) {
            if (retained!=i) catalog.entries[retained]=std::move(entry);
            ++retained;
        }
    }
    for (std::uint32_t i=retained;i<catalog.count;++i) catalog.entries[i]={};
    catalog.count=retained;
    const auto findNode=[&](RE::NiAVObject* node,std::uint32_t size) {
        return std::lower_bound(catalog.entries.begin(),catalog.entries.begin()+size,node,
            [&](const auto& entry,auto* value) {return less(entry.node.get(),value);});
    };
    for (std::size_t i=0;i<count;++i) {
        auto* node=nodes[i];
        const auto found=findNode(node,retained);
        if (found!=catalog.entries.begin()+retained && found->node.get()==node) continue;
        auto& entry=catalog.entries[catalog.count++];
        entry.node.reset(node);
        entry.value={};
        entry.value.weaponGenerationKey=generation;
        entry.value.sourceKey=provider::WeaponSourceCatalog::nextKey.fetch_add(1,std::memory_order_relaxed);
        copySourceTransform(entry.value.sourceParentLocal,node->local);
    }
    std::sort(catalog.entries.begin(),catalog.entries.begin()+catalog.count,
        [&](const auto& a,const auto& b) {return less(a.node.get(),b.node.get());});
    const auto evidence=_weaponCollision.getProfileEvidenceDescriptors();
    for (std::uint32_t i=0;i<catalog.count;++i) {
        auto& entry=catalog.entries[i];
        auto* node=entry.node.get();
        const auto parent=findNode(node->parent,catalog.count);
        const auto parentKey=node==root ? 0 : parent->value.sourceKey;
        if (entry.value.parentKey!=parentKey) copySourceTransform(entry.value.sourceParentLocal,node->local);
        entry.value.parentKey=parentKey;
        std::fill(std::begin(entry.value.name),std::end(entry.value.name),0);
        const auto* name=node->name.c_str();
        if (name) for (std::size_t n=0;n<63 && name[n];++n) entry.value.name[n]=name[n];
        entry.value.bodyId=0x7FFFFFFF;
        for (const auto& descriptor:evidence) if (descriptor.valid && descriptor.sourceRootAddress==reinterpret_cast<std::uintptr_t>(node)) {
            entry.value.bodyId=descriptor.bodyId; break;
        }
    }
}
std::uintptr_t PhysicsInteraction::resolveProviderWeaponSource(std::uint64_t generation,std::uint64_t key) const {
    const auto& catalog=_providerSources;
    auto* root=resolveEquippedWeaponInteractionNode();
    if (!key || !generation || generation!=catalog.generation || generation!=_weaponCollision.getCurrentWeaponGenerationKey() || root!=catalog.root || catalog.overflow) return 0;
    for (std::uint32_t i=0;i<catalog.count;++i) if (catalog.entries[i].value.sourceKey==key) {
        return reinterpret_cast<std::uintptr_t>(catalog.entries[i].node.get());
    }
    return 0;
}
std::uint64_t PhysicsInteraction::providerWeaponSourceKey(std::uint64_t generation,std::uintptr_t node) const {
    const auto& catalog=_providerSources;
    if (!node || !generation || generation!=catalog.generation || generation!=_weaponCollision.getCurrentWeaponGenerationKey() || resolveEquippedWeaponInteractionNode()!=catalog.root || catalog.overflow) return 0;
    for (std::uint32_t i=0;i<catalog.count;++i) if (reinterpret_cast<std::uintptr_t>(catalog.entries[i].node.get())==node) return catalog.entries[i].value.sourceKey;
    return 0;
}
std::uint64_t PhysicsInteraction::providerWeaponSourceKeyForBody(std::uint64_t generation,std::uint32_t body) const {
    const auto evidence=_weaponCollision.getProfileEvidenceDescriptors();
    const auto* descriptor=evidence.find(body);
    return descriptor?providerWeaponSourceKey(generation,descriptor->sourceRootAddress):0;
}
std::uintptr_t PhysicsInteraction::resolveProviderWeaponSourceName(std::uint64_t generation,const char* name) const {
    if (!name || !*name || _providerSources.overflow || generation!=_providerSources.generation) return 0;
    std::uint64_t found=0;
    for (std::uint32_t i=0;i<_providerSources.count;++i) {
        const auto& entry=_providerSources.entries[i];
        if (std::strcmp(name,entry.value.name)!=0) continue;
        if (found) return 0;
        found=entry.value.sourceKey;
    }
    return resolveProviderWeaponSource(generation,found);
}
api::Status PhysicsInteraction::copyProviderWeaponSources(std::uint64_t generation,std::uint32_t offset,provider::WeaponSourceRecord* output,std::uint32_t capacity,std::uint32_t& copied,std::uint32_t& total) const {
    copied=0; total=0;
    if (!generation || generation!=_providerSources.generation || generation!=_weaponCollision.getCurrentWeaponGenerationKey() || resolveEquippedWeaponInteractionNode()!=_providerSources.root) return api::Status::GenerationMismatch;
    if (_providerSources.overflow) return api::Status::CapacityFull;
    total=_providerSources.count; copied=std::min(capacity,total>offset?total-offset:0);
    for (std::uint32_t i=0;i<copied;++i) output[i]=_providerSources.entries[offset+i].value;
    return api::Status::Ok;
}
}

namespace rock {
api::Status PhysicsInteraction::queryProviderWeaponSourcePose(std::uint64_t generation,std::uint64_t key,provider::WeaponSourcePose& output) const {
    output={};
    auto* node=reinterpret_cast<RE::NiAVObject*>(resolveProviderWeaponSource(generation,key));
    if (!node) return api::Status::GenerationMismatch;
    auto* root=resolveEquippedWeaponInteractionNode();
    RE::NiTransform relative{};
    if (!root || !finiteNiTransform(root->world) || !finiteNiTransform(node->local) ||
        !weapon_hierarchy::tryResolveDescendantLocalTransform(root,node,relative)) return api::Status::NotReady;
    const auto world=transform_math::composeTransforms(root->world,relative);
    if (!finiteNiTransform(world)) return api::Status::NotReady;
    output.weaponGenerationKey=generation; output.sourceKey=key;
    copySourceTransform(output.sourceParentLocal,node->local);
    copySourceTransform(output.weaponRootLocal,relative);
    copySourceTransform(output.world,world);
    return api::Status::Ok;
}
}

namespace rock {
api::Status PhysicsInteraction::queryProviderWeaponSourcePath(std::uint64_t generation,std::uint64_t key,std::uint64_t& parentKey,std::uint32_t& childIndex) const {
    parentKey=0; childIndex=0;
    auto* node=reinterpret_cast<RE::NiAVObject*>(resolveProviderWeaponSource(generation,key));
    if (!node) return api::Status::GenerationMismatch;
    if (node==_providerSources.root) return api::Status::Ok;
    auto* parent=node->parent;
    if (!parent) return api::Status::TargetUnavailable;
    for (std::uint32_t i=0;i<_providerSources.count;++i) if (_providerSources.entries[i].node.get()==parent) {
        const auto keyOfParent=_providerSources.entries[i].value.sourceKey;
        auto& children=parent->GetRuntimeData().children;
        for (decltype(children.size()) index=0;index<children.size();++index) if (children[index].get()==node) {
            parentKey=keyOfParent; childIndex=static_cast<std::uint32_t>(index); return api::Status::Ok;
        }
        return api::Status::TargetUnavailable;
    }
    return api::Status::TargetUnavailable;
}
}
