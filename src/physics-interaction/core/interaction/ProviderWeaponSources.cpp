#include "physics-interaction/core/PhysicsInteractionInternal.h"
#include "physics-interaction/actor/ActorEquipmentGrab.h"
#include <algorithm>
#include <cstring>

namespace rock {
void PhysicsInteraction::refreshProviderWeaponSources() {
    auto* root=resolveEquippedWeaponInteractionNode();
    const auto generation=_weaponCollision.getCurrentWeaponGenerationKey();
    auto& catalog=_providerSources;
    if (root==catalog.root && generation==catalog.generation) return;
    catalog.count=0; catalog.root=root; catalog.generation=generation; catalog.overflow=false;
    if (!root || !generation) return;
    struct Pending { RE::NiAVObject* node; std::uint64_t parent; std::uint32_t depth; };
    std::array<Pending,provider::WeaponSourceCatalog::Capacity> pending{};
    std::size_t pendingCount=1;
    pending[0]={root,0,0};
    const auto evidence=_weaponCollision.getProfileEvidenceDescriptors();
    while (pendingCount) {
        const auto current=pending[--pendingCount];
        if (!current.node) continue;
        if (catalog.count==catalog.entries.size() || current.depth>64) { catalog.overflow=true; break; }
        bool seen=false;
        for (std::uint32_t i=0;i<catalog.count;++i) if (catalog.entries[i].node==current.node) { seen=true; break; }
        if (seen) continue;
        auto& entry=catalog.entries[catalog.count++];
        entry={}; entry.node=current.node;
        entry.value.weaponGenerationKey=generation;
        entry.value.sourceKey=provider::WeaponSourceCatalog::nextKey.fetch_add(1,std::memory_order_relaxed);
        entry.value.parentKey=current.parent;
        const auto& local=current.node->local;
        for (std::size_t row=0;row<3;++row) for (std::size_t col=0;col<3;++col) entry.value.sourceParentLocal.rotate[row*3+col]=local.rotate.entry[row][col];
        entry.value.sourceParentLocal.translate[0]=local.translate.x;
        entry.value.sourceParentLocal.translate[1]=local.translate.y;
        entry.value.sourceParentLocal.translate[2]=local.translate.z;
        entry.value.sourceParentLocal.scale=local.scale;
        const auto* name=current.node->name.c_str();
        if (name) std::copy_n(name,std::min<std::size_t>(std::strlen(name),63),entry.value.name);
        for (const auto& descriptor:evidence) if (descriptor.valid && descriptor.sourceRootAddress==reinterpret_cast<std::uintptr_t>(current.node)) {
            entry.value.bodyId=descriptor.bodyId; break;
        }
        if (auto* node=current.node->IsNode()) {
            auto& children=node->GetRuntimeData().children;
            for (decltype(children.size()) i=0;i<children.size();++i) {
                if (!children[i]) continue;
                if (pendingCount==pending.size()) { catalog.overflow=true; break; }
                pending[pendingCount++]={children[i].get(),entry.value.sourceKey,current.depth+1};
            }
        }
        if (catalog.overflow) break;
    }
    // A partial catalog must never turn an ambiguous selector into a unique one.
    if (catalog.overflow) catalog.count=0;
}
std::uintptr_t PhysicsInteraction::resolveProviderWeaponSource(std::uint64_t generation,std::uint64_t key) const {
    const auto& catalog=_providerSources;
    auto* root=resolveEquippedWeaponInteractionNode();
    if (!key || !generation || generation!=catalog.generation || generation!=_weaponCollision.getCurrentWeaponGenerationKey() || root!=catalog.root || catalog.overflow) return 0;
    for (std::uint32_t i=0;i<catalog.count;++i) if (catalog.entries[i].value.sourceKey==key) {
        auto* node=catalog.entries[i].node;
        return actor_equipment_grab::nodeContainsNode(root,node,64)?reinterpret_cast<std::uintptr_t>(node):0;
    }
    return 0;
}
std::uint64_t PhysicsInteraction::providerWeaponSourceKey(std::uint64_t generation,std::uintptr_t node) const {
    const auto& catalog=_providerSources;
    if (!node || !generation || generation!=catalog.generation || generation!=_weaponCollision.getCurrentWeaponGenerationKey() || resolveEquippedWeaponInteractionNode()!=catalog.root || catalog.overflow) return 0;
    for (std::uint32_t i=0;i<catalog.count;++i) if (reinterpret_cast<std::uintptr_t>(catalog.entries[i].node)==node) return catalog.entries[i].value.sourceKey;
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
    if (!root || !finiteNiTransform(root->world) || !finiteNiTransform(node->world) || !finiteNiTransform(node->local)) return api::Status::NotReady;
    const auto copy=[](api::Transform& out,const RE::NiTransform& in) {
        for (std::size_t row=0;row<3;++row) for (std::size_t col=0;col<3;++col) out.rotate[row*3+col]=in.rotate.entry[row][col];
        out.translate[0]=in.translate.x; out.translate[1]=in.translate.y; out.translate[2]=in.translate.z; out.scale=in.scale;
    };
    const auto relative=transform_math::composeTransforms(transform_math::invertTransform(root->world),node->world);
    if (!finiteNiTransform(relative)) return api::Status::NotReady;
    output.weaponGenerationKey=generation; output.sourceKey=key;
    copy(output.sourceParentLocal,node->local); copy(output.weaponRootLocal,relative); copy(output.world,node->world);
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
    for (std::uint32_t i=0;i<_providerSources.count;++i) if (_providerSources.entries[i].node==parent) {
        const auto keyOfParent=_providerSources.entries[i].value.sourceKey;
        auto& children=parent->GetRuntimeData().children;
        for (decltype(children.size()) index=0;index<children.size();++index) if (children[index].get()==node) {
            parentKey=keyOfParent; childIndex=index; return api::Status::Ok;
        }
        return api::Status::TargetUnavailable;
    }
    return api::Status::TargetUnavailable;
}
}
