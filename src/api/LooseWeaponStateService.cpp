#include "LooseWeaponStateService.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/PhysicsLog.h"
#include "RE/Bethesda/TESObjectREFRs.h"
#include "RE/NetImmerse/NiNode.h"
#include <REL/Relocation.h>
#include <array>
#include <algorithm>
#include <mutex>
#include <cstring>
#include <cmath>

namespace rock::loose_weapon_state {
namespace {
    struct Entry {
        api::OwnerToken owner{};contract::ItemKey key{};contract::ItemState state{},originalState{};
        RE::NiPointer<RE::NiAVObject> model;
        bool retiring{};std::uintptr_t failedModel{};
    };
    // Value-only descriptions; native model callbacks never retain engine
    // pointers or call back into a consumer. Registry mutations are bounded.
    std::array<Entry,256> entries;
    std::mutex mutex;
    using Set3D=void(*)(RE::TESObjectREFR*,RE::NiAVObject*,bool);
    Set3D original{};
    bool valid(const contract::ItemKey& key) {return key.weapon&&key.owner&&key.unique<=65535;}
    bool valid(const contract::ItemState& state) {
        if(state.size!=sizeof(state)||!state.count||state.count>contract::kMaxItemParts)return false;
        for(std::uint32_t i=0;i<state.count;++i) {
            const auto& p=state.parts[i];
            if(!p.path[0]||!std::memchr(p.path,0,sizeof(p.path))||p.hidden>1||!std::isfinite(p.local.scale)||p.local.scale<0)return false;
            for(float v:p.local.rotate)if(!std::isfinite(v))return false;
            for(float v:p.local.translate)if(!std::isfinite(v))return false;
            for(std::uint32_t j=0;j<i;++j)if(!std::strcmp(p.path,state.parts[j].path))return false;
        }
        return true;
    }
    RE::NiAVObject* child(RE::NiAVObject* parent,std::string_view name) {
        auto* branch=parent?parent->IsNode():nullptr;if(!branch)return nullptr;
        RE::NiAVObject* found{};
        for(const auto& c:branch->children)if(c&&c->name.c_str()&&name==c->name.c_str()) {
            if(found)return nullptr;found=c.get();
        }
        return found;
    }
    RE::NiAVObject* path(RE::NiAVObject* root,std::string_view name) {
        auto* current=root;
        for(unsigned depth=0;!name.empty()&&depth<64;++depth) {
            const auto slash=name.find('/');current=child(current,name.substr(0,slash));
            if(!current)return nullptr;if(slash==name.npos)return current;name.remove_prefix(slash+1);
        }
        return nullptr;
    }
    RE::NiAVObject* weaponRoot(RE::NiAVObject* root) {
        std::array<RE::NiAVObject*,2048> nodes{};nodes[0]=root;std::size_t count=1;RE::NiAVObject* found{};
        for(std::size_t i=0;i<count;++i) {
            auto* current=nodes[i];if(!current)return nullptr;
            if(current->name.c_str()&&!std::strcmp(current->name.c_str(),"Weapon")) {
                if(found)return nullptr;found=current;
            }
            if(auto* node=current->IsNode())for(const auto& c:node->children)if(c) {
                if(count==nodes.size())return nullptr;nodes[count++]=c.get();
            }
        }
        return found?found:root;
    }
    void set3D(RE::TESObjectREFR* ref,RE::NiAVObject* model,bool queued) noexcept {
        // Native caller owns the model through Set3D. Apply only to a detached
        // incoming tree before native publication; never touch shared templates.
        try {if(model&&!model->parent)apply(ref,model);}catch(...) {ROCK_LOG_ERROR(Weapon,"Loose item model state application failed");}
        original(ref,model,queued);
    }
}
api::Status set(api::OwnerToken owner,const contract::ItemKey& key,const contract::ItemState& state) {
    if(!original)return api::Status::NotReady;
    if(!owner||!valid(key)||!valid(state))return api::Status::InvalidArgument;
    std::scoped_lock lock(mutex);
    auto found=std::find_if(entries.begin(),entries.end(),[&](const auto& e){return e.owner&&e.key==key;});
    if(found!=entries.end()&&found->owner!=owner)return api::Status::NotReady;
    if(found==entries.end())found=std::find_if(entries.begin(),entries.end(),[](const auto& e){return !e.owner&&!e.retiring;});
    if(found==entries.end())return api::Status::CapacityFull;
    found->owner=owner;found->key=key;found->state=state;found->failedModel=0;return api::Status::Ok;
}
api::Status move(api::OwnerToken owner,const contract::ItemKey& before,const contract::ItemKey& after) {
    if(!valid(before)||before.weapon!=after.weapon||(after.owner&&!valid(after)))return api::Status::InvalidArgument;
    std::scoped_lock lock(mutex);
    auto from=std::find_if(entries.begin(),entries.end(),[&](const auto& e){return e.owner==owner&&e.key==before;});
    if(from==entries.end()||before==after)return api::Status::Ok;
    if(!after.owner){from->owner=0;from->retiring=bool(from->model);return api::Status::Ok;}
    const auto to=std::find_if(entries.begin(),entries.end(),[&](const auto& e){return e.owner&&e.key==after;});
    if(to!=entries.end())return api::Status::NotReady;
    from->key=after;from->model.reset();from->failedModel=0;return api::Status::Ok;
}
api::Status clear(api::OwnerToken owner) {
    std::scoped_lock lock(mutex);for(auto& e:entries)if(e.owner==owner){e.owner=0;e.retiring=bool(e.model);}return api::Status::Ok;
}
std::size_t references(std::span<std::uint32_t> output) {
    std::scoped_lock lock(mutex);std::size_t count{};
    for(const auto& e:entries)if(e.owner&&!e.failedModel&&!e.key.unique&&count<output.size())output[count++]=e.key.owner;
    return count;
}
bool owns(std::uint32_t reference) {
    std::scoped_lock lock(mutex);
    return std::any_of(entries.begin(),entries.end(),[&](const auto& e){return e.owner&&!e.failedModel&&!e.key.unique&&e.key.owner==reference;});
}
bool apply(RE::TESObjectREFR* ref,RE::NiAVObject* model) {
    if(!ref||!model||!ref->GetObjectReference())return false;
    std::scoped_lock lock(mutex);
    const contract::ItemKey key{ref->GetObjectReference()->GetFormID(),ref->GetFormID(),0};
    const auto found=std::find_if(entries.begin(),entries.end(),[&](const auto& e){return e.owner&&e.key==key;});
    if(found==entries.end())return false;
    if(found->failedModel==reinterpret_cast<std::uintptr_t>(model))return false;
    auto* root=weaponRoot(model);if(!root)return false;
    std::array<RE::NiAVObject*,contract::kMaxItemParts> targets{};
    for(std::uint32_t i=0;i<found->state.count;++i) {
        targets[i]=path(root,found->state.parts[i].path);
        if(!targets[i]) {found->failedModel=reinterpret_cast<std::uintptr_t>(model);ROCK_LOG_WARN(Weapon,"Loose item state path unavailable ref={:08X} path={}",ref->GetFormID(),found->state.parts[i].path);return false;}
    }
    if(found->model.get()!=model) {
        found->model.reset(model);found->originalState=found->state;
        for(std::uint32_t i=0;i<found->state.count;++i) {
            const auto* node=targets[i];auto& p=found->originalState.parts[i];
            std::memcpy(p.local.rotate,node->local.rotate.entry,sizeof(p.local.rotate));
            p.local.translate[0]=node->local.translate.x;p.local.translate[1]=node->local.translate.y;p.local.translate[2]=node->local.translate.z;
            p.local.scale=node->local.scale;p.hidden=node->GetAppCulled()?1u:0u;
        }
    }
    for(std::uint32_t i=0;i<found->state.count;++i) {
        auto* node=targets[i];const auto& p=found->state.parts[i];
        std::memcpy(node->local.rotate.entry,p.local.rotate,sizeof(p.local.rotate));
        node->local.translate={p.local.translate[0],p.local.translate[1],p.local.translate[2]};node->local.scale=p.local.scale;
        node->SetAppCulled(p.hidden!=0);
    }
    ROCK_LOG_INFO(Weapon,"Loose item state applied before model publication ref={:08X} parts={}",ref->GetFormID(),found->state.count);
    return true;
}
void restoreWithdrawn() {
    std::scoped_lock lock(mutex);
    for(auto& e:entries)if(e.retiring) {
        auto* ref=RE::TESForm::GetFormByID<RE::TESObjectREFR>(e.key.owner);
        if(ref&&ref->Get3D()==e.model.get()) {
            auto* root=weaponRoot(e.model.get());
            for(std::uint32_t i=0;root&&i<e.originalState.count;++i) {
                const auto& p=e.originalState.parts[i];auto* node=path(root,p.path);if(!node)continue;
                std::memcpy(node->local.rotate.entry,p.local.rotate,sizeof(p.local.rotate));
                node->local.translate={p.local.translate[0],p.local.translate[1],p.local.translate[2]};node->local.scale=p.local.scale;
                node->SetAppCulled(p.hidden!=0);
            }
        }
        e.model.reset();e.retiring=false;
    }
}
bool install() noexcept {
    if(original)return true;
    // TESObjectREFR vtable slot88, independently checked against the VR Set3D
    // implementation. No actor/player vtables or released provider slots change.
    const auto slot=REL::Offset(0x2cbcdf8+0x88*sizeof(void*)).address();
    Set3D current{};
    if(!native_memory::tryReadValue(reinterpret_cast<Set3D*>(slot),current)||reinterpret_cast<std::uintptr_t>(current)!=REL::Offset(0x3ece40).address())return false;
    original=current;
    try {REL::safe_write(slot,reinterpret_cast<std::uintptr_t>(&set3D));}
    catch(...) {original=nullptr;return false;}
    return true;
}
}
