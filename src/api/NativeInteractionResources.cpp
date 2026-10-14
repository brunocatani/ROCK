#include "NativeInteractionState.h"
#include "providers/Boundary.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/native/HavokOffsets.h"
#include "physics-interaction/native/HavokRuntime.h"
#include "physics-interaction/native/HavokWorldLock.h"
#include "physics-interaction/native/NativePlayerCollisionFilter.h"
#include "physics-interaction/TransformMath.h"
#include "RE/Havok/hknpBody.h"
#include <cstring>

namespace rock::native_interaction_service::detail {
    using api::Status;
    RE::NiTransform fromApi(const api::Transform& value) {
        return ni::unpackTransform<RE::NiTransform>(value);
    }
    api::Transform toApi(const RE::NiTransform& value) {
        return ni::packTransform(value);
    }
    RE::NiPoint3 fromApi(const api::Point3& p) {return {p.x,p.y,p.z};}
    api::Point3 toApi(const RE::NiPoint3& p) {return {p.x,p.y,p.z};}
    bool finite(const api::Transform& value) {
        for(float x:value.rotate) if(!std::isfinite(x))return false;
        for(float x:value.translate) if(!std::isfinite(x))return false;
        return std::isfinite(value.scale)&&value.scale>0 &&
            transform_math::storedRotationOrthonormalityError(fromApi(value).rotate)<=0.001;
    }
    namespace {
        RE::hknpWorld* world(){return current?reinterpret_cast<RE::hknpWorld*>(current->frame.world):nullptr;}
        // Reads the live identity of a body in the callback's world: the body
        // slot, its collision object, owner node and owning world must agree.
        bool liveIdentity(std::uint32_t id,ni::BodyIdentity& identity) {
            havok_world_lock::ScopedWorldReadLock lock(world());
            const auto live=havok_runtime::snapshotBodyIdentity(world(),RE::hknpBodyId{id});
            std::array<std::byte,sizeof(RE::hknpBody)> bytes{};
            if(!live.valid||!native_memory::guardedCopyFromMemory(live.body,bytes.data(),bytes.size()))return false;
            const auto field=[&]<class T>(std::size_t offset,T& value){std::memcpy(&value,bytes.data()+offset,sizeof(value));};
            std::uint32_t filter=0;
            field(offsetof(RE::hknpBody,bodyId),identity.body);
            field(offsetof(RE::hknpBody,motionIndex),identity.motion);
            field(offsetof(RE::hknpBody,userData),identity.collisionObject);
            field(offsetof(RE::hknpBody,shape),identity.shape);
            field(offsetof(RE::hknpBody,collisionFilterInfo),filter);
            identity.group=filter>>16;
            if(identity.body!=id||!identity.shape||identity.motion==ni::kInvalidBody||
                identity.collisionObject!=reinterpret_cast<std::uintptr_t>(live.collisionObject)||
                !native_memory::tryReadField(live.collisionObject,offsets::kCollisionObject_OwnerNode,identity.ownerNode)||!identity.ownerNode)
                return false;
            RE::hknpWorld* ownerWorld=nullptr;RE::hknpBodyId ownerId{};
            return havok_runtime::tryResolveCollisionObjectBody(live.collisionObject,ownerWorld,ownerId)&&ownerWorld==world()&&ownerId.value==id;
        }
        bool matches(const ni::BodyIdentity& expected) {
            ni::BodyIdentity live{};
            return world()&&liveIdentity(expected.body,live)&&live==expected;
        }
        Status weaponSlot(Owner owner,std::uint32_t slot) {
            if(auto s=api::boundary::checkWeaponSlot(slot);s!=Status::Ok)return s;
            if(state().driver!=owner)return Status::OwnerConflict;
            return api::boundary::requireWeaponSlot(slot);
        }
    }
    Status activateWeapon(Owner owner,std::uint32_t slot) noexcept {
        return guarded(owner,[&] {
            if(auto s=access(owner,false,true);s!=Status::Ok)return s;
            if(auto s=weaponSlot(owner,slot);s!=Status::Ok)return s;
            if(state().weaponBody==ni::kInvalidBody)return Status::TargetUnavailable;
            havok_runtime::activateBody(world(),state().weaponBody);return Status::Ok;
        });
    }
    Status setWeaponContactExclusion(Owner owner,std::uint32_t slot,const ni::BodyIdentity* target) noexcept {
        return guarded(owner,[&] {
            if(auto s=access(owner,true,true);s!=Status::Ok)return s;
            if(auto s=weaponSlot(owner,slot);s!=Status::Ok)return s;
            if(!target||!matches(*target))return Status::TargetInvalid;
            if(!native_player_collision::publishToolContactExclusion(world(),state().weaponBody,target->body))return Status::TargetUnavailable;
            state().excludedTarget=target->body;return Status::Ok;
        });
    }
    Status clearWeaponContactExclusion(Owner owner,std::uint32_t slot) noexcept {
        return guarded(owner,[&] {
            if(auto s=access(owner,true,true);s!=Status::Ok)return s;
            if(auto s=weaponSlot(owner,slot);s!=Status::Ok)return s==Status::TargetUnavailable?Status::Ok:s;
            native_player_collision::clearToolContactExclusion(world());state().excludedTarget=ni::kInvalidBody;return Status::Ok;
        });
    }
    Status hasWeaponContactExclusion(Owner owner,std::uint32_t slot,const ni::BodyIdentity* target,std::uint32_t* result) noexcept {
        return guarded(owner,[&] {
            if(!result)return Status::InvalidArgument;*result=0;
            if(auto s=access(owner);s!=Status::Ok)return s;
            if(auto s=weaponSlot(owner,slot);s!=Status::Ok)return s;
            if(!target||!matches(*target))return Status::TargetInvalid;
            *result=native_player_collision::hasToolContactExclusion(world(),state().weaponBody,target->body);return Status::Ok;
        });
    }
    void releaseWeaponResources(Owner owner,RE::hknpWorld* live) {
        if(state().driver==owner){native_player_collision::clearToolContactExclusion(live);state().excludedTarget=ni::kInvalidBody;}
    }
}
