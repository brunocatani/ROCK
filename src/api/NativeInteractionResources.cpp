#include "NativeInteractionState.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/native/HavokRuntime.h"
#include "physics-interaction/native/HavokRefCount.h"
#include "physics-interaction/native/HavokMaterialRegistry.h"
#include "physics-interaction/native/HavokWorldLock.h"
#include "physics-interaction/native/GeneratedKeyframedBodyDrive.h"
#include "physics-interaction/native/ReferenceInteraction.h"
#include "physics-interaction/native/NativePlayerCollisionFilter.h"
#include "physics-interaction/grab/GrabAuthorityProxy.h"
#include "physics-interaction/native/SceneRelation.h"
#include "physics-interaction/TransformMath.h"
#include "RE/Havok/hknpBody.h"
#include "RE/Havok/hknpConstraintCinfo.h"
#include <cstring>
#include <optional>

namespace rock::native_interaction_service::detail {
    using api::Status;
    void HavokReferenceDeleter::operator()(void* value) const noexcept {havok_ref_count::release(value);}
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
        struct HavokRef {
            void* value{};
            ~HavokRef(){havok_ref_count::release(value);}
        };
        RE::hknpWorld* world(){return current?reinterpret_cast<RE::hknpWorld*>(current->frame.world):nullptr;}
        RE::hkTransformf nativeTransform(const api::Transform& value,ni::PoseBasis basis) {
            const auto pose=fromApi(value);RE::hkTransformf result{};
            result.rotation=basis==ni::PoseBasis::BodyStoredAxes?
                transform_math::niStoredAxesToHknpBodyColumns(pose.rotate):niRotToHkTransformRotation(pose.rotate);
            result.translation={pose.translate.x*gameToHavokScale(),pose.translate.y*gameToHavokScale(),pose.translate.z*gameToHavokScale(),0};
            return result;
        }
        bool place(BodySlot& slot,const api::Transform& pose) {
            const bool moved=slot.body.setTransform(nativeTransform(pose,slot.poseBasis));
            alignas(16) const float zero[4]{};
            return slot.body.setVelocity(zero,zero)&&moved;
        }
        ni::Resource resource(std::size_t index,std::uint32_t serial){return (std::uint64_t(serial)<<32)|(index+1);}
        template<class Slots> auto* lookup(Slots& slots,Owner owner,ni::Resource handle) {
            const auto index=std::uint32_t(handle);
            if(!index||index>slots.size())return static_cast<typename Slots::value_type*>(nullptr);
            auto& slot=slots[index-1];
            return slot.owner==owner&&slot.serial==std::uint32_t(handle>>32)?&slot:nullptr;
        }
        bool ownedBodyLive(const BodySlot& slot) {
            if(slot.world!=world()||!slot.body.isValid())return false;
            const auto live=havok_runtime::snapshotBodyIdentity(world(),slot.body.getBodyId());
            return live.valid&&live.collisionObject==slot.body.getCollisionObject();
        }
        void release(BodySlot& slot,RE::hknpWorld* live,void* bhk) {
            if(slot.body.isValid()) {
                const auto body=live&&live==slot.world&&bhk==slot.bhk?
                    havok_runtime::snapshotBodyIdentity(live,slot.body.getBodyId()):havok_runtime::BodyIdentitySnapshot{};
                if(body.valid&&body.collisionObject==slot.body.getCollisionObject())slot.body.retireDeferred(bhk);
                else slot.body.reset();
            }
            slot.owner=0;slot.world=nullptr;slot.bhk=nullptr;
        }
        void release(SliderSlot& slot,RE::hknpWorld* live) {
            if(slot.id!=ni::kInvalidBody&&live&&live==slot.world)live->DestroyConstraints(&slot.id,1);
            // Retain our own reference until after world destruction/retirement.
            // Native removal cannot invalidate the borrowed limits memory.
            slot.data.reset();
            slot.owner=0;slot.id=ni::kInvalidBody;slot.world=nullptr;slot.anchor=0;
        }
        bool limits(void* data,float low,float high) {
            if(!data||!std::isfinite(low)||!std::isfinite(high)||low>high)return false;
            low*=gameToHavokScale();high*=gameToHavokScale();
            // FO4VR prismatic constructor 1419B142D / validator 1419B1ABA.
            std::memcpy(static_cast<char*>(data)+0x10C,&low,sizeof(low));
            std::memcpy(static_cast<char*>(data)+0x110,&high,sizeof(high));
            return true;
        }
    }
    // Creation already holds the read lock across identity validation and shape
    // retain. Do not reacquire a reader lock while a native writer is queued.
    static Status readBodyImpl(Owner owner,std::uint32_t id,ni::BodySnapshot* output,bool lockAlreadyHeld) noexcept {
        return guarded(owner,[&] {
            if(!output||output->size!=sizeof(*output))return Status::InvalidSize;
            *output={};
            if(auto s=access(owner);s!=Status::Ok)return s;
            std::optional<havok_world_lock::ScopedWorldReadLock> lock;
            if(!lockAlreadyHeld)lock.emplace(world());
            const auto live=havok_runtime::snapshotBodyIdentity(world(),RE::hknpBodyId{id});
            std::array<std::byte,sizeof(RE::hknpBody)> bytes{};
            if(!live.valid||!native_memory::guardedCopyFromMemory(live.body,bytes.data(),bytes.size()))return Status::TargetInvalid;
            const auto field=[&]<class T>(std::size_t offset,T& value){std::memcpy(&value,bytes.data()+offset,sizeof(value));};
            auto& identity=output->identity;
            field(offsetof(RE::hknpBody,bodyId),identity.body);
            field(offsetof(RE::hknpBody,motionIndex),identity.motion);
            field(offsetof(RE::hknpBody,userData),identity.collisionObject);
            field(offsetof(RE::hknpBody,shape),identity.shape);
            field(offsetof(RE::hknpBody,flags),output->flags);
            field(offsetof(RE::hknpBody,collisionFilterInfo),output->filter);
            field(offsetof(RE::hknpBody,deactivationIslandId),output->membership);
            identity.group=output->filter>>16;
            std::uint16_t material=0;
            field(offsetof(RE::hknpBody,materialId),material);output->material=material;
            std::uint16_t properties=0;field(offsetof(RE::hknpBody,motionPropertiesId),properties);output->motionProperties=properties;
            output->stage=ni::BodyReadStage::BodyIdentity;
            if(identity.body!=id||!identity.shape||identity.motion==ni::kInvalidBody||
                identity.collisionObject!=reinterpret_cast<std::uintptr_t>(live.collisionObject)||
                !native_memory::tryReadField(live.collisionObject,offsets::kCollisionObject_OwnerNode,identity.ownerNode)||!identity.ownerNode)
                return Status::TargetInvalid;
            RE::hknpWorld* ownerWorld=nullptr;RE::hknpBodyId ownerId{};
            output->stage=ni::BodyReadStage::OwnerWorld;
            if(!havok_runtime::tryResolveCollisionObjectBody(live.collisionObject,ownerWorld,ownerId)||ownerWorld!=world()||ownerId.value!=id)
                return Status::TargetInvalid;
            std::array<float,16> pose{};std::memcpy(pose.data(),bytes.data(),sizeof(pose));
            output->stage=ni::BodyReadStage::BodyPose;
            for(std::size_t i=0;i<16;++i)if((i&3)!=3&&!std::isfinite(pose[i]))return Status::TargetInvalid;
            RE::NiTransform transform{};
            transform.rotate=havokRotationBlocksToNiMatrix(pose.data());
            transform.translate={pose[12]*havokToGameScale(),pose[13]*havokToGameScale(),pose[14]*havokToGameScale()};
            transform.scale=1;
            output->world=toApi(transform);
            if(!finite(output->world))return Status::TargetInvalid;
            output->valid=1;
            output->stage=ni::BodyReadStage::Complete;
            return Status::Ok;
        });
    }
    Status readBody(Owner owner,std::uint32_t id,ni::BodySnapshot* output) noexcept {return readBodyImpl(owner,id,output,false);}
    bool matches(RE::hknpWorld* targetWorld,const ni::BodyIdentity& expected) {
        if(!current||reinterpret_cast<std::uintptr_t>(targetWorld)!=current->frame.world)return false;
        ni::BodySnapshot live{};
        return readBody(current->owner,expected.body,&live)==Status::Ok&&live.identity==expected;
    }
    Status resolveCollisionBody(Owner owner,std::uintptr_t collision,std::uint32_t* body) noexcept {
        return guarded(owner,[&] {
            if(!body)return Status::InvalidArgument;*body=ni::kInvalidBody;
            if(auto s=access(owner);s!=Status::Ok)return s;
            havok_world_lock::ScopedWorldReadLock lock(world());
            RE::hknpWorld* nativeWorld=nullptr;RE::hknpBodyId id{};
            if(!havok_runtime::tryResolveCollisionObjectBody(reinterpret_cast<RE::NiCollisionObject*>(collision),nativeWorld,id)||nativeWorld!=world())return Status::TargetInvalid;
            *body=id.value;return Status::Ok;
        });
    }
    Status resolveBodyReference(Owner owner,std::uint32_t body,std::uintptr_t* reference) noexcept {
        return guarded(owner,[&] {
            if(!reference)return Status::InvalidArgument;*reference=0;
            if(auto s=access(owner);s!=Status::Ok)return s;
            *reference=reinterpret_cast<std::uintptr_t>(reference_interaction::resolveBody(world(),body));
            return *reference?Status::Ok:Status::TargetUnavailable;
        });
    }
    Status queryNodeRelation(Owner owner,std::uintptr_t root,std::uintptr_t target,ni::NodeRelation relation,std::uint32_t depth,std::uint32_t* result) noexcept {
        return guarded(owner,[&] {
            if(!result||depth>64||static_cast<unsigned>(relation)>1)return Status::InvalidArgument;*result=0;
            if(auto s=access(owner);s!=Status::Ok)return s;
            if(!native_memory::pointerRangeLooksReadable(reinterpret_cast<void*>(root),sizeof(RE::NiAVObject))||
                !native_memory::pointerRangeLooksReadable(reinterpret_cast<void*>(target),sizeof(RE::NiAVObject)))return Status::TargetInvalid;
            auto* a=reinterpret_cast<RE::NiAVObject*>(root);auto* b=reinterpret_cast<RE::NiAVObject*>(target);
            *result=relation==ni::NodeRelation::Descendant?scene_relation::nodeContainsNode(a,b,static_cast<int>(depth)):scene_relation::isSkinnedToNode(a,b);
            return Status::Ok;
        });
    }
    Status createBody(Owner owner,const ni::BodyCreate* request,ni::Resource* handle,std::uint32_t* bodyId) noexcept {
        return guarded(owner,[&] {
            if(!request||request->size!=sizeof(*request)||!handle||!bodyId)return Status::InvalidArgument;
            *handle=0;*bodyId=ni::kInvalidBody;
            if(auto s=access(owner,true,true);s!=Status::Ok)return s;
            if(!finite(request->pose)||request->name[47]||request->privateAnchor>1||
                static_cast<unsigned>(request->poseBasis)>1)return Status::InvalidArgument;
            if(request->privateAnchor&&(state().driver!=owner||current->frame.phase!=ni::Phase::WeaponUpdate))return Status::PermissionDenied;
            if(!request->privateAnchor&&(request->filter&collision_layer_policy::FO4_LAYER_FILTER_MASK)!=ni::kPassiveObstacleLayer)return Status::PermissionDenied;
            auto& slots=state().bodies;
            std::size_t index=0;for(;index<slots.size()&&slots[index].owner;++index){}
            if(index==slots.size())return Status::CapacityFull;
            HavokRef shape{};RE::hknpMaterialId material{};
            if(request->privateAnchor) {
                shape.value=grab_authority_proxy::buildProxyShape();
                material=havok_material_registry::registerGeneratedBodyMaterial(world());
            } else {
                havok_world_lock::ScopedWorldReadLock lock(world());
                ni::BodySnapshot source{};
                if(readBodyImpl(owner,request->source.body,&source,true)!=Status::Ok||source.identity!=request->source)return Status::GenerationMismatch;
                std::uint32_t refs=0;
                auto* nativeShape=reinterpret_cast<RE::hknpShape*>(source.identity.shape);
                if(!native_memory::tryReadField(nativeShape,8,refs)||!(refs>>16)||!(refs&0xFFFF))return Status::TargetInvalid;
                havok_ref_count::addRef(nativeShape);shape.value=nativeShape;
                material=RE::hknpMaterialId{static_cast<std::uint16_t>(source.material)};
            }
            if(!shape.value||material.value==0xFFFF)return Status::TargetUnavailable;
            auto& slot=slots[index];
            slot.owner=owner;slot.serial=state().nextSerial++;
            slot.world=world();slot.bhk=reinterpret_cast<void*>(current->frame.bhkWorld);
            slot.poseBasis=request->poseBasis;
            slot.privateAnchor=request->privateAnchor!=0;
            std::memcpy(slot.name.data(),request->name,slot.name.size());
            const auto filter=request->privateAnchor?grab_authority_proxy::noContactFilterInfo():request->filter;
            if(!slot.body.create(world(),slot.bhk,static_cast<RE::hknpShape*>(shape.value),filter,material,BethesdaMotionType::Keyframed,slot.name.data())||
                !place(slot,request->pose)) {
                release(slot,world(),slot.bhk);return Status::TargetUnavailable;
            }
            *handle=resource(index,slot.serial);*bodyId=slot.body.getBodyId().value;return Status::Ok;
        });
    }
    Status retireBody(Owner owner,ni::Resource handle) noexcept {
        return guarded(owner,[&] {
            if(auto s=access(owner,true,true);s!=Status::Ok)return s;
            auto* slot=lookup(state().bodies,owner,handle);if(!slot)return Status::TargetInvalid;
            for(const auto& slider:state().sliders)if(slider.owner==owner&&slider.anchor==handle)return Status::Busy;
            release(*slot,world(),reinterpret_cast<void*>(current->frame.bhkWorld));return Status::Ok;
        });
    }
    Status placeBody(Owner owner,ni::Resource handle,const api::Transform* target) noexcept {
        return guarded(owner,[&] {
            if(!target||!finite(*target))return Status::InvalidArgument;
            if(auto s=access(owner,false,true);s!=Status::Ok)return s;
            auto* slot=lookup(state().bodies,owner,handle);if(!slot||!ownedBodyLive(*slot))return Status::TargetInvalid;
            return place(*slot,*target)?Status::Ok:Status::TargetUnavailable;
        });
    }
    Status driveBody(Owner owner,ni::Resource handle,const api::Transform* target,float delta) noexcept {
        return guarded(owner,[&] {
            if(!target||!finite(*target)||!havok_physics_timing::isUsableDelta(delta))return Status::InvalidArgument;
            if(auto s=access(owner,false,true);s!=Status::Ok)return s;
            auto* slot=lookup(state().bodies,owner,handle);if(!slot||!ownedBodyLive(*slot))return Status::TargetInvalid;
            return slot->body.driveToKeyFrame(nativeTransform(*target,slot->poseBasis),delta)?Status::Ok:Status::TargetUnavailable;
        });
    }
    Status setBodyFilter(Owner owner,ni::Resource handle,std::uint32_t filter) noexcept {
        return guarded(owner,[&] {
            if(auto s=access(owner,false,true);s!=Status::Ok)return s;
            if((filter&collision_layer_policy::FO4_LAYER_FILTER_MASK)!=ni::kPassiveObstacleLayer&&!grab_authority_proxy::hasNoContactFilterInfo(filter))return Status::PermissionDenied;
            auto* slot=lookup(state().bodies,owner,handle);if(!slot||!ownedBodyLive(*slot))return Status::TargetInvalid;
            if(slot->privateAnchor&&!grab_authority_proxy::hasNoContactFilterInfo(filter))return Status::PermissionDenied;
            slot->body.setCollisionFilterInfo(filter);return Status::Ok;
        });
    }
    Status activateWeapon(Owner owner) noexcept {
        return guarded(owner,[&] {
            if(auto s=access(owner,false,true);s!=Status::Ok)return s;
            if(state().driver!=owner||state().weaponBody==ni::kInvalidBody)return Status::OwnerConflict;
            havok_runtime::activateBody(world(),state().weaponBody);return Status::Ok;
        });
    }
    Status createSlider(Owner owner,const ni::SliderCreate* request,ni::Resource* handle) noexcept {
        return guarded(owner,[&] {
            if(!request||request->size!=sizeof(*request)||!handle)return Status::InvalidArgument;*handle=0;
            if(auto s=access(owner,true,true);s!=Status::Ok)return s;
            if(state().driver!=owner||request->bodyA!=state().weaponBody||!finite(request->bodyWorld)||!finite(request->anchorWorld))return Status::PermissionDenied;
            auto* anchor=lookup(state().bodies,owner,request->anchor);
            if(!anchor||!anchor->privateAnchor||!ownedBodyLive(*anchor))return Status::TargetInvalid;
            auto& slots=state().sliders;std::size_t index=0;
            for(;index<slots.size()&&slots[index].owner;++index){}if(index==slots.size())return Status::CapacityFull;
            // These exact native constructors/frames were used by the migrated
            // slider. The world owns constraint data after CreateConstraint.
            using Ctor=void*(*)(void*);
            using Frames=void(*)(void*,const RE::hkTransformf&,const RE::hkTransformf&,const RE::hkVector4f&,const RE::hkVector4f&);
            const auto constructor=REL::Offset(offsets::kFunc_PrismaticConstraintData_Ctor).address();
            const auto setFrames=REL::Offset(offsets::kFunc_PrismaticConstraintData_SetInWorldSpace).address();
            const auto text=REL::Module::get().segment(REL::Segment::text);
            const auto entryMatches=[&]<std::size_t N>(std::uintptr_t address,const std::array<std::uint8_t,N>& expected) {
                std::array<std::uint8_t,N> actual{};
                return address>=text.address()&&address+N<=text.address()+text.size()&&
                    native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(address),actual.data(),N)&&actual==expected;
            };
            if(!entryMatches(constructor,std::array<std::uint8_t,7>{0xC7,0x41,0x08,0x01,0x00,0xFF,0xFF})||
                !entryMatches(setFrames,std::array<std::uint8_t,8>{0x4C,0x8B,0xDC,0x49,0x89,0x5B,0x08,0x49}))return Status::TargetUnavailable;
            HavokRef data{};
            data.value=havok_runtime::allocateHavok(0x120);
            if(!data.value)return Status::CapacityFull;
            REL::Relocation<Ctor>{REL::Offset(offsets::kFunc_PrismaticConstraintData_Ctor)}(data.value);
            const auto& p=request->pivotWorld;const auto& a=request->axisWorld;
            const float norm=a.x*a.x+a.y*a.y+a.z*a.z;
            if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z)||!std::isfinite(norm)||std::abs(norm-1)>0.001)return Status::InvalidArgument;
            const float scale=gameToHavokScale();
            REL::Relocation<Frames>{REL::Offset(offsets::kFunc_PrismaticConstraintData_SetInWorldSpace)}(data.value,
                nativeTransform(request->bodyWorld,ni::PoseBasis::BodyStoredAxes),nativeTransform(request->anchorWorld,ni::PoseBasis::BodyStoredAxes),
                RE::hkVector4f{p.x*scale,p.y*scale,p.z*scale,0},RE::hkVector4f{a.x,a.y,a.z,0});
            if(!limits(data.value,request->minimumGame,request->maximumGame))return Status::InvalidArgument;
            RE::hknpConstraintCinfo cinfo{};cinfo.constraintData=reinterpret_cast<RE::hkpConstraintData*>(data.value);
            cinfo.bodyIdA=request->bodyA;cinfo.bodyIdB=anchor->body.getBodyId().value;
            auto& slot=slots[index];slot.id=ni::kInvalidBody;
            world()->CreateConstraint(&slot.id,cinfo);
            if(slot.id==ni::kInvalidBody)return Status::TargetUnavailable;
            slot.owner=owner;slot.serial=state().nextSerial++;slot.world=world();slot.data.reset(data.value);
            slot.anchor=request->anchor;data.value=nullptr;
            *handle=resource(index,slot.serial);return Status::Ok;
        });
    }
    Status setSliderLimits(Owner owner,ni::Resource handle,float low,float high) noexcept {
        return guarded(owner,[&] {
            if(auto s=access(owner,true,true);s!=Status::Ok)return s;
            auto* slot=lookup(state().sliders,owner,handle);if(!slot||slot->world!=world())return Status::TargetInvalid;
            return limits(slot->data.get(),low,high)?Status::Ok:Status::InvalidArgument;
        });
    }
    Status destroySlider(Owner owner,ni::Resource handle) noexcept {
        return guarded(owner,[&] {
            if(auto s=access(owner,true,true);s!=Status::Ok)return s;
            auto* slot=lookup(state().sliders,owner,handle);if(!slot)return Status::TargetInvalid;
            release(*slot,world());return Status::Ok;
        });
    }
    Status setWeaponContactExclusion(Owner owner,const ni::BodyIdentity* target) noexcept {
        return guarded(owner,[&] {
            if(auto s=access(owner,true,true);s!=Status::Ok)return s;
            if(!target||state().driver!=owner||!matches(world(),*target))return Status::TargetInvalid;
            if(!native_player_collision::publishToolContactExclusion(world(),state().weaponBody,target->body))return Status::TargetUnavailable;
            state().excludedTarget=target->body;return Status::Ok;
        });
    }
    Status clearWeaponContactExclusion(Owner owner) noexcept {
        return guarded(owner,[&] {
            if(auto s=access(owner,true,true);s!=Status::Ok)return s;
            if(state().driver!=owner)return Status::OwnerConflict;
            native_player_collision::clearToolContactExclusion(world());state().excludedTarget=ni::kInvalidBody;return Status::Ok;
        });
    }
    Status hasWeaponContactExclusion(Owner owner,const ni::BodyIdentity* target,std::uint32_t* result) noexcept {
        return guarded(owner,[&] {
            if(!result)return Status::InvalidArgument;*result=0;
            if(auto s=access(owner);s!=Status::Ok)return s;
            if(!target||state().driver!=owner||!matches(world(),*target))return Status::TargetInvalid;
            *result=native_player_collision::hasToolContactExclusion(world(),state().weaponBody,target->body);return Status::Ok;
        });
    }
    void releaseResources(Owner owner,RE::hknpWorld* live,void* bhk) {
        for(auto& slot:state().sliders)if(slot.owner==owner)release(slot,live);
        for(auto& slot:state().bodies)if(slot.owner==owner)release(slot,live,bhk);
        if(state().driver==owner){native_player_collision::clearToolContactExclusion(live);state().excludedTarget=ni::kInvalidBody;}
    }
    void releaseWeaponResources(Owner owner,RE::hknpWorld* live,void* bhk) {
        for(auto& slot:state().sliders)if(slot.owner==owner)release(slot,live);
        for(auto& slot:state().bodies)if(slot.owner==owner&&slot.privateAnchor)release(slot,live,bhk);
        if(state().driver==owner){native_player_collision::clearToolContactExclusion(live);state().excludedTarget=ni::kInvalidBody;}
    }
}
