#include "NativeInteractionState.h"
#include "RockConfig.h"
#include "ProviderRuntimeServices.h"
#include "physics-interaction/core/PhysicsFrameContext.h"
#include "physics-interaction/weapon/WeaponCollision.h"
#include "physics-interaction/native/HavokRuntime.h"
#include "physics-interaction/native/PhysicsUtils.h"
#include "physics-interaction/native/NativePlayerCollisionFilter.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/PhysicsLog.h"
#include <algorithm>
#include <cstring>
#include <limits>

namespace rock::native_interaction_service {
    using api::Status;
    namespace ni=api::native_interaction;
    namespace detail {
        State& state(){static State value;return value;}
        thread_local const CallContext* current=nullptr;
        Extension* extension(Owner owner){for(auto& slot:state().extensions)if(owner&&slot.owner==owner)return &slot;return nullptr;}
        Status access(Owner owner,bool structural,bool write) {
            if(!current||current->owner!=owner||!extension(owner))return Status::WrongThread;
            const auto phase=current->frame.phase;
            const bool retire=phase==ni::Phase::Retire||phase==ni::Phase::WeaponRetire;
            const auto index=static_cast<unsigned>(extension(owner)-state().extensions.data());
            if(!retire&&(state().faultedExtensions.load(std::memory_order_acquire)&(1u<<index)))return Status::OwnerRevoked;
            const bool mutation=retire||phase==ni::Phase::Update||phase==ni::Phase::WeaponUpdate||phase==ni::Phase::Extraction;
            if((structural&&!mutation)||(write&&!mutation&&phase!=ni::Phase::PreCollide))return Status::WrongThread;
            if(!current->frame.world&&!retire)return Status::WorldNotReady;
            return Status::Ok;
        }
    }
    using namespace detail;
    namespace {
        thread_local const ni::v1_1::ContactDetails* contactDetails=nullptr;
        ni::Frame makeFrame(const PhysicsFrameContext& context,ni::Phase phase) {
            ni::Frame result{};result.phase=phase;result.sample=provider::runtime::sample();
            result.sample.frameIndex=provider::runtime::currentGameFrameIndex();
            result.world=reinterpret_cast<std::uintptr_t>(context.hknpWorld);
            result.bhkWorld=reinterpret_cast<std::uintptr_t>(context.bhkWorld);
            result.hmdPosition=toApi(context.hmdPositionWorld);result.deltaSeconds=context.deltaSeconds;
            result.gameToHavok=gameToHavokScale();result.havokToGame=havokToGameScale();
            result.worldReady=context.worldReady;result.menuBlocked=context.menuBlocked;result.hmdValid=context.hasHmdFrame;
            return result;
        }
        ni::Frame stepFrame(RE::hknpWorld* world,const havok_physics_timing::PhysicsTimingSample& timing) {
            auto frame=state().frame;frame.phase=ni::Phase::PreCollide;frame.world=reinterpret_cast<std::uintptr_t>(world);
            frame.substepSeconds=timing.substepDeltaSeconds;frame.timingValid=timing.valid;frame.timingFallback=timing.usedFallback;
            return frame;
        }
        // Keep SEH outside the RAII scope, so quiescence and callback context
        // unwind normally when an extension faults in native code.
        bool invokeProtected(void(*operation)(void*),void* context) {
#if defined(_MSC_VER)
            __try {operation(context);return true;}
            __except(EXCEPTION_EXECUTE_HANDLER){return false;}
#else
            operation(context);return true;
#endif
        }
        bool disabled(const Extension& slot) {
            const auto& service=state();
            if(service.faultedExtensions.load(std::memory_order_acquire)&(1u<<static_cast<unsigned>(&slot-service.extensions.data())))return true;
            if(service.removeAll.load(std::memory_order_acquire))return true;
            for(const auto& pending:service.pendingRemoval)if(pending.load(std::memory_order_acquire)==slot.owner)return true;
            return false;
        }
        template<class F> bool invoke(Extension& slot,const ni::Frame& frame,F&& operation) {
            const bool cleanup=frame.phase==ni::Phase::Retire||frame.phase==ni::Phase::WeaponRetire;
            if(!cleanup&&disabled(slot))return false;
            ContextScope scope(slot.owner,frame);
            const auto status=guarded(slot.owner,[&]{
                if(invokeProtected([](void* value){(*static_cast<std::remove_reference_t<F>*>(value))();},&operation))return Status::Ok;
                state().faultedExtensions.fetch_or(1u<<static_cast<unsigned>(&slot-state().extensions.data()),std::memory_order_release);
                return Status::InternalError;
            });
            return status==Status::Ok;
        }
        void combineMasks(RE::hknpWorld* world=nullptr) {
            std::uint64_t mask=0;for(const auto& slot:state().extensions)if(slot.owner)mask|=slot.obstacleLayers;
            const auto previous=state().obstacleLayers.exchange(mask,std::memory_order_acq_rel);
            if(previous!=mask&&world&&state().refreshLayers)state().refreshLayers(state().retireContext,world);
        }
        bool hasWeaponConstraintState() {
            const auto& service=state();
            if(service.excludedTarget!=ni::kInvalidBody)return true;
            for(const auto& slider:service.sliders)if(service.driver&&slider.owner==service.driver)return true;
            return false;
        }
        Status readContactDetails(Owner owner,ni::v1_1::ContactDetails* output) noexcept {
            return guarded(owner,[&] {
                if(!output||output->size!=sizeof(*output))return Status::InvalidSize;
                *output={};
                if(auto status=access(owner);status!=Status::Ok)return status;
                if(current->frame.phase!=ni::Phase::WeaponContact||!contactDetails)return Status::WrongThread;
                *output=*contactDetails;return output->valid?Status::Ok:Status::NotReady;
            });
        }
        Status registerExtension(Owner owner,const ni::Callbacks* callbacks) noexcept {
            return guarded(owner,[&] {
                if(!callbacks||callbacks->size!=sizeof(*callbacks)||callbacks->version!=1)return Status::InvalidSize;
                if(current)return Status::Busy;
                if(callbacks->weaponUpdate&&(!callbacks->weaponRetire||!callbacks->weaponPreCollide))return Status::InvalidArgument;
                if(callbacks->extractSelection&&(!callbacks->classifySelection||!callbacks->validateSelection))return Status::InvalidArgument;
                if(auto s=provider::runtime::authorize(owner,ni::kInterfaceId,3,true);s!=Status::Ok)return s;
                auto& service=state();
                if(!service.rockGate)return Status::NotReady;
                auto rock=service.rockGate->pauseForMutation();auto mutation=service.gate.pauseForMutation();
                if(extension(owner))return Status::OwnerConflict;
                if(callbacks->weaponUpdate&&service.driver)return Status::OwnerConflict;
                for(auto& slot:service.extensions)if(!slot.owner) {
                    slot.owner=owner;slot.callbacks=*callbacks;slot.obstacleLayers=0;slot.bodyCount=0;slot.visualFrame=0;
                    service.registeredOwners[&slot-service.extensions.data()].store(owner,std::memory_order_release);
                    service.faultedExtensions.fetch_and(~(1u<<static_cast<unsigned>(&slot-service.extensions.data())),std::memory_order_release);
                    if(callbacks->weaponUpdate)service.driver=owner;
                    ROCK_LOG_INFO(Physics,"Native interaction extension registered: owner={} weaponDriver={} selection={}",owner,
                        callbacks->weaponUpdate!=nullptr,callbacks->classifySelection!=nullptr);
                    return Status::Ok;
                }
                return Status::CapacityFull;
            });
        }
        Status unregisterExtension(Owner owner) noexcept {
            return guarded(owner,[&] {
                if(current)return Status::Busy;
                if(auto s=provider::runtime::authorize(owner,ni::kInterfaceId,0,true);s!=Status::Ok)return s;
                clearOwner(owner);return Status::Ok;
            });
        }
        Status setDynamicObstacleLayers(Owner owner,std::uint64_t mask) noexcept {
            return guarded(owner,[&] {
                if(auto s=access(owner,true,true);s!=Status::Ok)return s;
                // Only optional passive-object pairs are delegated. Native
                // actor/controller and generated self-collision policy stays core.
                constexpr auto allowed=(std::uint64_t{1}<<33)|(std::uint64_t{1}<<ni::kPassiveObstacleLayer);
                if(mask&~allowed)return Status::InvalidArgument;
                extension(owner)->obstacleLayers=mask;combineMasks(reinterpret_cast<RE::hknpWorld*>(current->frame.world));return Status::Ok;
            });
        }
        Status copyWeaponSources(Owner owner,ni::WeaponSource* output,std::uint32_t capacity,std::uint32_t* copied) noexcept {
            return guarded(owner,[&] {
                if(!copied||(capacity&&!output)||capacity>ni::kMaxWeaponSources)return Status::InvalidArgument;*copied=0;
                if(auto s=access(owner);s!=Status::Ok)return s;
                const auto& service=state();
                if(service.driver!=owner||!service.collision||current->frame.phase!=ni::Phase::WeaponUpdate)return Status::WrongThread;
                const auto sources=service.collision->getProfileEvidenceDescriptors();
                if(sources.size()>capacity)return Status::CapacityFull;
                for(const auto& source:sources)if(source.valid&&source.weaponGenerationKey==service.generation) {
                    auto& value=output[(*copied)++];value={};value.body=source.bodyId;value.generation=source.weaponGenerationKey;
                    if(source.sourceName.size()>=sizeof(value.name)){*copied=0;return Status::TargetInvalid;}
                    std::snprintf(value.name,sizeof(value.name),"%s",source.sourceName.c_str());
                }
                return Status::Ok;
            });
        }
        Status getSourceGeometry(Owner owner,std::uint32_t body,const api::Point3* direction,ni::SourceGeometry* output) noexcept {
            return guarded(owner,[&] {
                if(!output||output->size!=sizeof(*output))return Status::InvalidSize;*output={};
                if(auto s=access(owner);s!=Status::Ok)return s;
                const auto& service=state();
                if(service.driver!=owner||!service.collision||!service.weaponNode||current->frame.phase!=ni::Phase::WeaponUpdate)return Status::WrongThread;
                if(!finite(toApi(service.weaponNode->world)))return Status::TargetInvalid;
                WeaponCollision::SupportGripEvidenceView view{};
                if(!service.collision->tryGetSupportGripEvidenceView(body,service.weaponNode,view)||!view.sourceNodeCurrent||
                    view.weaponGenerationKey!=service.generation)return Status::TargetInvalid;
                output->generation=service.generation;
                output->sourceInWeapon=toApi(transform_math::composeTransforms(transform_math::invertTransform(service.weaponNode->world),view.localToWorld));
                if(!finite(output->sourceInWeapon))return Status::TargetInvalid;
                if(direction) {
                    if(!std::isfinite(direction->x)||!std::isfinite(direction->y)||!std::isfinite(direction->z)||view.localTriangles.empty()||view.localTriangles.size()>10000)return Status::TargetInvalid;
                    constexpr float largest=(std::numeric_limits<float>::max)();
                    api::Point3 low{largest,largest,largest},high{-largest,-largest,-largest};float support=-largest;
                    for(const auto& triangle:view.localTriangles)for(const auto& p:{triangle.v0,triangle.v1,triangle.v2}) {
                        if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))return Status::TargetInvalid;
                        low.x=(std::min)(low.x,p.x);low.y=(std::min)(low.y,p.y);low.z=(std::min)(low.z,p.z);
                        high.x=(std::max)(high.x,p.x);high.y=(std::max)(high.y,p.y);high.z=(std::max)(high.z,p.z);
                        const float projection=p.x*direction->x+p.y*direction->y+p.z*direction->z;
                        if(projection>support){support=projection;output->supportPointLocal=toApi(p);}
                    }
                    output->boundsLocal={low,high,1,0};output->triangleCount=static_cast<std::uint32_t>(view.localTriangles.size());
                }
                output->valid=1;return Status::Ok;
            });
        }
        Status publishVisuals(Owner owner,const ni::BodyVisual* bodies,std::uint32_t count) noexcept {
            return guarded(owner,[&] {
                if(auto s=access(owner);s!=Status::Ok)return s;
                if(current->frame.phase!=ni::Phase::Render)return Status::WrongThread;
                if(count>ni::kMaxVisualBodies||(count&&!bodies))return Status::CapacityFull;
                auto* slot=extension(owner);
                std::uint32_t combined=count;
                for(const auto& other:state().extensions)if(other.owner&&other.owner!=owner&&other.visualFrame==current->frame.sample.frameIndex)combined+=other.bodyCount;
                if(combined>ni::kMaxVisualBodies)return Status::CapacityFull;
                const auto colorValid=[](const float* color){for(int i=0;i<4;++i)if(!std::isfinite(color[i])||color[i]<0||color[i]>1)return false;return true;};
                for(std::uint32_t i=0;i<count;++i)if(bodies[i].size!=sizeof(*bodies)||bodies[i].label[63]||!colorValid(bodies[i].color)||
                    !std::isfinite(bodies[i].labelOffsetY)||(bodies[i].flags&~3u)||bodies[i].identity.body==ni::kInvalidBody||
                    !bodies[i].identity.collisionObject||!bodies[i].identity.shape)return Status::InvalidArgument;
                if(count)std::copy_n(bodies,count,slot->bodies.begin());
                slot->bodyCount=count;slot->visualFrame=current->frame.sample.frameIndex;
                return Status::Ok;
            });
        }
    }
    void initialize(PhysicsCallbackQuiescenceGate* gate,RetireWeapon retire,RefreshLayers refresh,void* context) {
        state().rockGate=gate;state().retireWeapon=retire;state().refreshLayers=refresh;state().retireContext=context;state().gate.resumeCallbacks();
    }
    void detach(PhysicsCallbackQuiescenceGate* gate) {
        auto& service=state();if(service.rockGate!=gate)return;
        service.gate.pauseAndWait();service.rockGate=nullptr;service.retireWeapon=nullptr;service.refreshLayers=nullptr;service.retireContext=nullptr;
        service.frame={};service.command={};service.weaponBody=ni::kInvalidBody;
    }
    void update(const PhysicsFrameContext& context) {
        auto& service=state();if(!service.rockGate)return;
        auto rock=service.rockGate->pauseForMutation();auto mutation=service.gate.pauseForMutation();
        service.frame=makeFrame(context,ni::Phase::Update);
        const bool removeAll=service.removeAll.exchange(false,std::memory_order_acq_rel);
        for(auto& pending:service.pendingRemoval)if(const auto owner=pending.exchange(0,std::memory_order_acq_rel))clearOwner(owner);
        if(removeAll)for(auto& slot:service.extensions)if(slot.owner)clearOwner(slot.owner);
        const auto faults=service.faultedExtensions.exchange(0,std::memory_order_acq_rel);
        for(std::size_t i=0;i<service.extensions.size();++i) {
            auto& slot=service.extensions[i];if(!slot.owner)continue;
            if(faults&(1u<<i)){ROCK_LOG_ERROR(Physics,"Native interaction extension faulted: owner={}",slot.owner);clearOwner(slot.owner);continue;}
            if(slot.callbacks.update)invoke(slot,service.frame,[&]{slot.callbacks.update(&service.frame,slot.callbacks.context);});
        }
    }
    void preCollide(RE::hknpWorld* world,const havok_physics_timing::PhysicsTimingSample& timing) {
        if(timing.phase!=havok_physics_timing::PhysicsStepPhase::SubstepPreCollide)return;
        auto lease=state().gate.tryEnterCallback();if(!lease||reinterpret_cast<std::uintptr_t>(world)!=state().frame.world)return;
        const auto frame=stepFrame(world,timing);
        for(auto& slot:state().extensions)if(slot.owner&&slot.callbacks.preCollide)invoke(slot,frame,[&]{slot.callbacks.preCollide(&frame,slot.callbacks.context);});
    }
    void render(const PhysicsFrameContext& context) {
        auto& service=state();if(!service.rockGate)return;
        auto rock=service.rockGate->pauseForMutation();auto mutation=service.gate.pauseForMutation();
        const auto frame=makeFrame(context,ni::Phase::Render);
        for(auto& slot:service.extensions)if(slot.owner) {
            slot.bodyCount=0;
            if(slot.callbacks.render)invoke(slot,frame,[&]{slot.callbacks.render(&frame,slot.callbacks.context);});
        }
    }
    bool hasVisuals(){for(const auto& slot:state().extensions)if(slot.owner&&slot.bodyCount)return true;return false;}
    bool hasVisualLabels(){for(const auto& slot:state().extensions)if(slot.owner)for(std::uint32_t i=0;i<slot.bodyCount;++i)
        if(slot.bodies[i].label[0]&&(slot.bodies[i].flags&static_cast<unsigned>(ni::VisualFlag::Dimensions)))return true;return false;}
    std::uint64_t dynamicObstacleLayers(){return state().obstacleLayers.load(std::memory_order_acquire);}
    void clear(RE::hknpWorld* world,void* bhk) {
        auto& service=state();if(!service.rockGate)return;
        auto rock=service.rockGate->pauseForMutation();auto mutation=service.gate.pauseForMutation();
        if(service.driver&&(service.command.result==ni::DriveResult::Controlled||hasWeaponConstraintState())&&service.retireWeapon)
            service.retireWeapon(service.retireContext,bhk);
        auto frame=service.frame;frame.phase=ni::Phase::Retire;frame.world=reinterpret_cast<std::uintptr_t>(world);frame.bhkWorld=reinterpret_cast<std::uintptr_t>(bhk);
        for(auto& slot:service.extensions)if(slot.owner) {
            if(slot.callbacks.retire)invoke(slot,frame,[&]{slot.callbacks.retire(&frame,slot.callbacks.context);});
            releaseResources(slot.owner,world,bhk);slot.obstacleLayers=0;slot.bodyCount=0;
        }
        service.command={};service.weaponBody=ni::kInvalidBody;combineMasks(world);service.frame={};
    }
    Status canUnregisterOwner(Owner owner) {
        if(!owner)return Status::Ok;
        for(const auto& value:state().registeredOwners)if(value.load(std::memory_order_acquire)==owner)
            return current?Status::Busy:(provider::runtime::onOwnerThread()?Status::Ok:Status::WrongThread);
        return Status::Ok;
    }
    void clearOwner(Owner owner) {
        if(!owner)return;
        auto& service=state();
        if(current||!provider::runtime::onOwnerThread()) {
            for(auto& pending:service.pendingRemoval){auto value=pending.load(std::memory_order_acquire);
                if(value==owner||(!value&&pending.compare_exchange_strong(value,owner,std::memory_order_acq_rel)))return;}
            service.removeAll.store(true,std::memory_order_release);
            return;
        }
        auto* slot=extension(owner);if(!slot)return;
        if(!service.rockGate) {
            service.gate.pauseAndWait();releaseResources(owner,nullptr,nullptr);
            service.registeredOwners[slot-service.extensions.data()].store(0,std::memory_order_release);
            slot->owner=0;slot->callbacks={};slot->obstacleLayers=0;slot->bodyCount=0;
            if(service.driver==owner){service.driver=0;service.command={};service.weaponBody=ni::kInvalidBody;}
            combineMasks();return;
        }
        auto rock=service.rockGate->pauseForMutation();auto mutation=service.gate.pauseForMutation();
        auto* player=RE::PlayerCharacter::GetSingleton();auto* cell=player?player->GetParentCell():nullptr;
        auto* bhk=cell?cell->GetbhkWorld():nullptr;
        auto* world=bhk?havok_runtime::getHknpWorldFromBhk(bhk):nullptr;
        if(reinterpret_cast<std::uintptr_t>(world)!=service.frame.world){world=nullptr;bhk=nullptr;}
        if(service.driver==owner&&(service.command.result==ni::DriveResult::Controlled||hasWeaponConstraintState())&&service.retireWeapon)service.retireWeapon(service.retireContext,bhk);
        auto frame=service.frame;frame.phase=ni::Phase::Retire;frame.world=reinterpret_cast<std::uintptr_t>(world);frame.bhkWorld=reinterpret_cast<std::uintptr_t>(bhk);
        if(slot->callbacks.retire)invoke(*slot,frame,[&]{slot->callbacks.retire(&frame,slot->callbacks.context);});
        releaseResources(owner,world,bhk);service.registeredOwners[slot-service.extensions.data()].store(0,std::memory_order_release);
        slot->owner=0;slot->callbacks={};slot->obstacleLayers=0;slot->bodyCount=0;
        if(service.driver==owner){service.driver=0;service.command={};service.weaponBody=ni::kInvalidBody;}
        combineMasks(world);
    }
    bool updateWeapon(const PhysicsFrameContext& context,const WeaponCollision& collision,RE::NiNode* node,BethesdaPhysicsBody& body,
        const RE::NiPoint3& center,std::uint64_t generation,RE::NiTransform& requested,bool support,const RE::NiPoint3* grip) {
        auto& service=state();auto* slot=extension(service.driver);if(!slot||!slot->callbacks.weaponUpdate)return true;
        auto rock=service.rockGate->pauseForMutation();auto mutation=service.gate.pauseForMutation();
        ni::WeaponFrame frame{};frame.frame=makeFrame(context,ni::Phase::WeaponUpdate);frame.body=body.getBodyId().value;
        const auto composition=collision.getWeaponCompositionSnapshot();
        frame.generation=generation;frame.form=composition.weaponGenerationKey==generation?composition.weaponFormId:0;frame.supportOwnsPose=support;
        frame.requestedWeapon=toApi(requested);frame.centerWeaponLocal=toApi(center);frame.primaryGripValid=grip!=nullptr;if(grip)frame.primaryGripLocal=toApi(*grip);
        frame.recoveryDistanceGame=g_rockConfig.rockWeaponCollisionGripRecoveryDistanceGameUnits;
        service.weaponBody=frame.body;service.generation=generation;service.collision=&collision;service.weaponNode=node;
        ni::WeaponCommand command{};command.requestedWeapon=frame.requestedWeapon;command.presentation=frame.requestedWeapon;
        const bool ok=invoke(*slot,frame.frame,[&]{slot->callbacks.weaponUpdate(&frame,&command,slot->callbacks.context);});
        service.collision=nullptr;service.weaponNode=nullptr;
        if(!ok||disabled(*slot)||command.size!=sizeof(command)||(command.result!=ni::DriveResult::Unchanged&&command.result!=ni::DriveResult::Controlled)||
            (command.result==ni::DriveResult::Unchanged&&hasWeaponConstraintState())||
            (command.result==ni::DriveResult::Controlled&&(!finite(command.requestedWeapon)||!finite(command.presentation))))return false;
        service.command=command;
        if(command.result==ni::DriveResult::Controlled)requested=fromApi(command.requestedWeapon);
        return true;
    }
    bool weaponControlled(){return state().command.result==ni::DriveResult::Controlled;}
    RE::NiTransform weaponPresentation(){return fromApi(state().command.presentation);}
    bool preCollideWeapon(RE::hknpWorld* world,const havok_physics_timing::PhysicsTimingSample& timing) {
        auto lease=state().gate.tryEnterCallback();if(!lease)return !weaponControlled();
        if(reinterpret_cast<std::uintptr_t>(world)!=state().frame.world)return !weaponControlled();
        auto* slot=extension(state().driver);if(!slot||!slot->callbacks.weaponPreCollide)return true;
        const auto frame=stepFrame(world,timing);std::uint32_t result=0;
        return invoke(*slot,frame,[&]{result=slot->callbacks.weaponPreCollide(&frame,slot->callbacks.context);})&&result!=0;
    }
    void weaponContact(RE::hknpWorld* world,std::uint32_t weapon,std::uint32_t target,std::uint32_t layer,const RE::NiPoint3& point,const RE::NiPoint3& normal,std::uint32_t pointCount) {
        auto lease=state().gate.tryEnterCallback();if(!lease)return;
        auto* slot=extension(state().driver);if(!slot||!slot->callbacks.weaponContact||state().weaponBody!=weapon||state().frame.world!=reinterpret_cast<std::uintptr_t>(world))return;
        ni::Contact contact{};contact.frame=state().frame;contact.frame.phase=ni::Phase::WeaponContact;
        contact.weapon=weapon;contact.target=target;contact.targetLayer=layer;contact.pointWorld=toApi(point);
        ni::v1_1::ContactDetails details{};details.pointCount=pointCount;
        const float length=normal.x*normal.x+normal.y*normal.y+normal.z*normal.z;
        if(pointCount>0&&pointCount<=4&&std::isfinite(length)&&length>0.000001f) {
            const float inverse=1.0f/std::sqrt(length);
            details.valid=1;details.normalWorld={normal.x*inverse,normal.y*inverse,normal.z*inverse};
        }
        const auto* previous=contactDetails;contactDetails=&details;
        invoke(*slot,contact.frame,[&]{slot->callbacks.weaponContact(&contact,slot->callbacks.context);});
        contactDetails=previous;
    }
    void retireWeapon(RE::hknpWorld* world,void* bhk) {
        auto& service=state();auto* slot=extension(service.driver);if(!slot||!service.rockGate)return;
        auto rock=service.rockGate->pauseForMutation();auto mutation=service.gate.pauseForMutation();
        auto frame=service.frame;frame.phase=ni::Phase::WeaponRetire;frame.world=reinterpret_cast<std::uintptr_t>(world);frame.bhkWorld=reinterpret_cast<std::uintptr_t>(bhk);
        if(slot->callbacks.weaponRetire)invoke(*slot,frame,[&]{slot->callbacks.weaponRetire(&frame,slot->callbacks.context);});
        // The parent retires its tool body immediately after this returns.
        // Consumer failure cannot leave a borrowed native constraint attached.
        releaseWeaponResources(slot->owner,world,bhk);
        service.command={};service.weaponBody=ni::kInvalidBody;
    }
    ni::SelectionReply classify(const ni::SelectionQuery& query) {
        ni::SelectionReply result{};auto& service=state();if(!service.rockGate)return result;
        auto rock=service.rockGate->pauseForMutation();auto mutation=service.gate.pauseForMutation();
        auto frame=service.frame;frame.phase=ni::Phase::Selection;frame.world=query.world;
        for(auto& slot:service.extensions)if(slot.owner&&slot.callbacks.classifySelection) {
            ni::SelectionReply candidate{};
            if(!invoke(slot,frame,[&]{slot.callbacks.classifySelection(&query,&candidate,slot.callbacks.context);}))continue;
            if(candidate.size==sizeof(candidate)&&candidate.kind!=ni::SelectionKind::Unhandled&&
                static_cast<unsigned>(candidate.kind)<=static_cast<unsigned>(ni::SelectionKind::ExtractableItem)&&candidate.reason[63]==0){candidate.owner=slot.owner;return candidate;}
        }
        return result;
    }
    bool validateSelection(const ni::SelectionQuery& query,const ni::SelectionReply& selection) {
        auto& service=state();auto* slot=extension(selection.owner);
        if(!service.rockGate||!slot||!slot->callbacks.validateSelection)return false;
        auto rock=service.rockGate->pauseForMutation();auto mutation=service.gate.pauseForMutation();
        auto frame=service.frame;frame.phase=ni::Phase::Selection;frame.world=query.world;std::uint32_t valid=0;
        return invoke(*slot,frame,[&]{valid=slot->callbacks.validateSelection(&query,&selection,slot->callbacks.context);})&&valid==1;
    }
    Status extract(const ni::SelectionQuery& query,const ni::SelectionReply& selection,ni::ExtractedReference& result) {
        auto& service=state();auto* slot=extension(selection.owner);if(!service.rockGate||!slot||!slot->callbacks.extractSelection)return Status::OwnerNotRegistered;
        auto rock=service.rockGate->pauseForMutation();auto mutation=service.gate.pauseForMutation();
        auto frame=service.frame;frame.phase=ni::Phase::Extraction;frame.world=query.world;Status status=Status::InternalError;
        if(!invoke(*slot,frame,[&]{status=slot->callbacks.extractSelection(&query,&selection,&result,slot->callbacks.context);}))return Status::InternalError;
        if(status==Status::Ok&&(result.size!=sizeof(result)||!result.nativeHandle||!result.referenceForm||!result.itemForm))return Status::TargetInvalid;
        return status;
    }
    ni::ProximityResult proximityTarget(std::uintptr_t reference,float radiusGame) {
        auto& service=state();ni::ProximityResult result{};if(!service.rockGate)return result;
        auto rock=service.rockGate->pauseForMutation();auto mutation=service.gate.pauseForMutation();
        auto frame=service.frame;frame.phase=ni::Phase::Gameplay;
        ni::ProximityQuery query{};query.reference=reference;query.radiusGame=radiusGame;
        for(auto& slot:service.extensions)if(slot.owner&&slot.callbacks.proximityTarget) {
            ni::ProximityResult candidate{};
            if(invoke(slot,frame,[&]{slot.callbacks.proximityTarget(&query,&candidate,slot.callbacks.context);})&&candidate.size==sizeof(candidate)&&candidate.handled)return candidate;
        }
        return result;
    }
    std::uint32_t impactEffect(std::uintptr_t source,std::uintptr_t target,float damage) {
        auto& service=state();if(!service.rockGate||!target||!std::isfinite(damage)||damage<=0)return 0;
        auto rock=service.rockGate->pauseForMutation();auto mutation=service.gate.pauseForMutation();
        auto frame=service.frame;frame.phase=ni::Phase::Gameplay;
        ni::ImpactQuery query{};query.source=source;query.target=target;query.authoredDamage=damage;
        for(auto& slot:service.extensions)if(slot.owner&&slot.callbacks.impactEffect) {
            ni::ImpactResult candidate{};
            if(!invoke(slot,frame,[&]{slot.callbacks.impactEffect(&query,&candidate,slot.callbacks.context);}))return 0;
            if(candidate.size!=sizeof(candidate))return 0;
            if(candidate.handled)return candidate.targetForm;
        }
        return 0;
    }
    ni::PushKind classifyPush(const ni::SelectionQuery& query) {
        auto& service=state();if(!service.rockGate)return ni::PushKind::Unhandled;
        auto rock=service.rockGate->pauseForMutation();auto mutation=service.gate.pauseForMutation();
        auto frame=service.frame;frame.phase=ni::Phase::Selection;frame.world=query.world;
        for(auto& slot:service.extensions)if(slot.owner&&slot.callbacks.classifyPush) {
            auto result=ni::PushKind::Blocked;
            if(!invoke(slot,frame,[&]{result=slot.callbacks.classifyPush(&query,slot.callbacks.context);}))return ni::PushKind::Blocked;
            if(result!=ni::PushKind::Unhandled)return result;
        }
        return ni::PushKind::Unhandled;
    }
    const ni::ApiV1& table() noexcept {
        static const ni::ApiV1 api{&registerExtension,&unregisterExtension,&readBody,&resolveCollisionBody,&resolveBodyReference,&queryNodeRelation,
            &createBody,&retireBody,&placeBody,&driveBody,&setBodyFilter,&activateWeapon,&createSlider,&setSliderLimits,&destroySlider,
            &setWeaponContactExclusion,&clearWeaponContactExclusion,&hasWeaponContactExclusion,&setDynamicObstacleLayers,
            &copyWeaponSources,&getSourceGeometry,&publishVisuals};
        return api;
    }
    const ni::v1_1::Api& tableV1_1() noexcept {
        static const ni::v1_1::Api value{table(),&readContactDetails};return value;
    }
}
