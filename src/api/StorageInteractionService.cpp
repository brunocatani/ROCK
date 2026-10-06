#include <ROCK/StorageInteraction.h>
#include "api/StorageInteractionService.h"
#include "api/StorageInteractionPolicy.h"
#include "api/StorageService.h"
#include "api/StorageSceneLookup.h"
#include "api/ProviderRuntimeServices.h"
#include "physics-interaction/core/PhysicsInteractionInternal.h"
#include "physics-interaction/weapon/WeaponGeometry.h"
#include "physics-interaction/native/HavokCompoundShapeBuilder.h"
#include "physics-interaction/native/HavokConvexShapeBuilder.h"
#include "physics-interaction/native/NativePlayerCollisionFilter.h"
#include <chrono>

namespace rock::storage_interaction
{
    namespace
    {
        namespace contract=api::storage_interaction;
        namespace policy=storage_interaction_policy;
        using Shape=std::unique_ptr<RE::hknpShape,havok_compound_shape_builder::HavokShapeRelease>;
        struct Group { std::vector<RE::NiPoint3> points; };
        struct Display {
            api::OwnerToken owner{};
            contract::DisplayV1 request;
            std::uint64_t deadline{}, publishedFrame{};
            RE::NiPointer<RE::TESObjectREFR> parent;
            RE::NiPointer<RE::NiAVObject> node;
            BethesdaPhysicsBody body;
            GeneratedKeyframedBodyDriveState drive;
            Shape shape;
            std::vector<Group> groups;
            weapon_geometry_work::Task preparation;
            std::array<std::atomic_uint64_t,2> contactFrames{};
            std::atomic_bool rebuild{};
            float sourceScale{}, physicsScale{};
            unsigned freeHands{};
            bool failed{}, expired{};
            std::uint32_t boundReference{},boundParent{};
        };
        struct Routing { api::OwnerToken owner{}; contract::ShoulderRoutingV1 request; std::uint64_t deadline{}; };
        // The frame owner owns requests/scene/body lifetime. Native drive,
        // simulation filtering and manifold callbacks borrow through this gate.
        // They never resolve consumer pointers or perform structural mutations.
        struct Service {
            std::array<Display,contract::kMaximumDisplays> displays;
            std::array<Routing,8> routes;
            PhysicsCallbackQuiescenceGate callbacks;
            RE::hknpWorld* world{};
            void* bhk{};
            // Palm and all finger segments; the forearm remains excluded.
            struct HandBody { std::uint32_t id{policy::invalidBody}; std::uintptr_t object{}; };
            std::array<std::array<HandBody,dynamic_hand_collision_telemetry::kFirstForearmSlot>,2> hands{};
            unsigned preparationCursor{};
            std::atomic_bool active{};
        };
        Service service;
        api::Status authorize(api::OwnerToken owner,unsigned permissions=3)
        { return provider::runtime::authorize(owner,api::InterfaceId::Grab,permissions,true); }
        bool sameGeneration(const auto& request,const provider::RockProviderFrameSnapshot& frame)
        { return request.worldGeneration==frame.worldGeneration && request.skeletonGeneration==frame.skeletonGeneration && request.providerGeneration==frame.providerGeneration; }
        void retire(Display& value,RE::hknpWorld* world)
        {
            value.preparation={}; value.groups.clear();
            if(value.body.isValid()) {
                if(world && world==service.world && service.bhk) value.body.retireDeferred(service.bhk);
                else value.body.reset();
            }
            clearGeneratedKeyframedBodyDriveState(value.drive);
            value.shape.reset(); value.node.reset(); value.parent.reset();
            value.contactFrames[0]=0; value.contactFrames[1]=0;
            value.rebuild=false;
            value.sourceScale=value.physicsScale=0; value.failed=false; value.freeHands=0;
            value.boundReference=value.boundParent=0;
        }
        bool captureGeometry(Display& value)
        {
            std::vector<TriangleData> triangles;
            std::vector<GrabSurfaceTriangleData> surfaces;
            const auto result=extractBoundedSurfaceTriangles(value.node.get(),triangles,surfaces,32,128,65536,true);
            if(!result.hasTriangles() || result.nodeBudgetExceeded || result.shapeBudgetExceeded || result.triangleBudgetExceeded) return false;
            auto rigid=value.node->world; rigid.scale=1;
            const auto inverse=transform_math::invertTransform(rigid);
            RE::NiAVObject* previous{};
            for(const auto& surface:surfaces) {
                if(surface.sourceShape!=previous || value.groups.empty()) { value.groups.emplace_back(); previous=surface.sourceShape; }
                auto& points=value.groups.back().points;
                for(const auto& p:{surface.triangle.v0,surface.triangle.v1,surface.triangle.v2}) {
                    const auto local=transformPoint(inverse,p);
                    if(!std::isfinite(local.x) || !std::isfinite(local.y) || !std::isfinite(local.z)) return false;
                    points.push_back(local);
                }
            }
            return !value.groups.empty();
        }
        weapon_geometry_work::Task prepare(Display& value)
        {
            std::vector<Shape> shapes;
            for(const auto& group:value.groups) {
                weapon_collision_geometry_math::ConvexSupportFitResult<RE::NiPoint3> fitted;
                auto fitting=weapon_collision_geometry_math::fitConvexSupportPointCloudDeferred(group.points,48,192,.5f,true,fitted);
                while(fitting.step()) co_yield 0;
                if(!fitted.accepted || fitted.points.size()<4) { value.failed=true; co_return; }
                for(auto& point:fitted.points) point=point*value.physicsScale;
                Shape shape(havok_convex_shape_builder::buildConvexShapeFromLocalHavokPoints(fitted.points,.01f*value.physicsScale));
                if(!shape) { value.failed=true; co_return; }
                shapes.push_back(std::move(shape));
                co_yield 0;
            }
            if(shapes.size()==1) value.shape=std::move(shapes.front());
            else {
                std::vector<havok_compound_shape_builder::CompoundChild> children;
                for(const auto& shape:shapes) children.push_back({shape.get(),{}});
                value.shape.reset(havok_compound_shape_builder::buildStaticCompoundShape(children));
            }
            if(!value.shape) value.failed=true;
        }
        bool advancePreparation(Display& value)
        {
            // One shared slice per frame, with bounded coroutine quanta. Dense
            // meshes can finish promptly without doing an entire fit at once.
            const auto start=std::chrono::steady_clock::now();
            for(unsigned resumes=0;resumes<512;++resumes) {
                if(!value.preparation.step()) return true;
                if(std::chrono::steady_clock::now()-start>=std::chrono::microseconds(400)) break;
            }
            return false;
        }
        RE::NiAVObject* resolve(Display& value,PhysicsInteraction& physics)
        {
            if(!storage_service::ownsStored(value.owner,value.request.storedReference)) return nullptr;
            auto* parent=RE::TESForm::GetFormByID<RE::TESObjectREFR>(value.request.parentReference);
            if(!parent || parent->IsDisabled() || parent->IsDeleted()) return nullptr;
            if(parent!=RE::PlayerCharacter::GetSingleton() &&
                (!physics.getRightHand().isHolding() || physics.getRightHand().getHeldRef()!=parent) &&
                (!physics.getLeftHand().isHolding() || physics.getLeftHand().getHeldRef()!=parent)) return nullptr;
            auto* root=parent->Get3D();
            auto* node=storage_scene_lookup::find(root,RE::BSFixedString(value.request.nodeName));
            if(!node || node==root || node->GetAppCulled() || !finiteNiTransform(node->world)) return nullptr;
            value.parent.reset(parent); return node;
        }
        unsigned freeHands(api::OwnerToken owner)
        {
            unsigned mask{};
            for(unsigned h=0;h<2;++h) {
                provider::RockProviderHandInteractionStateV1 state;
                const auto hand=h?provider::RockProviderHand::Left:provider::RockProviderHand::Right;
                using Flag=api::grab::HandInteractionFlagV1;
                using Phase=provider::RockProviderHandInteractionPhaseV1;
                constexpr unsigned occupied=unsigned(Flag::FiringGrip)|unsigned(Flag::PartGrip)|unsigned(Flag::PartCarry)|unsigned(Flag::TouchGrab)|unsigned(Flag::NativeWeaponCarry)|unsigned(Flag::RockGripActive)|unsigned(Flag::TransitionSuppressed);
                if(provider::runtime::apiGetHandInteractionStateV1(owner,hand,&state)==provider::RockProviderResultV1::Ok &&
                    (state.flags & unsigned(Flag::Valid)) && !(state.flags & occupied) && !state.heldBodyCount &&
                    (state.phase==Phase::Idle || state.phase==Phase::Touching || state.phase==Phase::Selecting))
                    mask|=1u<<h;
            }
            return mask;
        }
        api::Status ROCK_CALL setDisplay(api::OwnerToken owner,const contract::DisplayV1* request) noexcept try
        {
            if(!request) return api::Status::InvalidArgument;
            if(request->size!=sizeof(*request)) return api::Status::InvalidSize;
            if(request->version!=1) return api::Status::UnsupportedVersion;
            if(!request->displayId || !request->parentReference || !request->storedReference ||
                !policy::validLease(request->leaseFrames) || !request->nodeName[0] || !std::memchr(request->nodeName,0,sizeof(request->nodeName))) return api::Status::InvalidArgument;
            const auto access=authorize(owner); if(access!=api::Status::Ok) return access;
            provider::RockProviderFrameSnapshot frame;
            if(!provider::runtime::apiGetFrameSnapshot(&frame) || !frame.providerReady || frame.menuBlocking) return api::Status::NotReady;
            if(!sameGeneration(*request,frame)) return api::Status::GenerationMismatch;
            if(!storage_service::ownsStored(owner,request->storedReference)) return api::Status::OwnerConflict;
            Display* slot{};
            for(auto& entry:service.displays) {
                if(entry.owner==owner && entry.request.displayId==request->displayId) { slot=&entry; break; }
                if(!entry.owner && !slot) slot=&entry;
            }
            if(!slot) return api::Status::CapacityFull;
            slot->owner=owner; slot->request=*request; slot->deadline=frame.frameIndex+request->leaseFrames; slot->expired=false;
            service.active=true;
            return api::Status::Ok;
        } catch(...) { return api::Status::InternalError; }
        api::Status ROCK_CALL clearDisplay(api::OwnerToken owner,std::uint64_t id) noexcept
        {
            const auto access=authorize(owner,0); if(access!=api::Status::Ok) return access;
            for(auto& entry:service.displays) if(entry.owner==owner && entry.request.displayId==id) entry.expired=true;
            return api::Status::Ok;
        }
        api::Status ROCK_CALL getContact(api::OwnerToken owner,api::Hand hand,contract::ContactV1* result) noexcept
        {
            if(!result || (hand!=api::Hand::Left && hand!=api::Hand::Right)) return api::Status::InvalidArgument;
            if(result->size!=sizeof(*result)) return api::Status::InvalidSize;
            if(result->version!=1) return api::Status::UnsupportedVersion;
            *result={}; result->hand=hand;
            const auto access=authorize(owner,1); if(access!=api::Status::Ok) return access;
            provider::RockProviderFrameSnapshot frame;
            if(!provider::runtime::apiGetFrameSnapshot(&frame) || !frame.providerReady || frame.menuBlocking) return api::Status::NotReady;
            result->sample={frame.frameIndex,frame.stateSequence,frame.worldGeneration,frame.skeletonGeneration,frame.providerGeneration,frame.collisionGeneration};
            if(!policy::allowsHand(freeHands(owner),hand==api::Hand::Left)) return api::Status::Ok;
            for(const auto& entry:service.displays) if(entry.owner==owner && !entry.expired && !entry.rebuild && entry.boundReference==entry.request.storedReference &&
                entry.boundParent==entry.request.parentReference && entry.deadline>=frame.frameIndex && sameGeneration(entry.request,frame)) {
                const auto contact=entry.contactFrames[hand==api::Hand::Left?1:0].load(std::memory_order_acquire);
                if(!policy::fresh(frame.frameIndex,contact)) continue;
                if(!result->valid || contact>result->contactFrame) {
                    result->valid=1; result->displayId=entry.request.displayId;
                    result->storedReference=entry.request.storedReference; result->contactFrame=contact;
                }
            }
            return api::Status::Ok;
        }
        api::Status ROCK_CALL setRouting(api::OwnerToken owner,const contract::ShoulderRoutingV1* request) noexcept
        {
            if(!request) return api::Status::InvalidArgument;
            if(request->size!=sizeof(*request)) return api::Status::InvalidSize;
            if(request->version!=1) return api::Status::UnsupportedVersion;
            if(!policy::validRouting(request->handMask,request->leaseFrames,request->reserved)) return api::Status::InvalidArgument;
            const auto access=authorize(owner); if(access!=api::Status::Ok) return access;
            provider::RockProviderFrameSnapshot frame;
            if(!provider::runtime::apiGetFrameSnapshot(&frame) || !frame.providerReady || frame.menuBlocking) return api::Status::NotReady;
            if(!sameGeneration(*request,frame)) return api::Status::GenerationMismatch;
            Routing* slot{};
            for(auto& entry:service.routes) {
                if(entry.owner && (entry.deadline<frame.frameIndex || !sameGeneration(entry.request,frame) || authorize(entry.owner,0)!=api::Status::Ok)) entry={};
                if(entry.owner==owner) { slot=&entry; break; }
                if(!entry.owner && !slot) slot=&entry;
            }
            if(!request->handMask) { if(slot && slot->owner==owner) *slot={}; return api::Status::Ok; }
            if(!slot) return api::Status::CapacityFull;
            *slot=request->handMask?Routing{owner,*request,frame.frameIndex+request->leaseFrames}:Routing{};
            return api::Status::Ok;
        }
        api::Status ROCK_CALL clearOwner(api::OwnerToken owner) noexcept
        {
            const auto access=authorize(owner,0); if(access!=api::Status::Ok) return access;
            for(auto& entry:service.displays) if(entry.owner==owner) entry.expired=true;
            for(auto& entry:service.routes) if(entry.owner==owner) entry={};
            return api::Status::Ok;
        }
    }
    bool blocksShoulder(bool left)
    {
        provider::RockProviderFrameSnapshot frame;
        if(!provider::runtime::apiGetFrameSnapshot(&frame)) return false;
        for(auto& route:service.routes) if(route.owner) {
            if(route.deadline<frame.frameIndex || !sameGeneration(route.request,frame) || authorize(route.owner,0)!=api::Status::Ok) { route={}; continue; }
            if(policy::allowsHand(route.request.handMask,left)) return true;
        }
        return false;
    }
    void update(PhysicsInteraction& physics,const PhysicsFrameContext& frame,const DynamicHandCollisionRuntime& hands,PhysicsCallbackQuiescenceGate& driveGate) try
    {
        if(!service.active.load(std::memory_order_acquire)) return;
        if(!frame.worldReady || frame.menuBlocked || !native_player_collision::installed()) { shutdown(frame.hknpWorld); return; }
        provider::RockProviderFrameSnapshot current;
        if(!provider::runtime::apiGetFrameSnapshot(&current)) return;
        auto driveMutation=driveGate.pauseForMutation();
        auto mutation=service.callbacks.pauseForMutation();
        if(service.world && service.world!=frame.hknpWorld) for(auto& entry:service.displays) { retire(entry,nullptr); entry.owner=0; }
        service.world=frame.hknpWorld; service.bhk=frame.bhkWorld;
        dynamic_hand_collision_telemetry::Snapshot handState;
        const bool haveHands=hands.getTelemetrySnapshot(handState);
        for(unsigned h=0;h<2;++h) for(unsigned part=0;part<service.hands[h].size();++part) {
            auto& hand=service.hands[h][part];
            hand.id=haveHands?handState.hands[h].twins[part].bodyId:policy::invalidBody;
            const auto body=havok_runtime::snapshotBodyIdentity(service.world,{hand.id});
            hand.object=body.valid?reinterpret_cast<std::uintptr_t>(body.collisionObject):0;
        }
        bool advanced{}, captured{}, anyOwner{};
        std::array<BethesdaPhysicsBody*,contract::kMaximumDisplays> refresh{}; unsigned refreshCount{};
        for(unsigned i=0;i<service.displays.size();++i) {
            auto& entry=service.displays[(i+service.preparationCursor)%service.displays.size()];
            if(!entry.owner) continue;
            if(entry.expired || entry.deadline<current.frameIndex || !sameGeneration(entry.request,current) || authorize(entry.owner)!=api::Status::Ok) {
                retire(entry,service.world); entry.owner=0; continue;
            }
            anyOwner=true;
            auto* node=resolve(entry,physics);
            if(!node) { retire(entry,service.world); continue; }
            const bool reparented=entry.boundParent && entry.boundParent!=entry.request.parentReference;
            if(entry.rebuild || entry.node.get()!=node || entry.boundReference!=entry.request.storedReference ||
                std::abs(entry.sourceScale-node->world.scale)>.0001f || entry.physicsScale!=physics_scale::gameToHavok()) {
                retire(entry,service.world);
                // Bound scene extraction too, including simultaneous consumer
                // registrations. Retired sources cannot leave stale contacts.
                if(captured) continue;
                captured=true;
                entry.node.reset(node); entry.sourceScale=node->world.scale; entry.physicsScale=physics_scale::gameToHavok();
                entry.boundReference=entry.request.storedReference; entry.boundParent=entry.request.parentReference;
                entry.parent.reset(RE::TESForm::GetFormByID<RE::TESObjectREFR>(entry.boundParent));
                if(!captureGeometry(entry)) {
                    entry.failed=true;
                    ROCK_LOG_WARN(Hand,"Stored display geometry unavailable or exceeds extraction bounds: display={} item={:08X}",entry.request.displayId,entry.request.storedReference);
                }
                else entry.preparation=prepare(entry);
            }
            if(reparented) {
                entry.boundParent=entry.request.parentReference;
                entry.contactFrames[0]=0; entry.contactFrames[1]=0;
            }
            const auto mask=freeHands(entry.owner);
            bool filterChanged=entry.freeHands!=mask;
            entry.freeHands=mask; entry.publishedFrame=current.frameIndex;
            if(!entry.failed && !entry.body.isValid() && !advanced) {
                advanced=true;
                if(advancePreparation(entry)) {
                    entry.preparation={}; entry.groups.clear();
                    if(!entry.failed && entry.shape) {
                        auto target=node->world; target.scale=1;
                        const auto filter=collision_layer_policy::ROCK_LAYER_STORED_DISPLAY | 0x4000u;
                        if(entry.body.create(service.world,service.bhk,entry.shape.get(),filter,RE::hknpMaterialId{0},BethesdaMotionType::Keyframed,"ROCK_StoredDisplay") &&
                            placeGeneratedKeyframedBodyImmediately(entry.body,target)) {
                            initializeGeneratedKeyframedBodyDriveState(entry.drive,target);
                            entry.body.setCollisionFilterInfo(collision_layer_policy::ROCK_LAYER_STORED_DISPLAY);
                            filterChanged=true;
                            ROCK_LOG_INFO(Hand,"Stored display collider ready: owner={} display={} item={:08X} body={}",entry.owner,entry.request.displayId,entry.request.storedReference,entry.body.getBodyId().value);
                        } else entry.failed=true;
                    }
                    if(entry.failed) {
                        if(entry.body.isValid()) entry.body.retireDeferred(service.bhk);
                        ROCK_LOG_WARN(Hand,"Stored display collision unavailable: display={} item={:08X}; visual/item ownership preserved",entry.request.displayId,entry.request.storedReference);
                    }
                }
            }
            if(entry.body.isValid()) {
                auto target=node->world; target.scale=1;
                // A consumer reparent is placement, not a physical sweep across
                // the player. The unchanged local mesh keeps its fitted hull.
                if(reparented) {
                    if(!placeGeneratedKeyframedBodyImmediately(entry.body,target)) {
                        ROCK_LOG_SAMPLE_WARN(Hand,1000,"Stored display reparent placement failed; collider retired");
                        retire(entry,service.world); continue;
                    }
                    initializeGeneratedKeyframedBodyDriveState(entry.drive,target);
                }
                (void)queueGeneratedKeyframedBodyTarget(entry.drive,target,frame.deltaSeconds,100);
                if(filterChanged) refresh[refreshCount++]=&entry.body;
            }
        }
        service.preparationCursor=(service.preparationCursor+1)%service.displays.size();
        service.callbacks.resumeCallbacks();
        for(unsigned i=0;i<refreshCount;++i) refresh[i]->refreshCollisionFilter(service.world);
        service.active=anyOwner;
    } catch(...) {
        shutdown(frame.hknpWorld);
        ROCK_LOG_SAMPLE_WARN(Hand,1000,"Stored display preparation failed; generated contacts retired");
    }
    void flush(RE::hknpWorld* world,const havok_physics_timing::PhysicsTimingSample& timing)
    {
        if(!service.active.load(std::memory_order_acquire)) return;
        auto lease=service.callbacks.tryEnterCallback();
        if(!lease || world!=service.world) return;
        for(auto& entry:service.displays) if(entry.body.isValid()) {
            const auto result=driveGeneratedKeyframedBody(world,entry.body,entry.drive,timing,"stored-display",0);
            // Structural changes belong to the frame owner. The callback only
            // withdraws admission and requests retirement at the next update.
            if(result.shouldRequestRebuild() || result.skippedStale) entry.rebuild=true;
        }
    }
    void shutdown(RE::hknpWorld* world)
    {
        auto mutation=service.callbacks.pauseForMutation();
        for(auto& entry:service.displays) { retire(entry,world); entry.owner=0; }
        service.routes={}; service.world=nullptr; service.bhk=nullptr; service.hands={};
        service.active=false;
    }
    bool allowsPair(RE::hknpWorld* world,std::uint32_t a,std::uint32_t b) noexcept
    {
        if(!service.active.load(std::memory_order_acquire)) return true;
        std::uint32_t af{},bf{};
        const bool haveA=havok_runtime::tryReadFilterInfo(world,{a},af),haveB=havok_runtime::tryReadFilterInfo(world,{b},bf);
        constexpr auto layer=collision_layer_policy::ROCK_LAYER_STORED_DISPLAY;
        const bool aDisplay=haveA && (af & 0x7F)==layer,bDisplay=haveB && (bf & 0x7F)==layer;
        if(!aDisplay && !bDisplay) return true;
        if(!haveA || !haveB) return false;
        auto lease=service.callbacks.tryEnterCallback();
        if(!lease || world!=service.world || aDisplay==bDisplay) return false;
        const auto id=aDisplay?a:b,peer=aDisplay?b:a;
        for(const auto& entry:service.displays) if(!entry.rebuild && entry.body.isValid() && entry.body.getBodyId().value==id) {
            const auto identity=havok_runtime::snapshotBodyIdentity(world,{id});
            if(!identity.valid || identity.collisionObject!=entry.body.getCollisionObject()) return false;
            for(unsigned h=0;h<2;++h) if(policy::allowsHand(entry.freeHands,h==1)) for(const auto& hand:service.hands[h]) if(peer==hand.id) {
                const auto handIdentity=havok_runtime::snapshotBodyIdentity(world,{peer});
                return handIdentity.valid && policy::exactHand(peer,reinterpret_cast<std::uintptr_t>(handIdentity.collisionObject),hand.id,hand.object);
            }
            return false;
        }
        return false;
    }
    void contact(RE::hknpWorld* world,std::uint32_t body,bool left) noexcept
    {
        if(!service.active.load(std::memory_order_acquire)) return;
        auto lease=service.callbacks.tryEnterCallback(); if(!lease || world!=service.world) return;
        for(auto& entry:service.displays) if(!entry.rebuild && entry.body.isValid() && entry.body.getBodyId().value==body && policy::allowsHand(entry.freeHands,left))
            entry.contactFrames[left?1:0].store(entry.publishedFrame,std::memory_order_release);
    }
}
extern "C" __declspec(dllexport) const rock::api::storage_interaction::ApiV1* ROCK_CALL ROCK_StorageInteraction_QueryV1(std::uint32_t major,std::uint32_t bytes) noexcept
{
    using namespace rock::storage_interaction;
    static const rock::api::storage_interaction::ApiV1 table{sizeof(table),1,&setDisplay,&clearDisplay,&getContact,&setRouting,&clearOwner};
    static_assert(sizeof(table)==48);
    return rock::storage_interaction_policy::supports(major,bytes)?&table:nullptr;
}
