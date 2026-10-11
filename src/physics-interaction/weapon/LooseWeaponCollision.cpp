#include "physics-interaction/weapon/LooseWeaponCollision.h"
#include "api/LooseWeaponStateService.h"
#include "physics-interaction/weapon/EquippedWeaponVisualState.h"
#include "physics-interaction/weapon/LooseWeaponCollisionPolicy.h"
#include "physics-interaction/weapon/WeaponRestSupport.h"
#include "physics-interaction/weapon/WeaponCollisionInternal.h"
#include "physics-interaction/core/PhysicsFrameContext.h"
#include "physics-interaction/core/RockRuntimeState.h"
#include "physics-interaction/native/NativeObjectPlacement.h"
#include "physics-interaction/native/HavokWorldLock.h"
#include "physics-interaction/native/HeldScenePresentation.h"
#include "physics-interaction/native/PhysicsRecursiveWrappers.h"
#include "physics-interaction/object/ObjectPhysicsBodySet.h"
#include "RE/Bethesda/TESBoundObjects.h"
#include "physics-interaction/performance/PerformanceProfiler.h"
#include <bit>
#include <chrono>

namespace rock {
namespace {
    using Shape = std::unique_ptr<RE::hknpShape, havok_compound_shape_builder::HavokShapeRelease>;
    using Suppressor = collision_suppression_registry::CollisionSuppressionOwner;
    using Motion = physics_body_classifier::BodyMotionType;
    constexpr auto kOwner = Suppressor::LooseGeneratedCollision;
    constexpr std::uint32_t kContactFlags = 0x40u | 0x80u;
    // Body-flag leases are keyed by the exact mask AND native update mode.
    // Match HandGrab's loose-object contact lease (0xC0, mode 0), otherwise
    // hand release clears our manifold events as if it were the final owner.
    constexpr std::uint32_t kContactFlagMode = 0u;
    using SetShape = void (*)(RE::hknpWorld*, RE::hknpBodyId, const RE::hknpShape*);

    SetShape shapeSetter()
    {
        static const auto result = []() -> SetShape {
            // VR 153AC70 swaps body+48, rebuilds caches/bounds, publishes shape
            // change signals and balances shape references. 1AF0A40 independently
            // passes world/body-id/shape. Do not write body->shape directly.
            const auto address = REL::Offset(0x153AC70).address();
            constexpr std::array<std::uint8_t, 10> expected{0x48,0x89,0x5C,0x24,0x18,0x48,0x89,0x6C,0x24,0x20};
            std::array<std::uint8_t, 10> actual{};
            // 153AC70 and 153FF80 both retain/release body shapes when this
            // native mode is enabled (the unpacked image initializes it to 1).
            // That retained reference covers native teardown after world loss.
            std::uint8_t referenceManaged{};
            if (!native_object_placement::available() ||
                !native_memory::guardedCopyFromMemory(reinterpret_cast<void*>(address), actual.data(), actual.size()) || actual != expected ||
                !native_memory::tryReadValue(reinterpret_cast<const std::uint8_t*>(REL::Offset(0x386601E).address()),referenceManaged) || !referenceManaged) return nullptr;
            return reinterpret_cast<SetShape>(address);
        }();
        return result;
    }
    struct WriteScope
    {
        RE::hknpWorld* world;
        explicit WriteScope(RE::hknpWorld* value) : world(value) {
            // These two entries are validated by native_object_placement::available.
            reinterpret_cast<void (*)(RE::hknpWorld*)>(REL::Offset(0x1DF5FB0).address())(world);
        }
        ~WriteScope() { reinterpret_cast<void (*)(RE::hknpWorld*)>(REL::Offset(0x1DF5FD0).address())(world); }
    };
    using weapon_rest_support::sameSurfacePose;
    using weapon_rest_support::parkingSurface;
}

struct LooseWeaponCollision::State
{
    RE::NiPointer<RE::TESObjectREFR> ref;
    RE::NiPointer<RE::NiAVObject> root;
    RE::hknpWorld* world{};
    struct Body {
        std::uint32_t id{};
        RE::NiPointer<RE::NiAVObject> node;
        RE::NiPointer<RE::NiCollisionObject> collision;
        Shape original;
        RE::NiTransform rootInBody{};
        RE::NiTransform restPose{};
        havok_compound_shape_builder::DynamicCompoundShape generated;
        std::vector<std::size_t> sources;
        std::vector<havok_compound_shape_builder::ChildTransform> poses;
        bool suppressed{}, changed{}, flagLease{};
    };
    struct Source {
        Shape shape;
        RE::NiPointer<RE::NiAVObject> node;
        RE::NiPoint3 center{};
    };
    std::vector<Body> bodies;
    std::vector<Source> sources;
    std::unique_ptr<WeaponCollision> builder;
    object_physics_body_set::ObjectPhysicsBodyScanCursor scanCursor;
    object_physics_body_set::ObjectPhysicsBodyScanCache scanCache;
    object_physics_body_set::BodySetScanOptions scanOptions;
    loose_weapon_collision_policy::Parking parking;
    std::size_t nextSource{}, nextMapping{}, nextCompound{}, primaryBody{};
    bool scanned{}, installed{}, frozen{}, parked{}, scriptStarted{}, failed{}, nativeOnly{};
    bool held{}, supported{}, stationary{};
    unsigned parkingStage{};
    float unsettledSeconds{};
    bool reportedUnsettled{};
    std::size_t preparationFrames{};
    double maxPreparationSliceMs{};
    const char* phase{"Preparing"};

    State(RE::TESObjectREFR* value, RE::hknpWorld* w) : ref(value), root(value->Get3D()), world(w)
    {
        scanOptions.requireSameResolvedRef = true;
        scanOptions.allowUnresolvedRefBodies = false;
        scanOptions.maxDepth = 32;
        scanOptions.allowWeaponRefExpansion = true;
        if (!object_physics_body_set::beginObjectPhysicsBodyScanCache(ref.get(), scanOptions, scanCursor, scanCache)) failed = true;

    }
    bool current(const Body& body, bool checkShape = true) const
    {
        const auto live = havok_runtime::snapshotBody(world, RE::hknpBodyId{body.id});
        return live.valid && live.body && live.ownerNode == body.node.get() && live.collisionObject == body.collision.get() &&
            (!checkShape || live.body->shape == (body.changed ? body.generated.get() : body.original.get()));
    }
    bool pose(const Body& body, const Source& source, havok_compound_shape_builder::ChildTransform& out) const
    {
        RE::NiTransform local;
        if (!tryResolveDescendantLocalTransform(root.get(), source.node.get(), local)) return false;
        local = loose_weapon_collision_policy::partInShapeBody(body.rootInBody, local, source.center);
        if (!weaponTransformFinite(local)) return false;
        out = {};
        out.column0 = {local.rotate.entry[0][0],local.rotate.entry[0][1],local.rotate.entry[0][2],0};
        out.column1 = {local.rotate.entry[1][0],local.rotate.entry[1][1],local.rotate.entry[1][2],0};
        out.column2 = {local.rotate.entry[2][0],local.rotate.entry[2][1],local.rotate.entry[2][2],0};
        out.translation = {local.translate.x*gameToHavokScale(),local.translate.y*gameToHavokScale(),local.translate.z*gameToHavokScale(),1};
        // Source scale is already baked into the generated convex geometry.
        return true;
    }
    bool captureBodies(const PhysicsFrameContext& frame)
    {
        if (!scanned) {
            phase="Preparing: native body scan";
            havok_world_lock::ScopedWorldReadLock lock(world);
            const auto step = object_physics_body_set::advanceObjectPhysicsBodyScanCache(world, scanOptions,
                {.maxVisitedNodes=2048, .maxCollisionObjects=128, .maxBodyIds=512}, scanCursor, scanCache);
            if (step.invalidated) { failed = true; return false; }
            if (!step.finished) { phase="Native pickup scan exceeded budget"; failed=true; return false; }
            const auto set = object_physics_body_set::buildObjectPhysicsBodySetFromScanCache(frame.bhkWorld, world, ref.get(), scanOptions, scanCache);
            const auto& d = set.diagnostics;
            if(set.records.empty()||d.unresolvedRefBodySkips) {phase="Waiting for native model physics publication";return false;}
            if (set.records.empty() || set.records.size()>loose_weapon_collision_policy::kMaxNativeBodies || d.scanFailures || d.invalidPhysicsSystems || d.foreignRefBodySkips ||
                d.unresolvedRefBodySkips || d.depthLimitSkips || d.staleCacheEntrySkips) { failed = true; return false; }
            bodies.reserve(set.records.size());
            for (const auto& record : set.records) {
                const auto live = havok_runtime::snapshotBody(world, RE::hknpBodyId{record.bodyId});
                if ((!record.accepted && record.motionType != Motion::Static && !((nativeOnly||loose_weapon_state::owns(ref->formID)) && record.motionType==Motion::Keyframed)) || !live.valid || !live.body || !live.body->shape ||
                    !live.ownerNode || !live.collisionObject || record.resolvedRef != ref.get()) { failed=true; return false; }
                RE::NiTransform bodyWorld;
                // A multipart weapon can have many differently oriented BODY
                // frames sharing one MOTION/COM. Shapes belong to the individual
                // BODY frame; the generic live resolver deliberately chooses COM.
                if (!havok_runtime::tryGetBodyArrayWorldTransform(world, RE::hknpBodyId{record.bodyId}, bodyWorld)) { failed=true; return false; }
                RE::NiTransform presented;
                const bool hasPresentation=held_scene_presentation::tryGetPresentedBodyWorld(world, record.bodyId, runtime_state::currentFrame().frameIndex, presented);
                if (hasPresentation) bodyWorld=presented;
                Body body;
                body.id=record.bodyId; body.node.reset(live.ownerNode); body.collision.reset(live.collisionObject);
                havok_ref_count::addRef(live.body->shape);
                body.original.reset(const_cast<RE::hknpShape*>(live.body->shape));
                body.rootInBody=transform_math::composeTransforms(transform_math::invertTransform(bodyWorld),root->world);
                bodies.push_back(std::move(body));
            }
            ROCK_LOG_INFO(Weapon,"Loose collision native scan ref={:08X} nodes={} collisionObjects={} bodies={} complete=true",
                ref->formID,d.visitedNodes,d.collisionObjects,bodies.size());
            scanned=true;
            if(loose_weapon_state::owns(ref->formID)) {
                frozen=std::all_of(bodies.begin(),bodies.end(),[&](const auto& b){
                    const auto live=havok_runtime::snapshotBody(world,RE::hknpBodyId{b.id});
                    return live.valid&&live.body&&physics_body_classifier::motionTypeFromBodyFlags(live.body->flags)!=Motion::Dynamic;
                });
            }
        }
        return true;
    }
    bool suppressNative()
    {
        phase="Preparing: native collision suppressed";
        WriteScope lock(world);
        for (auto& body:bodies) {
            if (!current(body)) return false;
            const auto lease=collision_suppression_registry::globalCollisionSuppressionRegistry().acquire(
                world,body.id,kOwner,"loose-pickup-pending");
            body.suppressed=lease.leaseIdentityValid && !lease.readFailed;
            const auto live=havok_runtime::snapshotBody(world,RE::hknpBodyId{body.id});
            if (!body.suppressed || !live.valid ||
                !(live.collisionFilterInfo & collision_suppression_registry::kSuppressionNoCollideBit)) return false;
        }
        return true;
    }
    bool refreshSuppression()
    {
        WriteScope lock(world);
        for (const auto& body:bodies) if (body.suppressed) {
            if (!current(body)) return false;
            const auto lease=collision_suppression_registry::globalCollisionSuppressionRegistry().refresh(
                world,body.id,kOwner,"loose-pickup-pending");
            if (lease.readFailed || !lease.leaseIdentityValid) return false;
        }
        return true;
    }
    bool prepare(const PhysicsFrameContext&)
    {
        // The transition's root cull is not an absent part. Child culls still
        // exclude missing magazines and payload during geometry extraction.
        struct RootVisibility {
            RE::NiAVObject* root;bool hidden,changed;
            RootVisibility(RE::NiAVObject* r,bool stateful):root(r),hidden(r->GetAppCulled()),changed(stateful&&hidden){if(changed)equipped_weapon_visual_state::setLocallyVisible(root,true);}
            ~RootVisibility(){if(changed)equipped_weapon_visual_state::setLocallyVisible(root,!hidden);}
        } visibility(root.get(),loose_weapon_state::owns(ref->formID));
        if (!builder) {
            builder=std::make_unique<WeaponCollision>();
            builder->_sources.preparation = std::make_unique<WeaponCollision::PendingSourcePreparation>();
            auto& prep = *builder->_sources.preparation;
            prep.root = root; prep.equippedKey = ref->formID;
            prep.task = builder->prepareGeneratedWeaponShapeSources(root, ref->formID, prep.sources,
                g_rockConfig.rockWeaponCollisionPreserveGaps, true);
        }
        auto& prep=*builder->_sources.preparation;
        phase="Preparing: mesh extraction";
        if (!prep.ready) {
            if (!builder->advanceSourcePreparation(loose_weapon_collision_policy::kPreparationSliceMilliseconds) && !builder->_sources.preparation) failed=true;
            return false; // Extraction and native hull building never share a slice.
        }
        if (prep.sources.empty()) { failed=true; return false; }
        if (sources.capacity()<prep.sources.size()) sources.reserve(prep.sources.size());
        phase="Preparing: generated hulls";
        if (nextSource<prep.sources.size()) {
            const auto& input=prep.sources[nextSource];
            Shape shape(WeaponCollision::buildGeneratedSourceShape(input));
            if (!shape) { failed=true; return false; }
            sources.push_back({std::move(shape),RE::NiPointer<RE::NiAVObject>{input.sourceRoot},input.sourceLocalCenterGame});
            ++nextSource;
            return false; // One source (at most eight convex children) per frame.
        }
        phase="Preparing: body compounds";
        // Assign each mesh to its closest native collision ancestor. Remaining
        // meshes belong to the shallowest collision owner in the same reference.
        // Empty auxiliary bodies retain their native identity but lose contacts
        // through an owned suppression lease until the original set is restored.
        if (nextMapping<sources.size()) {
            if (!nextMapping) {
                unsigned primaryDepth=UINT_MAX;
                for (std::size_t i=0;i<bodies.size();++i) {
                    auto* node=bodies[i].node.get(); unsigned depth=0;
                    while (node && node!=root.get() && depth<64) { node=node->parent; ++depth; }
                    if (node==root.get() && depth<primaryDepth) { primaryBody=i; primaryDepth=depth; }
                }
                if (primaryDepth==UINT_MAX) { failed=true; return false; }
            }
            const auto end=(std::min)(sources.size(),nextMapping+8);
            for (;nextMapping<end;++nextMapping) {
                auto selected=primaryBody; auto* node=sources[nextMapping].node.get(); bool found=false;
                for (unsigned depth=0;node && depth<64 && !found;++depth,node=node->parent)
                    for (std::size_t j=0;j<bodies.size();++j) if (node==bodies[j].node.get()) { selected=j; found=true; break; }
                bodies[selected].sources.push_back(nextMapping);
            }
            return false;
        }
        while (nextCompound<bodies.size()) {
            auto& body=bodies[nextCompound++];
            if (body.sources.empty()) continue;
            std::vector<havok_compound_shape_builder::CompoundChild> children;
            body.poses.resize(body.sources.size()); children.reserve(body.sources.size());
            for (std::size_t i=0;i<body.sources.size();++i) {
                const auto& source=sources[body.sources[i]];
                if (!pose(body,source,body.poses[i])) { failed=true; return false; }
                children.push_back({source.shape.get(),body.poses[i]});
            }
            if (!body.generated.create(children)) { failed=true; return false; }
            return false; // Bound compound construction independently of extraction.
        }
        builder.reset();
        return true;
    }
    bool install()
    {
        phase=nativeOnly ? "Watching native dropped bodies" : "Installing generated shapes";
        if(!nativeOnly&&loose_weapon_state::owns(ref->formID)&&!suppressNative())return false;
        WriteScope lock(world);
        for (const auto& body:bodies) if (!current(body)) return false;
        if (nativeOnly) {
            for (auto& body:bodies) {
                body.flagLease=havok_runtime::acquireBodyFlagLease(world,body.id,kContactFlags,kContactFlagMode,reinterpret_cast<std::uintptr_t>(this));
                if (!body.flagLease) return false;
                // A very short grab can release a previously sleeping object.
                // Wake once so its supported-rest witness is refreshed.
                (void)physics_recursive_wrappers::activateBody(world,body.id);
            }
            installed=true;
            return true;
        }
        for (auto& body:bodies) {
            if (body.generated) {
                shapeSetter()(world,RE::hknpBodyId{body.id},body.generated.get());
                body.changed=true;
                if (!current(body)) return false;
                body.flagLease=havok_runtime::acquireBodyFlagLease(world,body.id,kContactFlags,kContactFlagMode,reinterpret_cast<std::uintptr_t>(this));
                if (!body.flagLease) return false;
            }
        }
        // Publish collision only after every replacement shape is installed.
        // Unmapped native bodies remain suppressed until final restoration.
        for (auto& body:bodies) if (body.changed && body.suppressed) {
            const auto lease=collision_suppression_registry::globalCollisionSuppressionRegistry().release(
                world,body.id,kOwner,"loose-generated-ready");
            if (lease.readFailed) return false;
            body.suppressed=false;
        }
        // Read the final native state, not merely the requested filter value.
        // These per-body receipts distinguish missing discovery from a failed
        // shape replacement or a suppression write that did not stick.
        for (const auto& body:bodies) {
            const auto live=havok_runtime::snapshotBody(world,RE::hknpBodyId{body.id});
            const bool verified=live.valid && live.body && current(body) && (!body.suppressed ||
                (live.collisionFilterInfo & collision_suppression_registry::kSuppressionNoCollideBit)) &&
                (!body.flagLease || (live.body->flags & kContactFlags)==kContactFlags);
            if (!verified) { phase="Native replacement readback failed"; return false; }
        }
        installed=true;
        ROCK_LOG_INFO(Weapon,"Loose collision installed ref={:08X} nativeBodies={} generatedParts={} owner=native-body preparationFrames={} maxPreparationSliceMs={:.3f}",
            ref->formID,bodies.size(),sources.size(),preparationFrames,maxPreparationSliceMs);
        return true;
    }
    bool restore()
    {
        WriteScope lock(world);
        bool restored=true;
        for (auto& body:bodies) {
            // Recycled IDs and replaced scenes never authorize writes.
            if (!current(body,false)) { body.changed=false; body.suppressed=false; body.flagLease=false; continue; }
            if (body.changed) {
                if (!current(body)) { restored=false; continue; }
                shapeSetter()(world,RE::hknpBodyId{body.id},body.original.get());
                const auto live=havok_runtime::snapshotBody(world,RE::hknpBodyId{body.id});
                if (!live.valid || !live.body || live.body->shape!=body.original.get()) { restored=false; continue; }
                body.changed=false;
            }
            if (body.suppressed) {
                const auto result=collision_suppression_registry::globalCollisionSuppressionRegistry().release(world,body.id,kOwner,"loose-generated-restore");
                if (result.readFailed) { restored=false; continue; }
                body.suppressed=false;
            }
            if (body.flagLease) {
                if (!havok_runtime::releaseBodyFlagLease(world,body.id,kContactFlags,kContactFlagMode,reinterpret_cast<std::uintptr_t>(this))) { restored=false; continue; }
                body.flagLease=false;
            }
        }
        return restored;
    }
    bool animate()
    {
        WriteScope lock(world);
        for (auto& body:bodies) {
            if (!current(body)) return false;
            if (body.suppressed) {
                const auto lease=collision_suppression_registry::globalCollisionSuppressionRegistry().refresh(world,body.id,kOwner,"loose-generated-auxiliary");
                if (lease.readFailed || !lease.leaseIdentityValid) return false;
            }
            if (nativeOnly || !body.generated) continue;
            for (std::size_t i=0;i<body.sources.size();++i) if (!pose(body,sources[body.sources[i]],body.poses[i])) return false;
            if (!body.generated.updateTransforms(body.poses).succeeded) return false;
        }
        return true;
    }
};

LooseWeaponCollision::LooseWeaponCollision() = default;
LooseWeaponCollision::~LooseWeaponCollision() = default;
LooseWeaponCollision::Slot::Slot() = default;
LooseWeaponCollision::Slot::~Slot() = default;

void LooseWeaponCollision::publishWatchMask() noexcept
{
    std::uint32_t mask{};
    for (std::size_t i=0;i<_slots.size();++i) if (_slots[i]._count.load()) mask|=1u<<i;
    _watchMask.store(mask,std::memory_order_release);
}

void LooseWeaponCollision::clear(RE::hknpWorld* liveWorld)
{
    _watchMask.store(0,std::memory_order_release);
    for (auto& slot:_slots) { slot.clear(liveWorld); slot._declined=slot._rested=0; }
    for(auto& state:_parked)if(state) {
        if(liveWorld&&state->world==liveWorld&&!state->restore())continue;
        state.reset();
    }
    _nextPreparationSlot=0;
}

void LooseWeaponCollision::update(const PhysicsFrameContext& frame, RE::TESObjectREFR* right,
    RE::TESObjectREFR* left, PhysicsCallbackQuiescenceGate& gate,bool admitWeapons)
{
    if (!frame.worldReady || !frame.hknpWorld || frame.menuBlocked) return;
    performance_profiler::ScopedTimer timer(performance_profiler::Scope::LooseWeaponCollision);
    loose_weapon_state::restoreWithdrawn();
    // Retained parked shapes follow their live reference/model identity only.
    // Regrab reuses the same prepared body set rather than briefly restoring
    // an absent magazine's original collision.
    for(auto& parked:_parked)if(parked) {
        const bool current=parked->world==frame.hknpWorld&&parked->ref->Get3D()==parked->root.get()&&
            !parked->ref->IsDeleted()&&!parked->ref->IsDisabled()&&loose_weapon_state::owns(parked->ref->formID);
        if(!current) {
            auto mutation=gate.pauseForMutation();
            if(parked->world!=frame.hknpWorld||parked->restore())parked.reset();
        }else if(parked->ref.get()==right||parked->ref.get()==left) {
            auto slot=std::find_if(_slots.begin(),_slots.end(),[](const auto& s){return !s._state;});
            if(slot!=_slots.end()) {
                auto mutation=gate.pauseForMutation();
                parked->parked=parked->frozen=false;parked->parking={};parked->parkingStage=0;
                slot->_state=std::move(parked);slot->publishBodies();
            }
        }
    }
    auto preparing=_slots.size();
    for (std::size_t step=0;step<_slots.size();++step) {
        const auto i=(_nextPreparationSlot+step)%_slots.size();
        const auto& state=_slots[i]._state;
        if (state && !state->installed && !state->failed && !state->parked) {
            preparing=i; _nextPreparationSlot=(i+1)%_slots.size(); break;
        }
    }
    for (std::size_t i=0;i<_slots.size();++i) {
        auto& slot=_slots[i];
        try {
            // A quick-drop native watcher hands back to fresh shape preparation
            // on regrab. Never keep a second owner for the same reference.
            if (slot._state && !loose_weapon_state::owns(slot._state->ref->formID) && (slot._state->frozen || (admitWeapons && slot._state->nativeOnly)) &&
                (slot._state->ref.get()==right || slot._state->ref.get()==left)) {
                auto mutation=gate.pauseForMutation(); slot.clear(frame.hknpWorld);
            }
            slot.update(frame,right,left,gate,i==preparing);
            if (slot._state && slot._state->parked) {
                if(loose_weapon_state::owns(slot._state->ref->formID)&&!slot._state->nativeOnly) {
                    auto free=std::find_if(_parked.begin(),_parked.end(),[](const auto& s){return !s;});
                    if(free!=_parked.end()) {
                        slot._rested=slot._state->ref->formID;
                        slot._world.store(nullptr);slot._count.store(0);slot._generation.fetch_add(1);
                        *free=std::move(slot._state);
                    }
                    continue;
                }
                if(!admitWeapons)slot._rested=slot._state->ref->formID;
                auto mutation=gate.pauseForMutation(); slot.clear(frame.hknpWorld);
            }
        } catch (...) {
            auto mutation=gate.pauseForMutation();
            if (slot._state) slot._declined=slot._state->ref->formID;
            slot.clear(frame.hknpWorld);
            ROCK_LOG_SAMPLE_WARN(Weapon,2000,"Loose weapon collision update failed; native restoration requested");
        }
    }
    std::array<std::uint32_t,256> requested{};
    const auto requestCount=admitWeapons?loose_weapon_state::references(requested):0;
    for(std::size_t candidate=0;admitWeapons&&candidate<requestCount+2;++candidate) {
        auto* ref=candidate==0?right:candidate==1?left:RE::TESForm::GetFormByID<RE::TESObjectREFR>(requested[candidate-2]);
        auto* form=ref ? ref->GetObjectReference() : nullptr;
        auto* weapon=form ? form->As<RE::TESObjectWEAP>() : nullptr;
        if (!weapon || !loose_weapon_collision_policy::eligibleType(weapon->weaponData.type.get()) ||
            ref->IsDeleted() || ref->IsDisabled() || !ref->Get3D()) continue;
        if(candidate>=2) {
            auto mutation=gate.pauseForMutation();
            const bool wasReady=loose_weapon_state::owns(ref->formID);
            // Revisit pending descriptions before the existing-slot shortcut:
            // the engine can finish attachments on the same dummy root later.
            if(!loose_weapon_state::apply(ref,ref->Get3D()))continue;
            if(!wasReady) {
                for(auto& slot:_slots)if(slot._state&&slot._state->ref.get()==ref)slot.clear(frame.hknpWorld);
            }
        }
        if(std::any_of(_parked.begin(),_parked.end(),[&](const auto& s){return s&&s->ref.get()==ref;}))continue;
        if (std::any_of(_slots.begin(),_slots.end(),[&](const Slot& slot) {
            return (slot._state && slot._state->ref.get()==ref) || slot._declined==ref->formID;
        })) continue;
        std::array<loose_weapon_collision_policy::Slot,loose_weapon_collision_policy::kMaxWeapons> identities{};
        for (std::size_t i=0;i<_slots.size();++i)
            if (_slots[i]._state) identities[i].reference=_slots[i]._state->ref->formID;
        const auto admissions=loose_weapon_collision_policy::admissions(identities,{ref->formID,0});
        const auto found=std::find(admissions.begin(),admissions.end(),ref->formID);
        if (found==admissions.end()) {
            ROCK_LOG_SAMPLE_WARN(Weapon,2000,"Loose collision capacity reached ref={:08X}; native collision retained",ref->formID);
            continue;
        }
        auto free=_slots.begin()+static_cast<std::size_t>(found-admissions.begin());
        auto mutation=gate.pauseForMutation();
        try {
            if(loose_weapon_state::owns(ref->formID)&&!loose_weapon_state::apply(ref,ref->Get3D()))continue;
            free->admit(ref,frame);
        }
        catch (...) {
            free->_declined=ref->formID; free->clear(frame.hknpWorld);
            ROCK_LOG_SAMPLE_WARN(Weapon,2000,"Loose collision preparation failed ref={:08X}; native restoration requested",ref->formID);
        }
    }
    publishWatchMask();
}

bool LooseWeaponCollision::takeNativeRestCompletion(std::uint32_t reference) noexcept
{
    for(auto& slot:_slots)if(reference&&!slot._state&&slot._rested==reference){slot._rested=0;return true;}
    return false;
}
bool LooseWeaponCollision::presentationReady(std::uint32_t reference) const {
    if(!loose_weapon_state::owns(reference))return true;
    // A reported preparation failure must never strand a native drop/grab.
    // Slot::clear has restored its original physics before recording decline.
    for(const auto& slot:_slots)if(slot._declined==reference)return true;
    for(const auto& slot:_slots)if(slot._state&&slot._state->ref->formID==reference)
        return slot._state->installed&&!slot._state->failed;
    return std::any_of(_parked.begin(),_parked.end(),[&](const auto& state){return state&&state->ref->formID==reference;});
}

LooseWeaponCollision::NativeRest LooseWeaponCollision::watchNative(RE::TESObjectREFR* ref,
    const PhysicsFrameContext& frame,PhysicsCallbackQuiescenceGate& gate)
{
    if(!ref || !frame.worldReady || frame.menuBlocked || !frame.hknpWorld || !ref->Get3D() ||
       ref->IsDeleted() || ref->IsDisabled() || !native_object_placement::available())return NativeRest::Unavailable;
    if(std::any_of(_parked.begin(),_parked.end(),[&](const auto& state){return state&&state->ref.get()==ref;}))return NativeRest::Resting;
    for(auto& slot:_slots) {
        if(slot._state && slot._state->ref.get()==ref)return NativeRest::Watching;
        if(slot._rested==ref->formID){slot._rested=0;return NativeRest::Resting;}
    }
    auto free=std::find_if(_slots.begin(),_slots.end(),[](const auto& slot){return !slot._state;});
    if(free==_slots.end())return NativeRest::CapacityFull;
    auto mutation=gate.pauseForMutation();
    free->_rested=0;free->_state=std::make_unique<State>(ref,frame.hknpWorld);
    auto& state=*free->_state;state.nativeOnly=!loose_weapon_state::owns(ref->formID);
    if(state.failed || !state.captureBodies(frame)){free->clear(frame.hknpWorld);return NativeRest::Unavailable;}
    bool dynamic=false,valid=true;
    {
        havok_world_lock::ScopedWorldReadLock lock(frame.hknpWorld);
        for(const auto& body:state.bodies) {
            const auto current=havok_runtime::snapshotBody(frame.hknpWorld,RE::hknpBodyId{body.id});
            if(!current.valid || !current.body){valid=false;break;}
            dynamic=dynamic || physics_body_classifier::motionTypeFromBodyFlags(current.body->flags)==Motion::Dynamic;
        }
    }
    if(!valid){free->clear(frame.hknpWorld);return NativeRest::Unavailable;}
    // A native saved keyframed object needs neither reactivation nor another
    // contact watcher. This is also the save/load restoration path.
    if(!dynamic&&state.nativeOnly){free->clear(frame.hknpWorld);return NativeRest::Resting;}
    if(state.nativeOnly&&!state.install()){free->clear(frame.hknpWorld);return NativeRest::Unavailable;}
    free->publishBodies();publishWatchMask();return NativeRest::Watching;
}

bool LooseWeaponCollision::prepareRelease(RE::TESObjectREFR* reference, RE::hknpWorld* liveWorld,
    PhysicsCallbackQuiescenceGate& gate, bool physicalDrop)
{
    if (!physicalDrop) return releaseReference(reference,liveWorld,gate);
    for (auto& slot:_slots) {
        if (!slot._state || slot._state->ref.get()!=reference) continue;
        auto& state=*slot._state;
        if (state.world!=liveWorld) return false;
        if (state.installed) return !state.failed;
        if(loose_weapon_state::owns(reference->formID))return !state.failed;
        auto mutation=gate.pauseForMutation();
        if (!state.restore()) return false;
        state.nativeOnly=true;
        state.builder.reset();
        if (!state.install()) { state.failed=true; return false; }
        slot.publishBodies();
        publishWatchMask();
        ROCK_LOG_INFO(Weapon,"Loose collision quick release ref={:08X}; native collision restored, supported-rest watch retained",reference->formID);
    }
    return true;
}

bool LooseWeaponCollision::watches(std::uint32_t body) const noexcept
{
    auto mask=_watchMask.load(std::memory_order_acquire);
    while (mask) {
        const auto index=std::countr_zero(mask); mask&=mask-1;
        if (_slots[index].watches(body)) return true;
    }
    return false;
}

void LooseWeaponCollision::contact(RE::hknpWorld* world,std::uint32_t a,std::uint32_t b,
    const RE::NiPoint3& point,const RE::NiPoint3& normal) noexcept
{
    auto mask=_watchMask.load(std::memory_order_acquire);
    while (mask) {
        const auto index=std::countr_zero(mask); mask&=mask-1;
        _slots[index].contact(world,a,b,point,normal);
    }
}

void LooseWeaponCollision::Slot::publishBodies()
{
    std::array<std::uint32_t,loose_weapon_collision_policy::kMaxNativeBodies> ids{};
    const auto count=_state->bodies.size();
    for (std::size_t i=0;i<count;++i) ids[i]=_state->bodies[i].id;
    std::sort(ids.begin(),ids.begin()+count);
    for (std::size_t i=0;i<count;++i) _bodies[i].store(ids[i],std::memory_order_relaxed);
    _world.store(_state->world,std::memory_order_release);
    _count.store(static_cast<unsigned>(count),std::memory_order_release);
}

void LooseWeaponCollision::Slot::admit(RE::TESObjectREFR* reference,const PhysicsFrameContext& frame)
{
    const auto started=std::chrono::steady_clock::now();
    if (!shapeSetter()) { _declined=reference->formID; ROCK_LOG_ERROR(Weapon,"Loose collision native shape setter unavailable"); return; }
    _state=std::make_unique<State>(reference,frame.hknpWorld);
    const bool captured=!_state->failed&&_state->captureBodies(frame);
    if(!captured&&!_state->failed){_state.reset();return;}
    if (_state->failed || !captured ||
        (!loose_weapon_state::owns(reference->formID)&&!_state->suppressNative())) {
        ROCK_LOG_WARN(Weapon,"Loose pickup collision suppression failed ref={:08X} stage={}; restoring native collision",reference->formID,_state->phase);
        clear(frame.hknpWorld); _declined=reference->formID; return;
    }
    _state->phase="Held: native suppressed, preparing generated shapes";
    ROCK_LOG_INFO(Weapon,"Loose collision pickup ref={:08X} native collision suppressed bodies={} geometryQueued=true acquisitionMs={:.3f}",
        reference->formID,_state->bodies.size(),
        std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count());
}

bool LooseWeaponCollision::releaseReference(RE::TESObjectREFR* reference, RE::hknpWorld* liveWorld, PhysicsCallbackQuiescenceGate& gate)
{
    auto mutation = gate.pauseForMutation();
    bool restored = true;
    const auto release = [&](auto& slots) {
        for (auto& slot : slots) {
            if(reference && slot._rested==reference->formID)slot._rested=0;
            if (slot._state && slot._state->ref.get() == reference) {
                slot.clear(liveWorld);
                restored = restored && !slot._state;
            }
        }
    };
    release(_slots);
    for(auto& state:_parked)if(state&&state->ref.get()==reference) {
        if(state->world!=liveWorld||state->restore())state.reset();else restored=false;
    }
    publishWatchMask();
    return restored;
}

void LooseWeaponCollision::Slot::clear(RE::hknpWorld* liveWorld)
{
    // Close admission before advancing the generation. A callback must never
    // label the outgoing body's witness with the next owner's generation.
    _world.store(nullptr); _count.store(0);
    _generation.fetch_add(1,std::memory_order_acq_rel);
    if (_state && liveWorld && _state->world==liveWorld && !_state->restore()) {
        _state->failed=true; _state->phase="Restore pending";
        ROCK_LOG_SAMPLE_WARN(Weapon,2000,"Loose collision restoration pending ref={:08X}; shape ownership retained",_state->ref->formID);
        return;
    }
    const auto retiredRef=_state ? _state->ref->formID : 0;
    _state.reset();
    if (retiredRef) ROCK_LOG_INFO(Weapon,"Loose collision clear complete ref={:08X}",retiredRef);
    if (!_supportBusy.test_and_set(std::memory_order_acquire)) { _support={}; _supportBusy.clear(std::memory_order_release); }
}

void LooseWeaponCollision::Slot::update(const PhysicsFrameContext& frame, RE::TESObjectREFR* right,
    RE::TESObjectREFR* left, PhysicsCallbackQuiescenceGate& gate, bool prepare)
{
    if ((!right || right->formID!=_declined) && (!left || left->formID!=_declined) && !loose_weapon_state::owns(_declined)) _declined=0;
    if (!_state) return;
    const auto reset=[&](RE::hknpWorld* live) { auto mutation=gate.pauseForMutation(); clear(live); };
    if (_state && (_state->world!=frame.hknpWorld || _state->ref->IsDeleted() || _state->ref->IsDisabled() || _state->ref->Get3D()!=_state->root.get()))
        reset(_state->world==frame.hknpWorld ? frame.hknpWorld : nullptr);
    if (_state && _state->failed) {
        const auto id=_state->ref->formID;
        ROCK_LOG_SAMPLE_WARN(Weapon,1000,"Loose collision declined ref={:08X} stage={} bodies={} shapes={}; restoring native collision",
            id,_state->phase,_state->bodies.size(),_state->sources.size());
        reset(frame.hknpWorld); _declined=id; return;
    }
    if (!_state) return;
    auto& state=*_state;
    if (state.parked) return;
    state.held=right==state.ref.get() || left==state.ref.get();
    if (!state.installed) {
        if (!state.held && !state.nativeOnly && !loose_weapon_state::owns(state.ref->formID)) {
            // A quick release must regain collision before it can fall. Keep
            // this exact reference's rest watcher even if geometry is unfinished.
            auto mutation=gate.pauseForMutation();
            if (!state.restore()) { state.failed=true; return; }
            state.nativeOnly=true;
            state.builder.reset();
        }
        if (!state.nativeOnly) {
            auto mutation=gate.pauseForMutation();
            if (!state.refreshSuppression()) { state.failed=true; return; }
        }
        if (!prepare) return;
        if (state.failed) return;
        const auto started=std::chrono::steady_clock::now();
        const bool prepared=state.nativeOnly ? state.captureBodies(frame) : state.prepare(frame);
        ++state.preparationFrames;
        state.maxPreparationSliceMs=(std::max)(state.maxPreparationSliceMs,
            std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count());
        if (!prepared) return;
    }
    // Geometry preparation above yields within its existing work budget and
    // never pauses physics callbacks. Only live native mutations need the gate.
    {
        auto mutation=gate.pauseForMutation();
        if (!state.installed) {
            if (!state.install()) { state.failed=true; return; }
            publishBodies();
        }
        if (!state.animate()) { state.phase="Animated shape or native identity changed"; state.failed=true; return; }
    }
    Support support;
    if (!_supportBusy.test_and_set(std::memory_order_acquire)) { support=_support; if (state.held) _support={}; _supportBusy.clear(std::memory_order_release); }
    if (state.held) {
        state.parking.update(true,false,false,frame.deltaSeconds);
        state.phase="Generated: held";
        return; // Held weapons need no second body scan or rest-pose math.
    }
    state.supported=false; state.stationary=true;
    bool validRestPoses=true;
    {
        havok_world_lock::ScopedWorldReadLock lock(frame.hknpWorld);
        for (auto& body:state.bodies) {
            const auto live=havok_runtime::snapshotBody(frame.hknpWorld,RE::hknpBodyId{body.id});
            if (live.valid && live.body && physics_body_classifier::motionTypeFromBodyFlags(live.body->flags)==Motion::Static) continue;
            if (!live.valid || !live.body || !live.motion) { validRestPoses=false; break; }
            const auto pose=havok_runtime::bodyArrayWorldTransform(*live.body);
            // Anchor the entire interval. Comparing only adjacent frames lets
            // a slowly sliding weapon accumulate a full interval of false rest.
            // Instantaneous solver velocity is not a rest veto: ground jitter
            // may have a high short-lived speed but stay within this envelope.
            if (state.parking.supportedRestSeconds==0) body.restPose=pose;
            const auto delta=loose_weapon_collision_policy::poseDelta(pose,body.restPose,gameToHavokScale());
            if (!delta.valid) { validRestPoses=false; break; }
            state.stationary=state.stationary && delta.withinRestTolerance();
        }
        state.stationary=state.stationary && validRestPoses;
        if (!state.held && support.valid && support.generation==_generation.load(std::memory_order_acquire) && watches(support.weaponBody)) {
            const auto surface=havok_runtime::snapshotBody(frame.hknpWorld,RE::hknpBodyId{support.surfaceBody});
            RE::NiTransform weaponPose,surfacePose;
            state.supported=parkingSurface(surface) &&
                reinterpret_cast<std::uintptr_t>(surface.collisionObject)==support.surfaceCollision &&
                havok_runtime::tryGetBodyArrayWorldTransform(frame.hknpWorld,RE::hknpBodyId{support.weaponBody},weaponPose) &&
                havok_runtime::tryGetBodyArrayWorldTransform(frame.hknpWorld,RE::hknpBodyId{support.surfaceBody},surfacePose) &&
                loose_weapon_collision_policy::poseDelta(weaponPose,support.weaponPose,gameToHavokScale()).withinRestTolerance() &&
                sameSurfacePose(surfacePose,support.surfacePose);
        }
    }
    const bool park=state.parking.update(state.held,state.supported,state.stationary,frame.deltaSeconds);
    state.phase=state.nativeOnly ? (state.held ? "Dropped: held" : !state.supported ? "Dropped: no support" :
        state.stationary ? "Dropped: resting" : "Dropped: moving") : state.held ? "Generated: held" : !state.supported ? "Generated: no support" :
        state.stationary ? "Generated: resting" : "Generated: moving";
    if(state.nativeOnly&&!state.held&&!state.reportedUnsettled) {
        if(std::isfinite(frame.deltaSeconds)&&frame.deltaSeconds>0)state.unsettledSeconds+=std::min(frame.deltaSeconds,.1f);
        if(state.unsettledSeconds>=5.f&&!park) {
            state.reportedUnsettled=true;
            ROCK_LOG_WARN(Weapon,"Native retirement still waiting ref={:08X} phase={} supported={} stationary={} witness={} surface={} restSeconds={:.3f}; object remains native dynamic",
                state.ref->formID,state.phase,state.supported,state.stationary,support.valid,support.surfaceBody,state.parking.supportedRestSeconds);
        }
    }
    if (!park && !state.frozen) return;
    if (state.parkingStage==0) {
        state.parkingStage=1;
        ROCK_LOG_INFO(Weapon,"Loose collision parking begin ref={:08X} stage=script supported={} stationary={} rest={:.3f}",
            state.ref->formID,state.supported,state.stationary,state.parking.supportedRestSeconds);
    }
    const auto script=native_object_placement::prepareLoadScript(state.ref.get(),!state.scriptStarted);
    state.scriptStarted=true;
    if (script==native_object_placement::ScriptPreparation::Rejected) {
        state.failed=true; state.phase="Dropped: native retirement script rejected";
        return;
    }
    if (script!=native_object_placement::ScriptPreparation::Ready) return;
    const auto parkingStarted=std::chrono::steady_clock::now();
    auto mutation=gate.pauseForMutation();
    if (!state.frozen) {
        if (state.parkingStage!=2) {
            state.parkingStage=2;
            ROCK_LOG_INFO(Weapon,"Loose collision parking stage=native-anchor ref={:08X} bodies={}",state.ref->formID,state.bodies.size());
        }
        std::array<std::uint32_t,loose_weapon_collision_policy::kMaxNativeBodies> ids{};
        for (std::size_t i=0;i<state.bodies.size();++i) ids[i]=state.bodies[i].id;
        if (!native_object_placement::anchor(state.ref.get(),frame.hknpWorld,state.root->world,{ids.data(),state.bodies.size()})) {
            state.failed=true; state.phase="Native anchor rejected"; return;
        }
        state.frozen=true;
        state.parkingStage=3;
        ROCK_LOG_INFO(Weapon,"Loose collision parking stage=restore-shapes ref={:08X}",state.ref->formID);
    }
    if (!loose_weapon_state::owns(state.ref->formID) && !state.restore()) { state.phase="Parked: restore pending"; return; }
    _world.store(nullptr); _count.store(0);
    state.parked=true; state.phase=loose_weapon_state::owns(state.ref->formID)?"Parked: saved item collision":"Parked: native collision";
    state.parkingStage=4;
    ROCK_LOG_INFO(Weapon,"Loose collision parked ref={:08X} supportedRest={:.3f} save=native finalizeMs={:.3f}",
        state.ref->formID,state.parking.supportedRestSeconds,
        std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-parkingStarted).count());
}

bool LooseWeaponCollision::Slot::watches(std::uint32_t body) const noexcept
{
    unsigned first=0, last=_count.load(std::memory_order_acquire);
    while (first<last) {
        const auto middle=first+(last-first)/2;
        const auto id=_bodies[middle].load(std::memory_order_relaxed);
        if (id==body) return true;
        if (id<body) first=middle+1; else last=middle;
    }
    return false;
}

void LooseWeaponCollision::Slot::contact(RE::hknpWorld* world,std::uint32_t a,std::uint32_t b,
    const RE::NiPoint3& point,const RE::NiPoint3& normal) noexcept
{
    const auto generation=_generation.load(std::memory_order_acquire);
    if (_world.load()!=world) return;
    if (!std::isfinite(normal.z) || std::abs(normal.z)<0.5f) return;
    const bool mineA=watches(a),mineB=watches(b);
    if (mineA==mineB) return;
    Support witness;
    witness.generation=generation;
    witness.weaponBody=mineA ? a : b; witness.surfaceBody=mineA ? b : a;
    const auto surface=havok_runtime::snapshotBody(world,RE::hknpBodyId{witness.surfaceBody});
    if (!parkingSurface(surface)) return;
    witness.surfaceCollision=reinterpret_cast<std::uintptr_t>(surface.collisionObject);
    float comX{},comY{},comZ{};
    if (!havok_runtime::tryGetBodyArrayWorldTransform(world,RE::hknpBodyId{witness.weaponBody},witness.weaponPose) ||
        !havok_runtime::tryGetBodyArrayWorldTransform(world,RE::hknpBodyId{witness.surfaceBody},witness.surfacePose) ||
        !havok_runtime::getBodyCOMWorld(world,RE::hknpBodyId{witness.weaponBody},comX,comY,comZ)) return;
    if (!std::isfinite(point.z) || !std::isfinite(comZ) || point.z>comZ*havokToGameScale()+1.0f) return;
    witness.valid=true;
    if (!_supportBusy.test_and_set(std::memory_order_acquire)) {
        if (generation==_generation.load(std::memory_order_acquire)) _support=witness;
        _supportBusy.clear(std::memory_order_release);
    }
}

}
