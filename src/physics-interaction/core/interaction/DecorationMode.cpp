#include "physics-interaction/core/PhysicsInteractionInternal.h"
#include "physics-interaction/native/DecorationPlacement.h"
#include "physics-interaction/native/HavokRuntime.h"
#include "physics-interaction/native/HavokWorldLock.h"
#include "physics-interaction/object/ObjectPhysicsBodySet.h"
#include "physics-interaction/input/InputRemapRuntime.h"
#include "physics-interaction/core/RockRuntimeState.h"
#include "api/ProviderRuntimeServices.h"
#include "RE/Havok/hknpBody.h"

namespace rock {
std::uint32_t PhysicsInteraction::decorationCandidate(RE::hknpWorld* world) const
{
    if (!world || !g_rockConfig.rockDecorationMode || !decoration_placement::available()) return 0;
    const auto heldId=[](const Hand& hand) {
        return hand.isHolding() && hand.getHeldRef() ? hand.getHeldRef()->GetFormID() : 0u;
    };
    const auto id=decoration_mode::singleObject(heldId(_rightHand),heldId(_leftHand));
    if (!id) return 0;
    havok_world_lock::ScopedWorldReadLock lock(world);
    bool touching=false;
    for (const auto* hand : { &_rightHand, &_leftHand }) {
        if (!hand->isHolding()) continue;
        const auto& saved=hand->getSavedObjectState();
        if (!saved.isValid() || saved.refr->IsDeleted() || saved.refr->IsDisabled() ||
            saved.targetKind!=grab_target::Kind::LooseObject ||
            hand->getHeldBodyIds().empty() || hand->getHeldBodyIds().size()>64) return 0;
        const auto contact=hand->readDecorationSurfaceContact();
        if (!contact.recent || !hand->isHeldBodyId(contact.heldBodyId) ||
            _rightHand.isHeldBodyId(contact.otherBodyId) || _leftHand.isHeldBodyId(contact.otherBodyId)) continue;
        const auto other=havok_runtime::snapshotBodyIdentity(world,RE::hknpBodyId{contact.otherBodyId});
        if (!other.valid || !other.body || !other.body->shape) continue;
        const auto motion=physics_body_classifier::motionTypeFromBodyFlags(other.body->flags);
        touching |= motion==physics_body_classifier::BodyMotionType::Static ||
            motion==physics_body_classifier::BodyMotionType::Keyframed;
    }
    return touching ? id : 0;
}

void PhysicsInteraction::updateDecorationInput()
{
    const auto& frame=runtime_state::currentFrame();
    _decorationCandidate=0;
    const bool usable=isInitialized() && frame.visualAuthorityAvailable && frame.localSkeletonReady &&
        !frame.localMenuBlocking && !frame.compatibilityConfigBlocking &&
        !input_remap_runtime::isProviderOpenVrGameInputSuppressedForHand(false);
    if (!usable) { _decorationClick={}; _decorationPendingForm=0; return; }
    auto* bhk=getPlayerBhkWorld();
    auto* world=bhk ? getHknpWorld(bhk) : nullptr;
    if (bhk==_lifecycle.cachedBhkWorld && world==_lifecycle.cachedHknpWorld && physicsWritesAllowedForWorld(world))
        _decorationCandidate=decorationCandidate(world);
    // Bipod consumes unused edges when enabled. Otherwise decoration must
    // drain them itself so an old in-air click cannot anchor on later contact.
    const bool consume=_decorationCandidate || _decorationClick.draining ||
        (g_rockConfig.rockDecorationMode && !g_rockConfig.rockBipodMode);
    const auto raw=consume ?
        input_remap_runtime::consumeRawButtonState(false,decoration_mode::kButtonId) :
        input_remap_runtime::peekRawButtonState(false,decoration_mode::kButtonId);
    _decorationClick.update(g_rockConfig.rockDecorationMode,_decorationCandidate,
        raw.available,raw.held,raw.pressed,raw.sampleAgeMilliseconds);
    if (g_rockConfig.rockDecorationMode && (_rightHand.isHolding() || _leftHand.isHolding())) {
        const auto rightContact=_rightHand.readDecorationSurfaceContact();
        const auto leftContact=_leftHand.readDecorationSurfaceContact();
        if (raw.pressed) {
            ROCK_LOG_INFO(Hand,"Decoration click candidate={:08X} request={:08X} held={} available={} ageMs={} reserved={}",
                _decorationCandidate,_decorationClick.request,raw.held,raw.available,raw.sampleAgeMilliseconds,_decorationClick.reserved);
        }
        ROCK_LOG_SAMPLE_DEBUG(Hand,1000,
            "Decoration eligibility candidate={:08X} right[holding,legacyIncomplete,contact,other]=[{},{},{},{}] left=[{},{},{},{}] raw[available,held,ageMs]=[{},{},{}] source=manifold",
            _decorationCandidate,_rightHand.isHolding(),_rightHand.getActiveGrabLifecycle().hasIncompleteNativeScan(),
            rightContact.recent,rightContact.otherBodyId,
            _leftHand.isHolding(),_leftHand.getActiveGrabLifecycle().hasIncompleteNativeScan(),
            leftContact.recent,leftContact.otherBodyId,raw.available,raw.held,raw.sampleAgeMilliseconds);
    }
}

void PhysicsInteraction::queryDecorationState(api::input::v1_1::DecorationState& out) const
{
    using Flag=api::input::v1_1::DecorationFlag;
    out={};
    out.sample=provider::runtime::sample();
    if (g_rockConfig.rockDecorationMode) out.flags|=static_cast<std::uint32_t>(Flag::Enabled);
    if (_decorationClick.reserved) out.flags|=static_cast<std::uint32_t>(Flag::InputReserved);
    if (_decorationCandidate) out.flags|=static_cast<std::uint32_t>(Flag::Eligible);
    out.objectFormId=_decorationCandidate;
}

void PhysicsInteraction::commitDecoration(const PhysicsFrameContext& frame)
{
    const auto click=std::exchange(_decorationClick.request,0u);
    const bool starting=click!=0 && _decorationPendingForm==0;
    if (starting) { _decorationPendingForm=click; _decorationScriptWait=0.0f; }
    const auto requested=_decorationPendingForm;
    if (!requested) return;
    if (frame.menuBlocked || !frame.worldReady || !physicsWritesAllowedForWorld(frame.hknpWorld) ||
        requested!=decorationCandidate(frame.hknpWorld)) {
        ROCK_LOG_INFO(Hand,"Decoration rejected ref={:08X} stage=commit-eligibility menu={} worldReady={}",requested,frame.menuBlocked,frame.worldReady);
        _decorationPendingForm=0;
        return;
    }
    auto mutation=_generatedBodyStepDrive.callbackGate().pauseForMutation();
    auto& owner=_rightHand.isHolding() ? _rightHand : _leftHand;
    const auto retained=owner.getSavedObjectState().retainedRef;
    auto* ref=retained.get();
    auto* root=ref ? ref->Get3D() : nullptr;
    auto* cell=ref ? ref->GetParentCell() : nullptr;
    if (!root || !cell || cell->GetbhkWorld()!=frame.bhkWorld) {
        ROCK_LOG_WARN(Hand,"Decoration rejected ref={:08X} stage=reference-world",requested);
        _decorationPendingForm=0;
        return;
    }
    const auto prepared=decoration_placement::prepareLoadScript(ref,starting);
    if (prepared==decoration_placement::ScriptPreparation::Rejected) { _decorationPendingForm=0; return; }
    if (prepared==decoration_placement::ScriptPreparation::Pending) {
        _decorationScriptWait+=std::clamp(frame.deltaSeconds,0.0f,0.1f);
        if (_decorationScriptWait>=2.0f) {
            ROCK_LOG_WARN(Hand,"Decoration rejected ref={:08X} stage=load-script-timeout; grab retained",requested);
            _decorationPendingForm=0;
        }
        return;
    }
    _decorationPendingForm=0;
    const auto pose=root->world;
    std::array<std::uint32_t,decoration_mode::kMaxBodies> bodies{};
    std::size_t count=0;
    {
        // Cached grab prep deliberately leaves its lifecycle incomplete after
        // native activation. Re-enumerate the current tree once on placement,
        // without seeding a held body that could conceal missing discovery.
        using namespace object_physics_body_set;
        havok_world_lock::ScopedWorldReadLock lock(frame.hknpWorld);
        BodySetScanOptions options{};
        options.requireSameResolvedRef=true;
        ObjectPhysicsBodyScanCursor cursor;
        ObjectPhysicsBodyScanCache cache;
        if (!beginObjectPhysicsBodyScanCache(ref,options,cursor,cache)) {
            ROCK_LOG_WARN(Hand,"Decoration rejected ref={:08X} stage=body-scan-start",requested);
            return;
        }
        const auto step=advanceObjectPhysicsBodyScanCache(frame.hknpWorld,options,
            {256,64,static_cast<std::uint32_t>(bodies.size())},cursor,cache);
        const auto scanned=buildObjectPhysicsBodySetFromScanCache(frame.bhkWorld,frame.hknpWorld,ref,options,cache);
        const auto& diagnostics=scanned.diagnostics;
        const auto issues=diagnostics.scanFailures+diagnostics.invalidPhysicsSystems+
            diagnostics.benignScanSkips+diagnostics.foreignRefBodySkips+diagnostics.unresolvedRefBodySkips+
            diagnostics.weaponExpansionSkips+diagnostics.depthLimitSkips+diagnostics.staleCacheEntrySkips;
        const bool complete=decoration_mode::completeBodyScan(step.finished && !step.invalidated,
            scanned.records.size(),scanned.acceptedCount(),issues);
        ROCK_LOG_INFO(Hand,"Decoration body scan ref={:08X} complete={} finished={} bodies={} accepted={} issues={} nodes={} collisionObjects={} budgetExhausted={}",
            requested,complete,step.finished,scanned.records.size(),scanned.acceptedCount(),issues,
            diagnostics.visitedNodes,diagnostics.collisionObjects,step.budgetExhausted);
        if (!complete) return;
        for (const auto* hand : { &_rightHand, &_leftHand }) {
            if (!hand->isHolding()) continue;
            for (const auto id:hand->getHeldBodyIds()) {
                if (!scanned.containsAcceptedBody(id)) {
                    ROCK_LOG_WARN(Hand,"Decoration rejected ref={:08X} stage=held-body-missing body={}",requested,id);
                    return;
                }
            }
        }
        for (const auto& record:scanned.records) bodies[count++]=record.bodyId;
    }
    // Release both motors and their inertia/filter leases before freezing. A
    // native save never retains ROCK's hand constraint or a transient body ID.
    std::array<bool,2> released{};
    bool allReleased=true;
    for (const bool left : { false,true }) {
        auto& hand=left ? _leftHand : _rightHand;
        if (!hand.isHolding()) continue;
        auto context=makeGrabReleaseContext(hand,left);
        context.applyCapturedReleaseVelocity=false;
        context.reason="decoration-anchor";
        released[left ? 1u : 0u]=hand.releaseGrabbedObject(frame.hknpWorld,
            GrabReleaseCollisionRestoreMode::Delayed,context).released;
        if (hand.isHolding()) {
            ROCK_LOG_WARN(Hand,"Decoration rejected ref={:08X} stage=release hand={}",requested,left ? "left" : "right");
            allReleased=false;
            break;
        }
        releaseObject(ref,claimOwnerForHand(left));
        clearGameplayCandidatesForHand(hand,left);
        grab_input_intent_policy::reset(_grabInput.intentStates[left ? 1u : 0u]);
        publishHandInputOwnership(hand,left);
    }
    const bool anchored=allReleased && decoration_placement::anchor(ref,frame.hknpWorld,pose,
        std::span<const std::uint32_t>{bodies.data(),count});
    _decorationCandidate=0;
    ROCK_LOG_INFO(Hand,"Decoration placement ref={:08X} anchored={} bodies={} position=({:.3f},{:.3f},{:.3f}) save=native",
        requested,anchored,count,pose.translate.x,pose.translate.y,pose.translate.z);
    for (const bool left : { false,true }) {
        if (!released[left ? 1u : 0u]) continue;
        dispatchPhysicsMessage(kPhysMsg_OnRelease,left,ref,requested,0);
        dispatchSimpleGrabEvent(GrabEventType::Released,left,ref);
    }
}
}
