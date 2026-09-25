#include "physics-interaction/weapon/SecondaryEquippedWeapon.h"
#include "physics-interaction/core/PhysicsFrameContext.h"
#include "physics-interaction/core/RockRuntimeState.h"
#include "physics-interaction/visual/FrikHandWorldAuthority.h"
#include "physics-interaction/weapon/VanillaWeaponGripFrame.h"
#include "physics-interaction/input/InputRemapRuntime.h"
#include "physics-interaction/input/InputRemapPolicy.h"
#include "physics-interaction/TransformMath.h"
#include "RockConfig.h"
#include "rock_support/Fo4VrRuntime.h"

namespace rock
{
    SecondaryEquippedWeapon::SecondaryEquippedWeapon()
    {
        dynamic.setPhysicalSessionSlot(0);
        grip.setWeaponVisualIntentObserver(&dynamic, &DynamicWeaponCollisionRuntime::observeWeaponVisualIntent);
        grip.setSurfaceSupportRuntime(&dynamic);
    }

    void SecondaryEquippedWeapon::setPhysicsCallbackGate(PhysicsCallbackQuiescenceGate* gate)
    {
        _gate = gate;
        collision.setPhysicsCallbackGate(gate);
        dynamic.setPhysicsCallbackGate(gate);
    }

    void SecondaryEquippedWeapon::adopt(const Transfer& transfer)
    {
        _transfer = transfer;
        _pendingRelease = {};
        _transferredRelease = false;
    }

    void SecondaryEquippedWeapon::bridge(RE::NiPointer<RE::NiAVObject> model)
    {
        releaseBridge(true);
        if (!model || model->parent || !_transfer.valid) return;
        _bridgeParent.reset(f4vr::getWorldRootNode());
        if (!_bridgeParent) return;
        _bridgeModel = std::move(model);
        _bridgeGrip = _transfer;
        _bridgeParent->AttachChild(_bridgeModel.get(), true);
        updateBridge(true);
    }

    void SecondaryEquippedWeapon::releaseBridge(bool worldAvailable)
    {
        if (worldAvailable && _bridgeParent && _bridgeModel && _bridgeModel->parent == _bridgeParent.get())
            _bridgeParent->DetachChild(_bridgeModel.get());
        _bridgeModel.reset();
        _bridgeParent.reset();
        _bridgeGrip = {};
    }

    void SecondaryEquippedWeapon::updateBridge(bool worldAvailable)
    {
        if (!_bridgeModel) return;
        if (!worldAvailable || !_bridgeParent || _bridgeParent.get() != f4vr::getWorldRootNode() ||
            _bridgeModel->parent != _bridgeParent.get()) { releaseBridge(worldAvailable); return; }
        RE::NiTransform physical{};
        if (!frik_hand_world_authority::tryGetRawHandWorld(_bridgeGrip.isLeft, physical)) return;
        const auto hand = transform_math::composeTransforms(physical,_bridgeGrip.presentedHandInPhysical);
        const auto world = transform_math::composeTransforms(hand,transform_math::invertTransform(_bridgeGrip.firing.handWeaponLocal));
        _bridgeModel->local = transform_math::composeTransforms(transform_math::invertTransform(_bridgeParent->world),world);
        f4vr::updateDown(_bridgeModel.get(),true);
    }

    SecondaryEquippedWeapon::Transfer SecondaryEquippedWeapon::captureContinuity() const
    {
        if (_transfer.valid) return _transfer;
        Transfer result;
        RE::NiTransform physical{};
        bool left = grip.isFiringHandLeft();
        if (!node() || !grip.isManualOwnershipActive() ||
            !grip.captureMenuCarry(result.firing,result.paired,result.support,result.secondSupport,left) ||
            !vanilla_weapon_grip_frame::resolveModelTranslation(_snapshot.identity.form,node(),result.sourceModelTranslation) ||
            !frik_hand_world_authority::tryGetRawHandWorld(left,physical)) return {};
        if (result.support.validCarry()) result.firing = result.support.grip;
        if (result.paired.valid()) result.firing = result.paired.primary;
        if (!result.firing.valid()) return {};
        const auto hand = transform_math::composeTransforms(node()->world,result.firing.handWeaponLocal);
        result.presentedHandInPhysical = transform_math::composeTransforms(transform_math::invertTransform(physical),hand);
        result.form = _snapshot.identity.form;
        result.instance = _snapshot.identity.instance;
        result.isLeft = left;
        result.valid = weapon_grip_transfer::validFrame(result.presentedHandInPhysical);
        return result;
    }

    void SecondaryEquippedWeapon::interrupt(bool worldAvailable)
    {
        const auto carry = captureContinuity();
        clear(worldAvailable);
        if (carry.valid) adopt(carry);
    }

    void SecondaryEquippedWeapon::clear(bool worldAvailable)
    {
        auto mutation = _gate ? _gate->pauseForMutation() : PhysicsCallbackQuiescenceGate::MutationLease{};
        _ready = false;
        releaseBridge(worldAvailable);
        _heldHands.store(0, std::memory_order_release);
        grip.reset();
        push.clear();
        if (worldAvailable && _world) {
            dynamic.retireAll(_bhk);
            collision.shutdown();
        } else {
            dynamic.abandonHavokStateAfterWorldLoss();
            collision.abandonHavokStateAfterWorldLoss();
        }
        collision.bindEquippedSource(UINT32_MAX);
        _snapshot = {};
        _world = nullptr;
        _bhk = nullptr;
        _transfer = {};
        _pendingRelease = {};
        _transferredRelease = false;
        _contactAcquisition = {};
        toggle = {};
    }

    bool SecondaryEquippedWeapon::prepare(const PhysicsFrameContext& frame, const EquippedWeaponHandlingSettings& settings)
    {
        updateBridge(frame.worldReady);
        native_equipped_weapon::Snapshot observed;
        _ready = false;
        if (_transfer.valid && !frame.menuBlocked) {
            const auto physical = input_remap_runtime::peekRawButtonState(_transfer.isLeft,input_remap_policy::kGrabButtonId);
            _pendingRelease.observe(settings.weaponGrabMode,true,{physical.held,physical.pressed,physical.released},true);
        }
        if (!frame.worldReady || !native_equipped_weapon::read(1, observed)) {
            clear(frame.worldReady && _world == frame.hknpWorld);
            return false;
        }
        if (_world && (_world != frame.hknpWorld || _bhk != frame.bhkWorld ||
                _snapshot.identity != observed.identity || _snapshot.node != observed.node)) {
            const auto transfer = captureContinuity();
            const auto release = _pendingRelease;
            clear(_world == frame.hknpWorld && _bhk == frame.bhkWorld);
            if (transfer.valid && transfer.form == observed.identity.form && transfer.instance == observed.identity.instance) {
                _transfer = transfer;
                _pendingRelease = release;
            }
        }
        _snapshot = std::move(observed);
        if (!_snapshot.attached) return false;
        if (!_world) {
            _world = frame.hknpWorld;
            _bhk = frame.bhkWorld;
            collision.init(_world, _bhk);
            collision.bindEquippedSource(1);
        }
        collision.update(_world, node(), frame.deltaSeconds, true);
        const auto generation = collision.getCurrentWeaponGenerationKey();
        const auto ownership = collision.getCurrentEquippedWeaponOwnershipKey();
        grip.observeEquippedOwnership(ownership, generation, _transfer.valid);
        if (!generation || !ownership || !collision.hasWeaponBody()) return false;
        if (_transfer.valid) {
            if (_transfer.form != _snapshot.identity.form || _transfer.instance != _snapshot.identity.instance) {
                _transfer = {};
                return false;
            }
            RE::NiPoint3 translation{};
            RE::NiTransform physical{};
            if (!vanilla_weapon_grip_frame::resolveModelTranslation(_transfer.form, node(), translation) ||
                !frik_hand_world_authority::hasCalibratedRawHandFrame(_transfer.isLeft) ||
                !frik_hand_world_authority::tryGetRawHandWorld(_transfer.isLeft, physical)) return false;
            auto seat = _transfer.firing;
            const auto shift = translation - _transfer.sourceModelTranslation;
            seat.handWeaponLocal.translate += shift;
            seat.gripWeaponLocal += shift;
            const auto presented = transform_math::composeTransforms(physical, _transfer.presentedHandInPhysical);
            const auto weapon = transform_math::composeTransforms(presented, transform_math::invertTransform(seat.handWeaponLocal));
            const char* failure{};
            bool adopted{};
            if (_transfer.support.validCarry()) {
                adopted = grip.beginTransferredSupportGrip(node(),generation,ownership,_transfer.support,&failure,
                    _transfer.secondSupport.validCarry() ? &_transfer.secondSupport : nullptr);
            } else if (_transfer.paired.valid()) {
                adopted = grip.beginSecondaryEquippedPair(node(),generation,ownership,_transfer.paired,weapon,&failure);
            } else {
                adopted = grip.beginSecondaryEquippedGrip(node(), generation, ownership, _transfer.isLeft, seat, weapon, &failure);
            }
            if (!adopted) {
                ROCK_LOG_SAMPLE_WARN(Weapon, 1000, "Secondary equipped grip pending form={:08X} reason={}", _transfer.form, failure ? failure : "frame-unavailable");
                return false;
            }
            equipped_weapon_toggle_grab_policy::adoptTransferredGrips(toggle, settings.weaponGrabMode, ownership, grip.getGrabInputOccupancy());
            _transferredRelease = _pendingRelease.releaseRequested;
            _transfer = {};
        }
        _ready = grip.isManualOwnershipActive();
        return _ready;
    }

    TwoHandedGripUpdateResult SecondaryEquippedWeapon::update(const PhysicsFrameContext& frame, EquippedWeaponGripFrameInput input,
        const EquippedWeaponHandlingSettings& settings)
    {
        if (!_ready || _world != frame.hknpWorld) return {};
        grip.beginWeaponCollisionPresentationFrame();
        const auto generation = collision.getCurrentWeaponGenerationKey();
        const auto ownership = collision.getCurrentEquippedWeaponOwnershipKey();
        dynamic.updateSurfaceSupportInput(grip.isFiringHandLeft());
        dynamic.beginFrame(runtime_state::currentFrame().frameIndex, _world, _bhk, node(), generation, !frame.menuBlocked);
        std::array<WeaponInteractionContact, 2> contacts{};
        for (unsigned i = 0; i < contacts.size(); ++i) {
            const bool left = i == 1;
            const auto& hand = left ? frame.left : frame.right;
            const bool available = left ? input.leftHandAvailableForAcquisition : input.rightHandAvailableForAcquisition;
            const auto occupancy = grip.getGripOccupancy();
            if (hand.disabled || (!available && !(left ? occupancy.left : occupancy.right).weaponEngaged())) continue;
            const bool touch = collision.tryFindInteractionContactNearPoint(node(), hand.grabAnchorWorld,
                g_rockConfig.rockWeaponInteractionTouchRadius, contacts[i]);
            if (!touch && available) (void)collision.tryFindInteractionContactNearPoint(node(), hand.grabAnchorWorld,
                g_rockConfig.rockWeaponInteractionProbeRadius, contacts[i]);
            contacts[i].acquisitionSource = weapon_interaction_acquisition_policy::resolve(_contactAcquisition[i], touch, contacts[i].valid);
        }
        const auto classification = collision.getEquippedWeaponClassification();
        input.recoilWeapon = {.formID = classification.formID, .keywordFlags = classification.keywordFlags,
            .sizeClass = classification.sizeClass, .source = classification.classificationSource,
            .resolved = classification.hasEquippedWeapon && classification.classificationResolved};
        const auto result = grip.update(node(), contacts[1], contacts[0], input, frame.deltaSeconds, generation, ownership,
            collision, {}, {}, weapon_support_authority_policy::WeaponSupportAuthorityMode::FullTwoHandedSolver, true, settings);
        RE::NiPoint3 pivot{};
        const bool hasPivot = grip.tryGetSurfaceSupportPrimaryGripLocal(node(), generation, pivot);
        const auto resolved = dynamic.finishFrame(frame, !frame.menuBlocked, node(), generation, collision, hasPivot ? &pivot : nullptr);
        if (dynamic.compoundSourcesUnavailable()) collision.requestRebuildForReplacedSources();
        if (resolved.applyVisualCorrection && !grip.applyWeaponCollisionResolvedAuthority(node(), resolved.resolvedWeaponWorld, generation)) _ready = false;
        _heldHands.store((result.after.right.carriesWeapon() ? 1u : 0u) | (result.after.left.carriesWeapon() ? 2u : 0u), std::memory_order_release);
        return result;
    }

    void SecondaryEquippedWeapon::finishPresentation(const PhysicsFrameContext& frame)
    {
        if (!_ready || _world != frame.hknpWorld || !node()) return;
        collision.updateBodiesFromCurrentSourceTransforms(_world, node(), frame.deltaSeconds);
        dynamic.finalizeCompoundPose(collision, node(), frame, collision.getCurrentWeaponGenerationKey());
        grip.finalizeFrikWeaponOwnershipForFrame(collision.getCurrentEquippedWeaponOwnershipKey());
        grip.syncFrikOffHandGripReport();
        releaseBridge(true);
    }
}
