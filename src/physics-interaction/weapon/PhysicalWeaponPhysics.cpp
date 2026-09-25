#include "physics-interaction/weapon/PhysicalWeaponPhysics.h"
#include "physics-interaction/core/PhysicsFrameContext.h"
#include "physics-interaction/hand/Hand.h"
#include "physics-interaction/native/HeldScenePresentation.h"
#include "physics-interaction/core/RockRuntimeState.h"

namespace rock
{
    void PhysicalWeaponPhysics::setPhysicsCallbackGate(PhysicsCallbackQuiescenceGate* gate)
    {
        _gate = gate;
        collision.setPhysicsCallbackGate(gate);
        dynamic.setPhysicsCallbackGate(gate);
    }

    bool PhysicalWeaponPhysics::clear(bool worldAvailable)
    {
        auto mutation = _gate ? _gate->pauseForMutation() : PhysicsCallbackQuiescenceGate::MutationLease{};
        _ready = false;
        _heldHands.store(0, std::memory_order_release);
        push.clear();
        if (worldAvailable && _world) {
            dynamic.retireAll(_bhk);
            collision.shutdown();
            if (!_nativeBodies.releaseAll(_world, "physical-weapon-retired", [](auto, const auto&) {})) {
                ROCK_LOG_SAMPLE_WARN(Weapon, 1000, "Physical weapon collision restore remains pending");
                return false;
            }
        } else {
            dynamic.abandonHavokStateAfterWorldLoss();
            collision.abandonHavokStateAfterWorldLoss();
            _nativeBodies.clearTracking();
        }
        collision.bindPhysicalSource(nullptr, nullptr);
        _reference.reset(); _root.reset();
        _world = nullptr; _bhk = nullptr; _reportedGeneration = 0;
        return true;
    }

    void PhysicalWeaponPhysics::retireProxy(const char* reason)
    {
        auto mutation = _gate ? _gate->pauseForMutation() : PhysicsCallbackQuiescenceGate::MutationLease{};
        _ready = false;
        _heldHands.store(0, std::memory_order_release);
        dynamic.retireAll(_bhk);
        ROCK_LOG_SAMPLE_WARN(Weapon, 1000, "Physical weapon readiness deferred ref={:08X} reason={}",
            _reference ? _reference->formID : 0, reason);
        if (!_nativeBodies.releaseAll(_world, reason, [](auto, const auto&) {}))
            ROCK_LOG_SAMPLE_WARN(Weapon, 1000, "Physical weapon collision restore pending reason={}", reason);
    }

    bool PhysicalWeaponPhysics::update(const PhysicsFrameContext& frame, RE::TESObjectREFR* reference,
        RE::EquippedWeaponData* data, Hand& owner, Hand* support)
    {
        auto* root = reference && reference->Get3D() ? reference->Get3D()->IsNode() : nullptr;
        _ready = false;
        if (!frame.worldReady || !root || !data || owner.getHeldRef() != reference) {
            (void)clear(frame.worldReady && _world == frame.hknpWorld);
            return false;
        }
        if (_world && (_world != frame.hknpWorld || _reference.get() != reference || _root.get() != root) &&
            !clear(_world == frame.hknpWorld)) return false;
        if (!_world) {
            _world = frame.hknpWorld; _bhk = frame.bhkWorld;
            _reference.reset(reference); _root.reset(root);
            collision.init(_world, _bhk);
        }
        collision.bindPhysicalSource(reference, data);
        collision.update(_world, root, frame.deltaSeconds, true);
        const auto generation = collision.getCurrentWeaponGenerationKey();
        if (!generation || !collision.hasWeaponBody()) {
            retireProxy("physical-weapon-generation-unavailable");
            return false;
        }
        const bool left = owner.isLeft();
        const auto& tracking = left ? frame.left : frame.right;
        dynamic.updateSurfaceSupportInput(left);
        dynamic.beginFrame(runtime_state::currentFrame().frameIndex, _world, _bhk, root, generation, true);
        DynamicWeaponCollisionRuntime::observeWeaponVisualIntent(&dynamic, root, root->world,
            generation, dynamic_weapon_collision_policy::VisualIntentSource::ManagedGrip, &tracking.rawHandWorld);
        RE::NiPoint3 gripWorld{};
        const bool hasGrip = owner.tryGetHeldObjectGrabPivotWorld(_world, gripWorld);
        if (!hasGrip) { retireProxy("physical-weapon-grip-unavailable"); return false; }
        const auto grip = transform_math::worldPointToLocal(root->world, gripWorld);
        auto mutation = _gate ? _gate->pauseForMutation() : PhysicsCallbackQuiescenceGate::MutationLease{};
        const auto result = dynamic.finishFrame(frame, true, root, generation, collision, &grip);
        if (dynamic.compoundSourcesUnavailable()) collision.requestRebuildForReplacedSources();
        if (!result.proxyActive) {
            retireProxy("physical-weapon-proxy-unavailable");
            return false;
        }
        for (const auto body : owner.getHeldBodyIds()) {
            if (!_nativeBodies.acquire(_world, body, "physical-weapon-generated-collision").valid) {
                retireProxy("physical-weapon-activation-failed");
                return false;
            }
        }
        // The earlier grab owns a shared assembly's native scene publication.
        auto* presenter = support && support->heldGrabIdentity() < owner.heldGrabIdentity() ? support : &owner;
        _presenterLeft = presenter->isLeft();
        const auto before = root->world;
        if (!presenter->presentPhysicalWeapon(_world, reference, result.applyVisualCorrection ? result.resolvedWeaponWorld : before)) {
            retireProxy("physical-weapon-scene-unavailable");
            return false;
        }
        if (result.applyVisualCorrection) {
            const auto delta = transform_math::composeTransforms(root->world, transform_math::invertTransform(before));
            const bool ownerReady = owner.presentPhysicalWeaponHand(reference, delta);
            const bool supportReady = !support || support->presentPhysicalWeaponHand(reference, delta);
            if (!ownerReady || !supportReady) {
                retireProxy("physical-weapon-hand-presentation-unavailable");
                return false;
            }
        }
        _ready = true;
        _heldHands.store((left ? 2u : 1u) | (support ? (support->isLeft() ? 2u : 1u) : 0u), std::memory_order_release);
        if (_reportedGeneration != generation) {
            _reportedGeneration = generation;
            ROCK_LOG_INFO(Weapon, "Physical weapon physics ready ref={:08X} generation={:016X} shapes={} proxy={} hand={}",
                reference->formID, generation, collision.getWeaponBodyCount(), dynamic.proxyBodyIdForDebug().value, left ? "left" : "right");
        }
        return true;
    }

    void PhysicalWeaponPhysics::finishParts(const PhysicsFrameContext& frame)
    {
        if (!_ready || !_root || _world != frame.hknpWorld) return;
        if (!held_scene_presentation::refreshPhysicalWeaponParts(_presenterLeft, _world, _root.get())) {
            retireProxy("physical-weapon-mechanical-scene-unavailable");
            return;
        }
        const auto generation = collision.getCurrentWeaponGenerationKey();
        collision.updateBodiesFromCurrentSourceTransforms(_world, _root.get(), frame.deltaSeconds);
        dynamic.finalizeCompoundPose(collision, _root.get(), frame, generation);
    }
}
