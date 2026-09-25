#include "physics-interaction/weapon/DynamicWeaponCollision.h"
#include "RockConfig.h"

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/collision/CollisionLayerPolicy.h"
#include "physics-interaction/core/PhysicsFrameContext.h"
#include "physics-interaction/input/InputRemapRuntime.h"
#include "physics-interaction/native/HavokRuntime.h"

#include <chrono>

namespace rock
{
    namespace
    {
        std::uint64_t surfaceContactTimeMilliseconds() noexcept
        {
            return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count());
        }
    }

    void DynamicWeaponCollisionRuntime::updateSurfaceSupportInput(bool placementOwnsClick)
    {
        _surfaceInputReserved = false; // Also clears before interrupted-frame returns.
        if (!g_rockConfig.rockBipodMode) {
            _surfaceClickRequested = false;
            _surfaceToggle = {};
            if (_surfaceSupport.latched()) {
                weapon_surface_support::release(_surfaceSupport);
                ROCK_LOG_INFO(Weapon, "Weapon surface support released: reason=bipod-mode-disabled");
            }
            return; // Do not read or consume the button while this mode is off.
        }
        if (placementOwnsClick) {
            _surfaceClickRequested = false;
            _surfaceToggle = {};
            return;
        }
        if (input_remap_runtime::isProviderOpenVrGameInputSuppressedForHand(false)) {
            _surfaceClickRequested = false;
            _surfaceToggle = {};
            return; // The provider owns its raw click edge until it yields input.
        }
        const auto raw = input_remap_runtime::consumeRawButtonState(false, weapon_surface_support::kButtonId);
        _surfaceClickRequested = _surfaceToggle.consume({
            .available = raw.available && !input_remap_runtime::isMenuInputActive(),
            .held = raw.held,
            .pressed = raw.pressed,
            .sampleAgeMilliseconds = raw.sampleAgeMilliseconds,
        });
    }

    void DynamicWeaponCollisionRuntime::recordSurfaceSupportContact(
        RE::hknpWorld* world, const std::uint32_t otherBodyId, const RE::NiPoint3& pointGame)
    {
        using namespace dynamic_weapon_collision_policy;
        if (!_enabledAtomic.load(std::memory_order_acquire) || world != _createdWorld ||
            !_created || !isFinitePoint(pointGame)) return;
        const auto other = havok_runtime::snapshotBodyIdentity(world, RE::hknpBodyId{ otherBodyId });
        if (!other.valid || !other.body || other.motionIndex != 0 || !other.body->shape ||
            !collision_layer_policy::isWorldSurfaceLayer(
                other.collisionFilterInfo & collision_layer_policy::FO4_LAYER_FILTER_MASK)) return;
        RE::NiTransform surfaceWorld{}, proxyWorld{};
        if (!havok_runtime::tryResolveLiveBodyWorldTransform(world, other.bodyId, surfaceWorld) ||
            !havok_runtime::tryResolveLiveBodyWorldTransform(world, _body.getBodyId(), proxyWorld) ||
            !isFiniteTransform(surfaceWorld) || std::abs(surfaceWorld.scale) < 0.0001f ||
            !isFiniteTransform(proxyWorld)) return;

        weapon_surface_support::Contact contact{};
        contact.world = reinterpret_cast<std::uintptr_t>(world);
        contact.shape = reinterpret_cast<std::uintptr_t>(other.body->shape);
        contact.collisionObject = reinterpret_cast<std::uintptr_t>(other.collisionObject);
        contact.generation = _createdGenerationKey;
        contact.sampledAtMilliseconds = surfaceContactTimeMilliseconds();
        contact.proxyBodyId = _body.getBodyId().value;
        contact.surfaceBodyId = otherBodyId;
        contact.surfaceWorld = surfaceWorld;
        contact.weaponWorld = reconstructWeaponRoot(proxyWorld, _createdCenterWeaponLocal, _createdWeaponScale);
        contact.weaponPointLocal = transform_math::worldPointToLocal(contact.weaponWorld, pointGame);
        contact.surfacePointLocal = transform_math::worldPointToLocal(surfaceWorld, pointGame);
        contact.valid = isFiniteTransform(contact.weaponWorld) && isFinitePoint(contact.weaponPointLocal) &&
            isFinitePoint(contact.surfacePointLocal);
        if (contact.valid) _surfaceContacts.publish(contact);
    }

    bool DynamicWeaponCollisionRuntime::resolveSurfaceSupportBody(
        const weapon_surface_support::Contact& contact, RE::NiTransform& surfaceWorld) const
    {
        if (!contact.valid || contact.world != reinterpret_cast<std::uintptr_t>(_frameWorld) ||
            contact.generation != _frameGenerationKey) return false;
        const auto other = havok_runtime::snapshotBodyIdentity(_frameWorld, RE::hknpBodyId{ contact.surfaceBodyId });
        return other.valid && other.body && other.motionIndex == 0 &&
            reinterpret_cast<std::uintptr_t>(other.body->shape) == contact.shape &&
            reinterpret_cast<std::uintptr_t>(other.collisionObject) == contact.collisionObject &&
            collision_layer_policy::isWorldSurfaceLayer(
                other.collisionFilterInfo & collision_layer_policy::FO4_LAYER_FILTER_MASK) &&
            havok_runtime::tryResolveLiveBodyWorldTransform(_frameWorld, other.bodyId, surfaceWorld) &&
            dynamic_weapon_collision_policy::isFiniteTransform(surfaceWorld);
    }

    void DynamicWeaponCollisionRuntime::updateSurfaceSupport(
        const PhysicsFrameContext& frame, const RE::NiPoint3* primaryGripWeaponLocal)
    {
        using namespace weapon_surface_support;
        using namespace dynamic_weapon_collision_policy;
        if (_surfaceSupport.ownsPose() && std::abs(_surfaceSupport.captureWorld.scale - _createdWeaponScale) > 0.0001f) {
            _surfaceSupport = {};
            ROCK_LOG_INFO(Weapon, "Weapon surface support cleared: reason=weapon-scale-changed");
        }
        RE::NiTransform surfaceWorld{};
        if (_surfaceSupport.latched() && !resolveSurfaceSupportBody(_surfaceSupport.contact, surfaceWorld)) {
            ROCK_LOG_INFO(Weapon, "Weapon surface support released: reason=surface-invalid body={} generation={:016X}",
                _surfaceSupport.contact.surfaceBodyId, _surfaceSupport.contact.generation);
            release(_surfaceSupport);
        }

        // Use the same current contact for click arbitration and acquisition.
        // Sample even without a click so consumers can defer before pressing.
        Contact contact{};
        RE::NiTransform liveProxyWorld{};
        const bool contactRead = g_rockConfig.rockBipodMode && !_surfaceSupport.latched() && _surfaceContacts.read(contact);
        const bool fresh = contactRead && isFresh(contact, reinterpret_cast<std::uintptr_t>(frame.hknpWorld),
            _createdGenerationKey, _body.getBodyId().value, surfaceContactTimeMilliseconds());
        const bool surfaceValid = fresh && resolveSurfaceSupportBody(contact, surfaceWorld);
        const bool currentContact = surfaceValid &&
            havok_runtime::tryResolveLiveBodyWorldTransform(frame.hknpWorld, _body.getBodyId(), liveProxyWorld);
        if (currentContact) {
            contact.weaponWorld = reconstructWeaponRoot(liveProxyWorld, _createdCenterWeaponLocal, _createdWeaponScale);
            contact.surfaceWorld = surfaceWorld;
        }
        const bool touching = currentContact && contactStillTouches(contact, contact.weaponWorld, surfaceWorld);
        // Keep the unlatch click reserved across later presentation passes in
        // this frame, even after release(). The next input update clears it.
        _surfaceInputReserved = g_rockConfig.rockBipodMode &&
            (_surfaceInputReserved || _surfaceSupport.latched() || touching);

        if (_surfaceClickRequested) {
            _surfaceClickRequested = false;
            if (_surfaceSupport.latched()) {
                release(_surfaceSupport);
                ROCK_LOG_INFO(Weapon, "Weapon surface support released: reason=right-stick-click");
            } else {
                RE::NiPoint3 primaryLocal{};
                bool primaryValid = primaryGripWeaponLocal && isFinitePoint(*primaryGripWeaponLocal);
                if (primaryValid) {
                    primaryLocal = *primaryGripWeaponLocal;
                } else if (_frameIntentDriverValid) {
                    // Native carry can precede authored grip capture. Its
                    // already-isolated physical hand is the acquisition input.
                    primaryLocal = transform_math::worldPointToLocal(_frameRequestedWeaponWorld, _frameIntentDriverWorld.translate);
                    primaryValid = isFinitePoint(primaryLocal);
                }
                if (primaryValid && touching && capture(_surfaceSupport, contact, _frameRequestedWeaponWorld, primaryLocal)) {
                    const auto anchor = transform_math::localPointToWorld(surfaceWorld, contact.surfacePointLocal);
                    ROCK_LOG_INFO(Weapon,
                        "Weapon surface support latched: body={} generation={:016X} anchor=({:.3f},{:.3f},{:.3f}) local=({:.3f},{:.3f},{:.3f})",
                        contact.surfaceBodyId, contact.generation,
                        anchor.x, anchor.y, anchor.z,
                        contact.weaponPointLocal.x, contact.weaponPointLocal.y, contact.weaponPointLocal.z);
                } else {
                    const char* reason = !primaryValid ? "primary-input-unavailable" :
                        !contactRead ? "no-static-contact" : !fresh ? "stale-contact" :
                        !surfaceValid ? "surface-invalid" : !currentContact ? "proxy-unavailable" :
                        !touching ? "contact-separated" : "degenerate-aim";
                    ROCK_LOG_INFO(Weapon, "Weapon surface support click rejected: reason={} body={} generation={:016X}",
                        reason, contact.surfaceBodyId, _createdGenerationKey);
                }
            }
        }

        if (_surfaceSupport.latched()) {
            // A firing-hand transfer changes the grip input, never the anchor.
            // Rebase the aiming relationship at the currently presented pose.
            if (primaryGripWeaponLocal && isFinitePoint(*primaryGripWeaponLocal) &&
                weaponSolverLength(weaponSolverSub(*primaryGripWeaponLocal, _surfaceSupport.primaryGripLocal)) > 0.01f) {
                _surfaceSupport.primaryGripLocal = *primaryGripWeaponLocal;
                _surfaceSupport.primaryTargetAtCapture = transform_math::localPointToWorld(_frameRequestedWeaponWorld, *primaryGripWeaponLocal);
                _surfaceSupport.captureWorld = _surfaceSupport.lastWorld;
                _surfaceSupport.contact.surfaceWorld = surfaceWorld;
            }
            RE::NiTransform supported{};
            if (solve(_surfaceSupport, _frameRequestedWeaponWorld, surfaceWorld, supported)) {
                _frameRequestedWeaponWorld = supported;
            } else {
                _frameRequestedWeaponWorld = _surfaceSupport.lastWorld;
                ROCK_LOG_SAMPLE_WARN(Weapon, 1000, "Weapon surface support holding last pose: reason=degenerate-aim");
            }
        } else if (_surfaceSupport.phase == Phase::Returning) {
            _frameRequestedWeaponWorld = advanceReturn(_surfaceSupport, _frameRequestedWeaponWorld, frame.deltaSeconds);
        }

        const RE::NiPoint3 pivot = _surfaceSupport.latched() ? _surfaceSupport.contact.weaponPointLocal : RE::NiPoint3{};
        if (!setAuthorityPivot(frame, pivot, _frameRequestedWeaponWorld)) {
            _surfaceSupport = {};
            _rebuildRequestedAtomic.store(true, std::memory_order_release);
            ROCK_LOG_WARN(Weapon, "Weapon surface support authority rejected: reason=pivot-constraint-failed");
        }
    }
}
