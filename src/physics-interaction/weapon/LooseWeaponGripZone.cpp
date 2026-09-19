#include "physics-interaction/weapon/LooseWeaponGripZone.h"
#include "physics-interaction/weapon/WeaponGripCalibration.h"
#include "physics-interaction/weapon/PipeFiringGripPolicy.h"
#include "physics-interaction/weapon/WeaponTypePolicy.h"

#include <array>
#include <cmath>

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/VectorMath.h"
#include "physics-interaction/animation/AuthoredWeaponGripCapturePolicy.h"
#include "physics-interaction/hand/HandFrame.h"
#include "physics-interaction/hand/HandFingerMirrorMath.h"
#include "physics-interaction/hand/TrackedHandIsolationPolicy.h"
#include "physics-interaction/visual/FrikVisualAuthorityBridge.h"
#include "physics-interaction/weapon/AuthoredWeaponGripLibrary.h"
#include "physics-interaction/weapon/TwoHandedGrip.h"

#include "RE/Bethesda/TESBoundObjects.h"
#include "RE/Bethesda/BSExtraData.h"
#include "RockConfig.h"
#include "physics-interaction/weapon/EquippedWeaponHandlingRuntime.h"
#include "RE/Bethesda/TESObjectREFRs.h"
#include "RE/NetImmerse/NiNode.h"

#include "rock_support/Fo4VrRuntime.h"

namespace rock::loose_weapon_grip_zone
{
    namespace
    {
        struct HandZoneState
        {
            bool valid{ false };
            bool palmValid{ false };
            bool insideRadius{ false };
            float insideSettledSeconds{ 0.0f };
            float palmToGripDistance{ 0.0f };
            RE::NiPoint3 gripWorld{};
            RE::NiPoint3 palmWorld{};
            RE::NiPoint3 gripWeaponLocal{};
            RE::NiTransform firingHandWeaponLocal{};
            RE::NiTransform loosePlacementHandWeaponLocal{};
            bool hasFiringHandWeaponLocal{ false };
            bool hasLoosePlacementHandWeaponLocal{ false };
            const char* reason{ "notEvaluated" };
            const char* placementReason{ "notEvaluated" };
        };

        std::array<HandZoneState, 2> s_handStates{};

        // Hover probe over the not-yet-grabbed selection candidate; fully
        // separate from the held-weapon state so grabbing never inherits a
        // stale hover result (and vice versa).
        std::array<HandZoneState, 2> s_hoverStates{};

        // Per-frame publication from PhysicsInteraction; see the header.
        CanonicalPrimaryHandFrame s_canonicalPrimaryHandFrame{};
        CanonicalPrimaryHandFrame s_physicalLeftHandFrame{};
        struct NearState
        {
            std::uint32_t refID{ 0 };
            std::uint64_t captureSequence{ 0 };
            RE::NiPoint3 lastDirection{};
            bool lastDirectionValid{ false };
            AuthoredSupportDebug debug{};
            std::array<RE::NiPoint3, 2> indicatorLocal{};
            std::size_t indicatorCount{ 0 };
        };
        std::array<NearState, 2> s_nearStates{};

        bool isUsableWorldTransform(const RE::NiTransform& transform);

        bool physicalHandWorld(bool isLeft, RE::NiTransform& out)
        {
            const auto& frame = isLeft ? s_physicalLeftHandFrame : s_canonicalPrimaryHandFrame;
            if (frame.valid && isUsableWorldTransform(frame.handWorld)) {
                out = frame.handWorld;
                return true;
            }
            RE::NiPoint3 palm{};
            return !frame.presentedByRock && TwoHandedGrip::tryCaptureRootFlattenedPalmWorld(isLeft, palm, out);
        }


        std::size_t handIndex(const bool isLeft) { return isLeft ? 0u : 1u; }



        bool isFinitePoint(const RE::NiPoint3& point)
        {
            return vector_math::hasFiniteComponents(point);
        }

        bool isUsableWorldTransform(const RE::NiTransform& transform)
        {
            bool rotationFinite = true;
            for (int row = 0; row < 3; ++row) {
                for (int column = 0; column < 3; ++column) {
                    rotationFinite = rotationFinite &&
                                     std::isfinite(
                                         transform.rotate.entry[row][column]);
                }
            }
            return rotationFinite &&
                   isFinitePoint(transform.translate) &&
                   std::isfinite(transform.scale) &&
                   std::fabs(transform.scale) > 0.0001f;
        }

        float pointDistance(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs)
        {
            const float dx = lhs.x - rhs.x;
            const float dy = lhs.y - rhs.y;
            const float dz = lhs.z - rhs.z;
            return std::sqrt(dx * dx + dy * dy + dz * dz);
        }

        // The authored grip fixes the visual wrist in weapon space. ROCK's
        // controller aim independently defines physical carry orientation;
        // probing hands never move the authored target on the loose weapon.
        bool tryResolveGripWorldForModel(
            const bool isLeft,
            const RE::TESObjectWEAP* weapon,
            RE::NiAVObject* looseRoot,
            HandZoneState& state,
            RE::NiTransform* outTestedHandWorld = nullptr)
        {
            if (!weapon) {
                state.reason = "missingWeaponForm";
                return false;
            }
            /*
             * Throwables are hand-thrown refs, not first-person weapon
             * attachments. Do not project their grip zone through FRIK primary
             * offsets; force/pull arrivals seat them through the hand proxy.
             */
            if (weapon->weaponData.type == RE::WEAPON_TYPE::kGrenade ||
                weapon->weaponData.type == RE::WEAPON_TYPE::kMine) {
                state.reason = "throwableSkipsAuthoredGrip";
                return false;
            }

            if (!looseRoot || !isUsableWorldTransform(looseRoot->world)) {
                state.reason = "missingWeaponRoot";
                return false;
            }

            /*
             * ROCK's canonical authored weapon frame is always the physical
             * right-hand/native weapon frame. Native game handedness is not a
             * ROCK input and cannot change controller or offset identity.
             * The published physical frame is the source: the rendered
             * RArm_Hand bone is ROCK's own output while a support lock, part
             * carry, or authored seat presents it, and mirroring a left hold
             * through that seat rotated every loose grab taken while the
             * right hand carried the equipped weapon.
             */
            constexpr bool canonicalHandIsLeft = false;
            RE::NiPoint3 canonicalPalmWorld{};
            RE::NiTransform canonicalHandWorld{};
            const CanonicalPrimaryHandFrame& publishedFrame = s_canonicalPrimaryHandFrame;
            if (publishedFrame.valid && isUsableWorldTransform(publishedFrame.handWorld)) {
                canonicalHandWorld = publishedFrame.handWorld;
                canonicalPalmWorld = computeGrabLegacyPalmPivotAWorldFromHandBasis(
                    canonicalHandWorld,
                    canonicalHandIsLeft);
            } else if (publishedFrame.presentedByRock) {
                state.reason = "canonicalPrimaryHandPresentedByRock";
                return false;
            } else if (!TwoHandedGrip::tryCaptureRootFlattenedPalmWorld(
                           canonicalHandIsLeft,
                           canonicalPalmWorld,
                           canonicalHandWorld)) {
                state.reason = "missingCanonicalPrimaryPalm";
                return false;
            }
            if (!isFinitePoint(canonicalPalmWorld) ||
                !isUsableWorldTransform(canonicalHandWorld)) {
                state.reason = "missingCanonicalPrimaryPalm";
                return false;
            }

            auto authoredLookup = authored_weapon_grip_library::find(
                weapon, looseRoot, f4vr::isInPowerArmor());
            authored_weapon_grip_library::applyPipeDefaultOffset(authoredLookup, isLeft);
            if (!authoredLookup.found) {
                state.reason = authoredLookup.reason;
                return false;
            }

            const RE::NiTransform canonicalHandWeaponLocal = authoredLookup.rightHandWeaponLocal;
            state.gripWeaponLocal = computeGrabLegacyPalmPivotAWorldFromHandBasis(canonicalHandWeaponLocal, false);
            state.reason = authoredLookup.reason;
            RE::NiTransform aimWorld{};
            if (!TwoHandedGrip::tryGetRightWeaponAimWorld(looseRoot->world.scale, aimWorld)) {
                state.reason = "rockWeaponAimUnavailable";
                return false;
            }
            const RE::NiTransform positionOnlyWeaponWorld =
                authored_weapon_grip_capture_policy::resolveAuthoredPrimaryWeaponWorldPositionOnly(
                    aimWorld, state.gripWeaponLocal, canonicalPalmWorld,
                    [](const RE::NiTransform& transform, const RE::NiPoint3& point) {
                        return transform_math::localPointToWorld(transform, point);
                    });
            const RE::NiTransform canonicalPlacementHandWeaponLocal = transform_math::composeTransforms(
                transform_math::invertTransform(positionOnlyWeaponWorld), canonicalHandWorld);
            state.placementReason = "authoredRockControllerPositionOnly";

            if (!isFinitePoint(state.gripWeaponLocal) ||
                !isUsableWorldTransform(canonicalHandWeaponLocal) ||
                !isUsableWorldTransform(
                    canonicalPlacementHandWeaponLocal)) {
                state.reason = "nonFiniteCanonicalGrip";
                return false;
            }

            state.gripWorld = transform_math::localPointToWorld(looseRoot->world, state.gripWeaponLocal);
            if (!isFinitePoint(state.gripWorld)) {
                state.reason = "nonFiniteGripPoint";
                return false;
            }

            /*
             * Compare the physical hands used for ROCK's controller-based
             * seat and left-hand mirror with the provider's tracked hands.
             */
            {
                RE::NiTransform firstPersonRight{};
                const bool rightValid = frik_visual_authority::tryGetHandWorldTransform(
                    frik_visual_authority::Hand::Right, firstPersonRight);
                ROCK_LOG_SAMPLE_INFO(Hand, 1000,
                    "Loose seat right canonical hand: published={} presentedByRock={} vsFirstPerson={:.1f}deg/{:.2f}gu",
                    publishedFrame.valid,
                    publishedFrame.presentedByRock,
                    rightValid ? tracked_hand_isolation_policy::rotationDegrees(canonicalHandWorld, firstPersonRight) : -1.0f,
                    rightValid ? tracked_hand_isolation_policy::translationGameUnits(canonicalHandWorld, firstPersonRight) : -1.0f);
            }

            if (!isLeft) {
                state.firingHandWeaponLocal = canonicalHandWeaponLocal;
                state.loosePlacementHandWeaponLocal =
                    canonicalPlacementHandWeaponLocal;
                state.hasFiringHandWeaponLocal = isUsableWorldTransform(state.firingHandWeaponLocal);
                state.hasLoosePlacementHandWeaponLocal =
                    isUsableWorldTransform(
                        state.loosePlacementHandWeaponLocal);
                if (outTestedHandWorld) {
                    *outTestedHandWorld = canonicalHandWorld;
                }
            } else {
                RE::NiTransform leftHandWorld{};
                if (physicalHandWorld(true, leftHandWorld)) {
                    {
                        RE::NiTransform firstPersonLeft{};
                        const bool leftValid = frik_visual_authority::tryGetHandWorldTransform(
                            frik_visual_authority::Hand::Left, firstPersonLeft);
                        ROCK_LOG_SAMPLE_INFO(Hand, 1000,
                            "Loose seat left physical hand: published={} presentedByRock={} vsFirstPerson={:.1f}deg/{:.2f}gu",
                            s_physicalLeftHandFrame.valid,
                            s_physicalLeftHandFrame.presentedByRock,
                            leftValid ? tracked_hand_isolation_policy::rotationDegrees(leftHandWorld, firstPersonLeft) : -1.0f,
                            leftValid ? tracked_hand_isolation_policy::translationGameUnits(leftHandWorld, firstPersonLeft) : -1.0f);
                    }
                    state.hasFiringHandWeaponLocal = TwoHandedGrip::tryBuildMirroredLeftFiringHandWeaponLocal(
                        canonicalHandWeaponLocal,
                        state.gripWeaponLocal,
                        canonicalHandWorld,
                        leftHandWorld,
                        state.firingHandWeaponLocal);
                    state.hasLoosePlacementHandWeaponLocal =
                        TwoHandedGrip::tryBuildMirroredLeftFiringHandWeaponLocal(
                            canonicalPlacementHandWeaponLocal,
                            state.gripWeaponLocal,
                            canonicalHandWorld,
                            leftHandWorld,
                            state.loosePlacementHandWeaponLocal, false, false);
                    if (state.hasFiringHandWeaponLocal && state.hasLoosePlacementHandWeaponLocal) {
                        // Shift both relations in authored wrist axes while
                        // preserving the controller's independent orientation.
                        state.loosePlacementHandWeaponLocal.translate += weapon_grip_calibration::offsetInWeapon(
                            state.firingHandWeaponLocal, g_rockConfig.rockLeftFiringGripOffsetGameUnits, true);
                    }
                    if (outTestedHandWorld) {
                        *outTestedHandWorld = leftHandWorld;
                    }
                }
            }

            return true;
        }

        bool tryResolveGripWorld(
            const bool isLeft,
            RE::TESObjectREFR* heldRef,
            HandZoneState& state,
            RE::NiTransform* outTestedHandWorld = nullptr)
        {
            if (!heldRef) {
                state.reason = "missingWeaponRef";
                return false;
            }
            auto* baseForm = heldRef->GetObjectReference();
            const auto* weapon = baseForm ? baseForm->As<RE::TESObjectWEAP>() : nullptr;
            return tryResolveGripWorldForModel(
                isLeft,
                weapon,
                heldRef->Get3D(),
                state,
                outTestedHandWorld);
        }
    }

    namespace
    {
        bool resolveSupportRelation(bool isLeft, const authored_weapon_grip_library::LookupResult& authored,
            RE::NiTransform& out)
        {
            out = authored.supportHandWeaponLocal;
            if (!authored.found || !authored.hasSupportRelation || !authored.supportFingerPose.complete() ||
                !isUsableWorldTransform(out)) return false;
            if (isLeft) return true;
            RE::NiTransform leftWorld{}, rightWorld{};
            auto* nodes = f4vr::getPlayerNodes();
            const bool mirrored = nodes && nodes->SecondaryWandNode && nodes->primaryWandNode &&
                physicalHandWorld(true, leftWorld) && physicalHandWorld(false, rightWorld) &&
                isUsableWorldTransform(nodes->SecondaryWandNode->world) && isUsableWorldTransform(nodes->primaryWandNode->world) &&
                TwoHandedGrip::tryBuildMirroredSupportHandWeaponLocal(authored.supportHandWeaponLocal,
                    transform_math::composeTransforms(transform_math::invertTransform(nodes->SecondaryWandNode->world), leftWorld),
                    transform_math::composeTransforms(transform_math::invertTransform(nodes->primaryWandNode->world), rightWorld), out);
            if (mirrored) out = weapon_grip_calibration::shiftedHand(out,
                g_rockConfig.rockRightSupportGripOffsetGameUnits, false);
            return mirrored;
        }

        bool capturePose(bool isLeft, std::uint32_t formId, loose_weapon_authored_grab_policy::Role role,
            const RE::NiTransform& handLocal, const RE::NiTransform& placementLocal,
            const authored_weapon_grip_library::LookupResult& authored,
            AuthoredWeaponGripPose& out)
        {
            using loose_weapon_authored_grab_policy::Role;
            out = {};
            if (!authored.found || role == Role::None) return false;
            const auto& fingers = role == Role::Support ? authored.supportFingerPose : authored.rightFiringFingerPose;
            if (!fingers.complete() || (role == Role::Support && !authored.hasSupportRelation)) return false;
            out.role = role;
            out.isLeft = isLeft;
            out.weaponFormId = formId;
            out.handWeaponLocal = handLocal;
            out.placementHandWeaponLocal = placementLocal;
            out.fingerMask = fingers.enabledMask;
            out.fingerLocals = fingers.localTransforms;
            if ((role == Role::Support) != isLeft &&
                !hand_finger_mirror_math::mirrorFingerLocalsAcrossHands<RE::NiTransform>(
                    std::span<const RE::NiTransform>(fingers.localTransforms), std::span<RE::NiTransform>(out.fingerLocals))) return false;
            return out.valid();
        }
    }

    bool tryResolveAuthoredGrabPose(bool isLeft, RE::TESObjectREFR* ref,
        loose_weapon_authored_grab_policy::Role role, AuthoredWeaponGripPose& out)
    {
        out = {};
        auto* base = ref ? ref->GetObjectReference() : nullptr;
        auto* weapon = base ? base->As<RE::TESObjectWEAP>() : nullptr;
        auto* root = ref ? ref->Get3D() : nullptr;
        if (!weapon || !root) return false;
        const auto authored = authored_weapon_grip_library::find(weapon, root, f4vr::isInPowerArmor());
        RE::NiTransform local{}, placement{};
        if (role == loose_weapon_authored_grab_policy::Role::Support) {
            if (!resolveSupportRelation(isLeft, authored, local)) return false;
            placement = local;
        } else {
            HandZoneState firing{};
            if (!tryResolveGripWorld(isLeft, ref, firing) || !firing.hasFiringHandWeaponLocal) return false;
            local = firing.firingHandWeaponLocal;
            if (!firing.hasLoosePlacementHandWeaponLocal) return false;
            placement = firing.loosePlacementHandWeaponLocal;
        }
        return capturePose(isLeft, weapon->formID, role, local, placement, authored, out);
    }

    void publishPhysicalLeftHandFrame(const CanonicalPrimaryHandFrame& frame)
    {
        s_physicalLeftHandFrame = frame;
    }

    bool tryResolveNearGrab(bool isLeft, RE::TESObjectREFR* ref, NearGrab& out, bool peerHolding)
    {
        namespace activation = authored_weapon_grip_activation_policy;
        namespace selection = loose_weapon_authored_grab_policy;
        out = {};
        auto& state = s_nearStates[handIndex(isLeft)];
        state.debug = {};
        state.indicatorCount = 0;
        auto* base = ref ? ref->GetObjectReference() : nullptr;
        auto* weapon = base ? base->As<RE::TESObjectWEAP>() : nullptr;
        auto* root = ref ? ref->Get3D() : nullptr;
        if (!weapon || !root || !isUsableWorldTransform(root->world) ||
            weapon->weaponData.type == RE::WEAPON_TYPE::kGrenade ||
            weapon->weaponData.type == RE::WEAPON_TYPE::kMine ||
            !physicalHandWorld(isLeft, out.handWorld)) {
            state = {};
            return false;
        }
        out.handWorldValid = true;
        const auto authored = authored_weapon_grip_library::find(weapon, root, f4vr::isInPowerArmor());
        if (state.refID != ref->GetFormID() || state.captureSequence != authored.supportCaptureSequence) {
            state = {};
            state.refID = ref->GetFormID();
            state.captureSequence = authored.supportCaptureSequence;
        }
        const auto probeWorld = computeGrabLegacyPalmPivotAWorldFromHandBasis(out.handWorld, isLeft);
        const auto probeLocal = transform_math::worldPointToLocal(root->world, probeWorld);
        HandZoneState firing{};
        const bool firingResolved = tryResolveGripWorld(isLeft, ref, firing);
        const float firingDistance = pointDistance(probeLocal, firing.gripWeaponLocal) * std::fabs(root->world.scale);
        const auto& settings = equipped_weapon_handling_runtime::current();
        const auto canonicalRightWorld = transform_math::composeTransforms(root->world, authored.rightHandWeaponLocal);
        const auto lateral = computePalmNormalFromHandBasis(canonicalRightWorld, false);
        const auto vec = [](const RE::NiPoint3& p) { return activation::Vec3{p.x, p.y, p.z}; };
        const auto firingZone = firing_grip_reattach_zone_policy::evaluateZone({
            .gripWorld = vec(firing.gripWorld), .palmWorld = vec(probeWorld), .weaponLeftAxisWorld = vec(lateral),
            .reachGameUnits = settings.firingGripReattachRadiusGameUnits,
            .radiusGameUnits = settings.firingGripReattachCylinderRadiusGameUnits,
        });
        const bool firingEligible = firingResolved && firing.hasFiringHandWeaponLocal && authored.found &&
            authored.rightFiringFingerPose.complete() && firingZone.inside;
        RE::NiTransform supportLocal{};
        const bool supportResolved = resolveSupportRelation(isLeft, authored, supportLocal);
        const auto nativeSupportPalm = computeGrabLegacyPalmPivotAWorldFromHandBasis(authored.supportHandWeaponLocal, true);
        out.arrangement = selection::arrangement(authored.found && authored.rightFiringFingerPose.complete(),
            authored.hasSupportRelation && authored.supportFingerPose.complete(), authored.supportPoseAbsent,
            pointDistance(nativeSupportPalm, firing.gripWeaponLocal) * std::abs(root->world.scale),
            settings.firingGripProximitySupportRadiusGameUnits);
        const bool sharedZone = selection::sharedFiringZone(out.arrangement);
        float supportDistance = 0.0f;
        bool supportEligible = false;
        if (supportResolved) {
            const auto seatLocal = computeGrabLegacyPalmPivotAWorldFromHandBasis(supportLocal, isLeft);
            supportDistance = pointDistance(probeLocal, seatLocal) * std::fabs(root->world.scale);
            auto& debug = state.debug;
            debug.seatWorld = transform_math::localPointToWorld(root->world, seatLocal);
            debug.probeWorld = probeWorld;
            debug.radius = g_rockConfig.rockWeaponInteractionProbeRadius;
            RE::TBO_InstanceData* instance = nullptr;
            if (ref->extraList) {
                if (const auto* extra = ref->extraList->GetByType<RE::ExtraInstanceData>()) {
                    instance = extra->data.get();
                }
            }
            const auto* equipSlot = weapon->GetEquipSlot(instance);
            if (!equipSlot) {
                equipSlot = weapon->GetEquipSlot(nullptr);
            }
            debug.family = activation::resolveWeaponFamily({
                .effectiveEquipSlotFormID = equipSlot ? equipSlot->GetFormID() : 0,
                .equippedWeaponPresent = true,
                .meleeOrUnarmed = weapon_type_policy::isMelee(weapon->weaponData.type.get()),
            });
            const auto topology = activation::resolveHandTopology(!isLeft, isLeft);
            debug.valid = !sharedZone && TwoHandedGrip::tryResolveAuthoredActivationAxes(
                authored.rightHandWeaponLocal, root->world, topology,
                debug.sideWorld, debug.downWorld, debug.referenceWorld);
            const auto gate = sharedZone ? activation::DirectionGateResult{} : activation::evaluateDirectionGate({
                .weaponFamily = debug.family,
                .handTopology = topology,
                .authoredSeatWorld = vec(debug.seatWorld),
                .liveProbeWorld = vec(probeWorld),
                .supportSideAxisWorld = vec(debug.sideWorld),
                .downAxisWorld = vec(debug.downWorld),
                .lastStableDirectionWorld = vec(state.lastDirection),
                .radialCapGameUnits = debug.radius,
                .lastStableDirectionValid = state.lastDirectionValid,
            });
            if (gate.directionValid && gate.radialDistanceGameUnits >= activation::kMinimumDirectionDistanceGameUnits) {
                state.lastDirection = {gate.approachDirectionWorld.x, gate.approachDirectionWorld.y, gate.approachDirectionWorld.z};
                state.lastDirectionValid = true;
            }
            // The visible activation cone is the capture area. Requiring mesh
            // touch here made its indicator promise a seat the grab ignored.
            supportEligible = sharedZone ? firingEligible : debug.valid && gate.spatialPass;
            debug.eligible = supportEligible;
            const auto indicator = activation::evaluateIndicator({
                .weaponFamily = debug.family,
                .authoredSeatWorld = vec(debug.seatWorld),
                .supportSideAxisWorld = vec(debug.sideWorld),
                .downAxisWorld = vec(debug.downWorld),
                .activationStateValid = debug.valid,
                .activationSpatialPass = supportEligible,
                .supportGripAllowed = true,
            });
            if (indicator.visible && !sharedZone) {
                state.indicatorLocal[state.indicatorCount++] = transform_math::worldPointToLocal(root->world,
                    RE::NiPoint3{indicator.markerWorld.x, indicator.markerWorld.y, indicator.markerWorld.z});
            }
        }
        out.role = selection::select(firingEligible, firingDistance, supportEligible, supportDistance);
        if (sharedZone && firingEligible) out.role = peerHolding ? selection::Role::Support : selection::Role::Firing;
        // Indicator ownership follows the selected station; overlapping roles
        // never produce two markers for the same hand's acquisition.
        if (out.role == selection::Role::Firing || sharedZone) {
            state.indicatorCount = 0;
            if (firingEligible && firingZone.indicatorValid) {
                state.indicatorLocal[state.indicatorCount++] = transform_math::worldPointToLocal(root->world,
                    RE::NiPoint3{firingZone.indicatorWorld.x, firingZone.indicatorWorld.y, firingZone.indicatorWorld.z});
            }
        }
        if (out.role == selection::Role::None) {
            return false;
        }
        // Hover resolves only the zone and role. Capture/mirror all 15 finger
        // transforms once, when the grab commits through tryResolveAuthoredGrabPose.
        return true;
    }

    void updateNearGrabCandidate(bool isLeft, RE::TESObjectREFR* ref, bool peerHolding)
    {
        NearGrab ignored{};
        (void)tryResolveNearGrab(isLeft, ref, ignored, peerHolding);
    }

    bool tryGetAuthoredSupportDebug(bool isLeft, AuthoredSupportDebug& out)
    {
        out = s_nearStates[handIndex(isLeft)].debug;
        return out.valid;
    }

    std::size_t collectIndicators(bool isLeft, RE::TESObjectREFR* ref, std::span<RE::NiPoint3> positions)
    {
        const auto& state = s_nearStates[handIndex(isLeft)];
        auto* root = ref ? ref->Get3D() : nullptr;
        if (!ref || ref->GetFormID() != state.refID || !root ||
            !isUsableWorldTransform(root->world) || !f4vr::isNodeVisible(root)) {
            return 0;
        }
        const auto count = (std::min)(positions.size(), state.indicatorCount);
        for (std::size_t index = 0; index < count; ++index) {
            positions[index] = transform_math::localPointToWorld(root->world, state.indicatorLocal[index]);
        }
        return count;
    }

    void publishCanonicalPrimaryHandFrame(const CanonicalPrimaryHandFrame& frame)
    {
        s_canonicalPrimaryHandFrame = frame;
        if (s_canonicalPrimaryHandFrame.valid &&
            !isUsableWorldTransform(s_canonicalPrimaryHandFrame.handWorld)) {
            s_canonicalPrimaryHandFrame.valid = false;
        }
    }

    void updateHeldLooseWeapon(
        const bool isLeft,
        const bool holdingLooseWeapon,
        RE::TESObjectREFR* heldRef,
        const bool heldSettled,
        const float dt,
        const float equipRadiusGameUnits,
        const bool firingGripEligible)
    {
        auto& state = s_handStates[handIndex(isLeft)];
        if (!holdingLooseWeapon || !heldRef) {
            state = {};
            return;
        }

        HandZoneState next{};
        next.insideSettledSeconds = state.insideSettledSeconds;

        RE::NiTransform palmHandWorld{};
        next.palmValid = TwoHandedGrip::tryCaptureRootFlattenedPalmWorld(isLeft, next.palmWorld, palmHandWorld) && isFinitePoint(next.palmWorld);
        if (!next.palmValid) {
            next.reason = "missingPalm";
            next.insideSettledSeconds = 0.0f;
            state = next;
            return;
        }

        next.valid = tryResolveGripWorld(isLeft, heldRef, next);
        if (!next.valid) {
            next.insideSettledSeconds = 0.0f;
            if (state.valid || state.reason != next.reason) {
                ROCK_LOG_INFO(Hand,
                    "{} hand loose weapon grip zone unavailable: reason={} formID={:08X}",
                    isLeft ? "left" : "right",
                    next.reason,
                    heldRef->GetFormID());
            }
            state = next;
            return;
        }

        next.palmToGripDistance = pointDistance(next.palmWorld, next.gripWorld);
        next.insideRadius = firingGripEligible && next.palmToGripDistance <= equipRadiusGameUnits;
        if (next.insideRadius && heldSettled) {
            next.insideSettledSeconds += (std::max)(0.0f, dt);
        } else {
            next.insideSettledSeconds = 0.0f;
        }

        if (next.insideRadius != state.insideRadius) {
            ROCK_LOG_INFO(Hand,
                "{} hand loose weapon grip zone {}: palmDist={:.2f}gu radius={:.2f}gu heldSettled={} offsetSource={}",
                isLeft ? "left" : "right",
                next.insideRadius ? "entered" : "exited",
                next.palmToGripDistance,
                equipRadiusGameUnits,
                heldSettled ? "yes" : "no",
                next.reason);
        }

        state = next;
    }

    void updateHoverCandidateWeapon(
        const bool isLeft,
        RE::TESObjectREFR* candidateRef,
        const float equipRadiusGameUnits)
    {
        auto& state = s_hoverStates[handIndex(isLeft)];
        if (!candidateRef) {
            state = {};
            return;
        }

        /*
         * Cheap identity gate before any projection work: the hover probe is
         * fed the raw selection candidate every frame and most selections are
         * not weapons. Non-weapons clear silently instead of churning the
         * unavailable-reason log the way the held path does.
         */
        auto* baseForm = candidateRef->GetObjectReference();
        if (!baseForm || !baseForm->As<RE::TESObjectWEAP>()) {
            state = {};
            return;
        }

        HandZoneState next{};
        RE::NiTransform palmHandWorld{};
        next.palmValid = TwoHandedGrip::tryCaptureRootFlattenedPalmWorld(isLeft, next.palmWorld, palmHandWorld) && isFinitePoint(next.palmWorld);
        if (!next.palmValid) {
            next.reason = "missingPalm";
            state = next;
            return;
        }

        next.valid = tryResolveGripWorld(isLeft, candidateRef, next);
        if (!next.valid) {
            if (state.reason != next.reason) {
                ROCK_LOG_DEBUG(Hand,
                    "{} hand loose weapon grip-zone hover unavailable: reason={} formID={:08X}",
                    isLeft ? "left" : "right",
                    next.reason,
                    candidateRef->GetFormID());
            }
            state = next;
            return;
        }

        next.palmToGripDistance = pointDistance(next.palmWorld, next.gripWorld);
        next.insideRadius = next.palmToGripDistance <= equipRadiusGameUnits;

        if (next.insideRadius != state.insideRadius) {
            ROCK_LOG_DEBUG(Hand,
                "{} hand loose weapon grip-zone hover {}: palmDist={:.2f}gu radius={:.2f}gu formID={:08X} offsetSource={}",
                isLeft ? "left" : "right",
                next.insideRadius ? "entered" : "exited",
                next.palmToGripDistance,
                equipRadiusGameUnits,
                candidateRef->GetFormID(),
                next.reason);
        }

        state = next;
    }

    bool isGripZoneHoverInsideRadius(const bool isLeft)
    {
        const auto& state = s_hoverStates[handIndex(isLeft)];
        return state.valid && state.insideRadius;
    }

    bool isGripZoneEquipSettled(const bool isLeft, const float settleSeconds)
    {
        const auto& state = s_handStates[handIndex(isLeft)];
        return state.valid &&
               state.insideRadius &&
               state.insideSettledSeconds >= settleSeconds;
    }

    bool tryResolveLooseWeaponFiringHandHoldForModel(
        const bool isLeft,
        const RE::TESObjectWEAP* weapon,
        RE::NiAVObject* weaponRoot,
        RE::NiTransform& outHandWorld,
        RE::NiTransform& outHandWeaponLocal,
        const char** outReason)
    {
        HandZoneState scratch{};
        RE::NiTransform testedHandWorld{};
        const bool resolved =
            tryResolveGripWorldForModel(isLeft, weapon, weaponRoot, scratch, &testedHandWorld);
        if (outReason) {
            *outReason = resolved && !scratch.hasFiringHandWeaponLocal ?
                             "mirroredHoldUnavailable" :
                             scratch.reason;
        }
        if (!resolved ||
            !scratch.hasFiringHandWeaponLocal || !scratch.hasLoosePlacementHandWeaponLocal ||
            !isUsableWorldTransform(testedHandWorld)) {
            return false;
        }

        outHandWorld = testedHandWorld;
        outHandWeaponLocal = scratch.loosePlacementHandWeaponLocal;
        return true;
    }

    bool tryGetFiringHandWeaponLocal(
        const bool isLeft,
        RE::NiTransform& outHandWeaponLocal,
        RE::NiPoint3& outFiringGripWeaponLocal)
    {
        const auto& state = s_handStates[handIndex(isLeft)];
        if (!state.valid || !state.hasFiringHandWeaponLocal) {
            return false;
        }
        outHandWeaponLocal = state.firingHandWeaponLocal;
        outFiringGripWeaponLocal = state.gripWeaponLocal;
        if (isLeft) outFiringGripWeaponLocal += weapon_grip_calibration::offsetInWeapon(
            outHandWeaponLocal, g_rockConfig.rockLeftFiringGripOffsetGameUnits, true);
        return true;
    }

    bool tryGetGripZoneDebug(const bool isLeft, GripZoneDebug& out)
    {
        const auto& state = s_handStates[handIndex(isLeft)];
        if (!state.valid) {
            return false;
        }
        out.valid = state.valid;
        out.palmValid = state.palmValid;
        out.insideRadius = state.insideRadius;
        out.gripWorld = state.gripWorld;
        out.palmWorld = state.palmWorld;
        out.palmToGripDistance = state.palmToGripDistance;
        return true;
    }
}
