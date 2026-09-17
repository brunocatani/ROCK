#include "physics-interaction/weapon/LooseWeaponGripZone.h"
#include "physics-interaction/weapon/PipeFiringGripPolicy.h"
#include "physics-interaction/weapon/WeaponTypePolicy.h"

#include <array>
#include <cmath>

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/VectorMath.h"
#include "physics-interaction/animation/AuthoredWeaponGripCapturePolicy.h"
#include "physics-interaction/grab/FrikWeaponOffsetCache.h"
#include "physics-interaction/hand/HandFrame.h"
#include "physics-interaction/hand/TrackedHandIsolationPolicy.h"
#include "physics-interaction/visual/FrikVisualAuthorityBridge.h"
#include "physics-interaction/weapon/AuthoredWeaponGripLibrary.h"
#include "physics-interaction/weapon/TwoHandedGrip.h"
#include "physics-interaction/weapon/WeaponGripAuthorityPolicy.h"

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

        constexpr const char* kLiveWeaponNodeCarrierRejected =
            "liveWeaponNodeLocalIsNotNativeCarrier";

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

        /*
         * Resolve one fixed firing grip in WEAPON space. Explicit hFRIK JSON
         * remains user correction authority. Otherwise ROCK consumes the
         * exact native-animation relation learned while this weapon/stock was
         * equipped. Embedded hFRIK data is a cold fallback only; the unrelated
         * live Weapon-node local is accepted solely when the authored feature
         * is disabled to preserve legacy behavior.
         *
         * The tested hand is used only for final world placement. It must
         * never define the Weapon-relative grip point, or the target follows
         * the probing hand instead of remaining fixed on the gun.
         */
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
                state.reason = "throwableSkipsFrikOffset";
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

            const auto frikLookup =
                frik_weapon_offset_cache::findPrimaryWeaponOffset(weapon, looseRoot);
            auto authoredLookup = authored_weapon_grip_library::find(
                weapon,
                looseRoot,
                f4vr::isInPowerArmor());
            authored_weapon_grip_library::applyPipeDefaultOffset(authoredLookup, isLeft);
            const bool promotedPipeCalibration = authoredLookup.found &&
                pipe_firing_grip_policy::isPromotedCalibration(weapon->formID, isLeft, frikLookup.offset);
            constexpr bool authoredGripEligible = true;
            const auto selectedSource = weapon_grip_authority_policy::select(
                weapon_grip_authority_policy::Availability{
                    .frikCustomFile =
                        frikLookup.found &&
                        frikLookup.source == frik_weapon_offset_cache::OffsetSource::CustomFile && !promotedPipeCalibration,
                    .authoredAnimation = authoredGripEligible && authoredLookup.found,
                    .frikEmbeddedResource =
                        frikLookup.found &&
                        frikLookup.source == frik_weapon_offset_cache::OffsetSource::EmbeddedResource,
                    .allowFrikLiveNodeFallback = false,
                });

            RE::NiTransform canonicalHandWeaponLocal{};
            RE::NiTransform canonicalPlacementHandWeaponLocal{};
            bool canonicalPlacementResolved = false;
            /*
             * Native carrier: hFRIK's Weapon-node local is authored under
             * RArm_Hand, so the canonical hand frame composed with that offset
             * is where the native right hand would carry this weapon. The live
             * Weapon node's parent is not consulted: ROCK re-parents it under
             * LArm_Hand for the left carry and presents RArm_Hand itself while
             * the right hand supports or part-carries the equipped weapon.
             */
            const auto tryResolveAttachedRootWorld =
                [&](RE::NiTransform& outAttachedRootWorld,
                    const char*& outFailureReason) {
                    outAttachedRootWorld = {};
                    if (!frikLookup.found) {
                        outFailureReason = frikLookup.reason;
                        return false;
                    }
                    /*
                     * The live Weapon-node local is hFRIK's carrier only while
                     * hFRIK drives that node. Under ROCK's carry (part carry,
                     * support lock, left-firing carry) it is ROCK's own solve
                     * and moves with the carry hands; consuming it rotated
                     * every loose grab of a weapon without an hFRIK offset
                     * while the equipped weapon was carried. hFRIK carries
                     * such a weapon on the animation's own local, which the
                     * authored relation already encodes, so the full authored
                     * hold below is the exact native placement for it.
                     */
                    if (frikLookup.source ==
                        frik_weapon_offset_cache::OffsetSource::LiveWeaponNodeFallback) {
                        outFailureReason = kLiveWeaponNodeCarrierRejected;
                        return false;
                    }

                    outAttachedRootWorld = transform_math::composeTransforms(
                        canonicalHandWorld,
                        frikLookup.offset);
                    outAttachedRootWorld.scale = looseRoot->world.scale;
                    if (!isUsableWorldTransform(outAttachedRootWorld)) {
                        outFailureReason = "nonFiniteAttachedRoot";
                        return false;
                    }
                    return true;
                };

            if (selectedSource == weapon_grip_authority_policy::Source::AuthoredAnimation) {
                canonicalHandWeaponLocal = authoredLookup.rightHandWeaponLocal;
                state.gripWeaponLocal =
                    computeGrabLegacyPalmPivotAWorldFromHandBasis(canonicalHandWeaponLocal, false);
                state.reason = authoredLookup.reason;
                const char* carrierFailureReason =
                    "nativeCarrierUnavailable";

                // Loose authored grabs always use the native-carrier solve.
                // The equipped-only cache must not change their rotation.
                if (!canonicalPlacementResolved) {
                    RE::NiTransform attachedRootWorld{};
                    if (tryResolveAttachedRootWorld(
                            attachedRootWorld,
                            carrierFailureReason)) {
                        const RE::NiTransform positionOnlyWeaponWorld =
                            authored_weapon_grip_capture_policy::
                                resolveAuthoredPrimaryWeaponWorldPositionOnly(
                                    attachedRootWorld,
                                    state.gripWeaponLocal,
                                    canonicalPalmWorld,
                                    [](const RE::NiTransform& transform,
                                        const RE::NiPoint3& point) {
                                        return transform_math::localPointToWorld(
                                            transform,
                                            point);
                                    });
                        canonicalPlacementHandWeaponLocal =
                            transform_math::composeTransforms(
                                transform_math::invertTransform(
                                    positionOnlyWeaponWorld),
                                canonicalHandWorld);
                        if (isUsableWorldTransform(
                                canonicalPlacementHandWeaponLocal)) {
                            canonicalPlacementResolved = true;
                            state.placementReason =
                                "authoredNativeCarrierPositionOnly";
                        } else {
                            carrierFailureReason =
                                "derivedPositionOnlyHoldInvalid";
                        }
                    }
                }

                if (!canonicalPlacementResolved) {
                    canonicalPlacementHandWeaponLocal =
                        canonicalHandWeaponLocal;
                    canonicalPlacementResolved = true;
                    const bool noFrikOffset =
                        carrierFailureReason == kLiveWeaponNodeCarrierRejected;
                    state.placementReason = noFrikOffset ?
                        "authoredFullRigidNoFrikOffset" :
                        "authoredFullRigidFallback";
                    if (!noFrikOffset) {
                        ROCK_LOG_SAMPLE_WARN(Hand, 1000,
                            "Authored loose weapon position-only carrier unavailable formID={:08X} source={}; using full authored hold",
                            weapon->formID,
                            carrierFailureReason);
                    }
                }
            } else if (
                selectedSource == weapon_grip_authority_policy::Source::FrikCustomFile ||
                selectedSource == weapon_grip_authority_policy::Source::FrikEmbeddedResource ||
                selectedSource == weapon_grip_authority_policy::Source::FrikLiveNodeFallback) {
                RE::NiTransform attachedRootWorld{};
                const char* attachedRootFailureReason =
                    "attachedRootUnavailable";
                if (!tryResolveAttachedRootWorld(
                        attachedRootWorld,
                        attachedRootFailureReason)) {
                    state.reason = attachedRootFailureReason;
                    return false;
                }

                state.gripWeaponLocal =
                    transform_math::worldPointToLocal(attachedRootWorld, canonicalPalmWorld);
                canonicalHandWeaponLocal = transform_math::composeTransforms(
                    transform_math::invertTransform(attachedRootWorld),
                    canonicalHandWorld);
                canonicalPlacementHandWeaponLocal =
                    canonicalHandWeaponLocal;
                canonicalPlacementResolved = true;
                state.reason = frikLookup.reason;
                state.placementReason = "frikFullRigid";
            } else {
                state.reason = authoredGripEligible ?
                                   authoredLookup.reason :
                                   weapon_grip_authority_policy::sourceName(selectedSource);
                return false;
            }

            if (!isFinitePoint(state.gripWeaponLocal) ||
                !isUsableWorldTransform(canonicalHandWeaponLocal) ||
                !canonicalPlacementResolved ||
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
             * Probe: the seat's orientation is FRIK's stored offset under the
             * canonical right hand, and a left seat mirrors it through both
             * physical hands. Compare those hands with FRIK's tracked hands.
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
                            state.loosePlacementHandWeaponLocal);
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

    void publishPhysicalLeftHandFrame(const CanonicalPrimaryHandFrame& frame)
    {
        s_physicalLeftHandFrame = frame;
    }

    bool tryResolveNearGrab(bool isLeft, RE::TESObjectREFR* ref, NearGrab& out)
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
        const bool firingEligible = firingResolved && firing.hasFiringHandWeaponLocal &&
            firingDistance <= g_rockConfig.rockWeaponInteractionProbeRadius;

        if (firingEligible) {
            state.indicatorLocal[state.indicatorCount++] = firing.gripWeaponLocal;
        }
        RE::NiTransform supportLocal = authored.supportHandWeaponLocal;
        bool supportResolved = authored.found && authored.hasSupportRelation &&
            authored.supportFingerPose.complete() && isUsableWorldTransform(supportLocal);
        if (supportResolved && !isLeft) {
            RE::NiTransform leftWorld{}, rightWorld{};
            auto* nodes = f4vr::getPlayerNodes();
            supportResolved = nodes && nodes->SecondaryWandNode && nodes->primaryWandNode &&
                physicalHandWorld(true, leftWorld) && physicalHandWorld(false, rightWorld) &&
                isUsableWorldTransform(nodes->SecondaryWandNode->world) &&
                isUsableWorldTransform(nodes->primaryWandNode->world);
            if (supportResolved) {
                supportResolved = TwoHandedGrip::tryBuildMirroredSupportHandWeaponLocal(
                    authored.supportHandWeaponLocal,
                    transform_math::composeTransforms(transform_math::invertTransform(nodes->SecondaryWandNode->world), leftWorld),
                    transform_math::composeTransforms(transform_math::invertTransform(nodes->primaryWandNode->world), rightWorld),
                    supportLocal);
            }
        }
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
            debug.valid = TwoHandedGrip::tryResolveAuthoredActivationAxes(
                authored.rightHandWeaponLocal, root->world, topology,
                debug.sideWorld, debug.downWorld, debug.referenceWorld);
            const auto vec = [](const RE::NiPoint3& p) { return activation::Vec3{p.x, p.y, p.z}; };
            const auto gate = activation::evaluateDirectionGate({
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
            supportEligible = debug.valid && gate.spatialPass;
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
            if (indicator.visible) {
                state.indicatorLocal[state.indicatorCount++] = transform_math::worldPointToLocal(root->world,
                    RE::NiPoint3{indicator.markerWorld.x, indicator.markerWorld.y, indicator.markerWorld.z});
            }
        }
        out.role = selection::select(firingEligible, firingDistance, supportEligible, supportDistance);
        if (out.role == selection::Role::None) {
            return false;
        }
        out.handWeaponLocal = out.role == selection::Role::Support ? supportLocal : firing.firingHandWeaponLocal;
        return true;
    }

    void updateNearGrabCandidate(bool isLeft, RE::TESObjectREFR* ref)
    {
        NearGrab ignored{};
        (void)tryResolveNearGrab(isLeft, ref, ignored);
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

    bool tryResolveLooseWeaponFiringHandHold(
        const bool isLeft,
        RE::TESObjectREFR* weaponRef,
        RE::NiTransform& outHandWorld,
        RE::NiTransform& outHandWeaponLocal,
        const char** outReason)
    {
        if (outReason) {
            *outReason = "missingWeaponRef";
        }
        if (!weaponRef) {
            return false;
        }

        HandZoneState scratch{};
        RE::NiTransform testedHandWorld{};
        const bool resolved = tryResolveGripWorld(isLeft, weaponRef, scratch, &testedHandWorld);
        if (outReason) {
            *outReason = resolved &&
                    !scratch.hasLoosePlacementHandWeaponLocal ?
                "mirroredPlacementHoldUnavailable" :
                scratch.placementReason;
        }
        if (!resolved ||
            !scratch.hasLoosePlacementHandWeaponLocal ||
            !isUsableWorldTransform(testedHandWorld)) {
            return false;
        }

        outHandWorld = testedHandWorld;
        outHandWeaponLocal = scratch.loosePlacementHandWeaponLocal;
        ROCK_LOG_INFO(Hand,
            "{} hand loose weapon placement hold resolved source={} placement={}",
            isLeft ? "left" : "right",
            scratch.reason,
            scratch.placementReason);
        return true;
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
            !scratch.hasFiringHandWeaponLocal ||
            !isUsableWorldTransform(testedHandWorld)) {
            return false;
        }

        outHandWorld = testedHandWorld;
        outHandWeaponLocal = scratch.firingHandWeaponLocal;
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
