#include "physics-interaction/weapon/LooseWeaponGripZone.h"

#include <array>
#include <cmath>

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/grab/FrikWeaponOffsetCache.h"
#include "physics-interaction/hand/HandFrame.h"
#include "physics-interaction/weapon/AuthoredWeaponGripLibrary.h"
#include "physics-interaction/weapon/TwoHandedGrip.h"
#include "physics-interaction/weapon/WeaponGripAuthorityPolicy.h"

#include "RE/Bethesda/TESBoundObjects.h"
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
            bool hasFiringHandWeaponLocal{ false };
            const char* reason{ "notEvaluated" };
        };

        std::array<HandZoneState, 2> s_handStates{};

        // Hover probe over the not-yet-grabbed selection candidate; fully
        // separate from the held-weapon state so grabbing never inherits a
        // stale hover result (and vice versa).
        std::array<HandZoneState, 2> s_hoverStates{};

        std::size_t handIndex(const bool isLeft) { return isLeft ? 0u : 1u; }

        bool isFinitePoint(const RE::NiPoint3& point)
        {
            return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
        }

        bool isUsableWorldTransform(const RE::NiTransform& transform)
        {
            return isFinitePoint(transform.translate) && std::isfinite(transform.scale) && std::fabs(transform.scale) > 0.0001f;
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

            // ROCK's canonical authored weapon frame is always the physical
            // right-hand/native weapon frame. Native game handedness is not a
            // ROCK input and cannot change controller or offset identity.
            constexpr bool canonicalHandIsLeft = false;
            RE::NiPoint3 canonicalPalmWorld{};
            RE::NiTransform canonicalHandWorld{};
            if (!TwoHandedGrip::tryCaptureRootFlattenedPalmWorld(
                    canonicalHandIsLeft,
                    canonicalPalmWorld,
                    canonicalHandWorld) ||
                !isFinitePoint(canonicalPalmWorld) ||
                !isUsableWorldTransform(canonicalHandWorld)) {
                state.reason = "missingCanonicalPrimaryPalm";
                return false;
            }

            const auto frikLookup =
                frik_weapon_offset_cache::findPrimaryWeaponOffset(weapon, looseRoot);
            const auto authoredLookup = authored_weapon_grip_library::find(
                weapon,
                looseRoot,
                f4vr::isInPowerArmor());
            constexpr bool authoredGripEligible = true;
            const auto selectedSource = weapon_grip_authority_policy::select(
                weapon_grip_authority_policy::Availability{
                    .frikCustomFile =
                        frikLookup.found &&
                        frikLookup.source == frik_weapon_offset_cache::OffsetSource::CustomFile,
                    .authoredAnimation = authoredGripEligible && authoredLookup.found,
                    .frikEmbeddedResource =
                        frikLookup.found &&
                        frikLookup.source == frik_weapon_offset_cache::OffsetSource::EmbeddedResource,
                    .allowFrikLiveNodeFallback = false,
                });

            RE::NiTransform canonicalHandWeaponLocal{};
            if (selectedSource == weapon_grip_authority_policy::Source::AuthoredAnimation) {
                canonicalHandWeaponLocal = authoredLookup.rightHandWeaponLocal;
                state.gripWeaponLocal =
                    computeGrabLegacyPalmPivotAWorldFromHandBasis(canonicalHandWeaponLocal, false);
                state.reason = authoredLookup.reason;
            } else if (
                selectedSource == weapon_grip_authority_policy::Source::FrikCustomFile ||
                selectedSource == weapon_grip_authority_policy::Source::FrikEmbeddedResource ||
                selectedSource == weapon_grip_authority_policy::Source::FrikLiveNodeFallback) {
                auto* weaponNode = f4vr::getWeaponNode();
                auto* attachParent = weaponNode ? weaponNode->parent : nullptr;
                if (!attachParent || !isUsableWorldTransform(attachParent->world)) {
                    state.reason =
                        selectedSource == weapon_grip_authority_policy::Source::FrikCustomFile ?
                            "customFrikMissingPrimaryWeaponParent" :
                            "missingPrimaryWeaponParent";
                    return false;
                }

                RE::NiTransform attachedRootWorld =
                    transform_math::composeTransforms(attachParent->world, frikLookup.offset);
                attachedRootWorld.scale = looseRoot->world.scale;
                if (!isUsableWorldTransform(attachedRootWorld)) {
                    state.reason = "nonFiniteAttachedRoot";
                    return false;
                }

                state.gripWeaponLocal =
                    transform_math::worldPointToLocal(attachedRootWorld, canonicalPalmWorld);
                canonicalHandWeaponLocal = transform_math::composeTransforms(
                    transform_math::invertTransform(attachedRootWorld),
                    canonicalHandWorld);
                state.reason = frikLookup.reason;
            } else {
                state.reason = authoredGripEligible ?
                                   authoredLookup.reason :
                                   weapon_grip_authority_policy::sourceName(selectedSource);
                return false;
            }

            if (!isFinitePoint(state.gripWeaponLocal) ||
                !isUsableWorldTransform(canonicalHandWeaponLocal)) {
                state.reason = "nonFiniteCanonicalGrip";
                return false;
            }

            state.gripWorld = transform_math::localPointToWorld(looseRoot->world, state.gripWeaponLocal);
            if (!isFinitePoint(state.gripWorld)) {
                state.reason = "nonFiniteGripPoint";
                return false;
            }

            if (!isLeft) {
                state.firingHandWeaponLocal = canonicalHandWeaponLocal;
                state.hasFiringHandWeaponLocal = isUsableWorldTransform(state.firingHandWeaponLocal);
                if (outTestedHandWorld) {
                    *outTestedHandWorld = canonicalHandWorld;
                }
            } else {
                RE::NiPoint3 leftPalmWorld{};
                RE::NiTransform leftHandWorld{};
                if (TwoHandedGrip::tryCaptureRootFlattenedPalmWorld(true, leftPalmWorld, leftHandWorld)) {
                    state.hasFiringHandWeaponLocal = TwoHandedGrip::tryBuildMirroredLeftFiringHandWeaponLocal(
                        canonicalHandWeaponLocal,
                        state.gripWeaponLocal,
                        canonicalHandWorld,
                        leftHandWorld,
                        state.firingHandWeaponLocal);
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

    void updateHeldLooseWeapon(
        const bool isLeft,
        const bool holdingLooseWeapon,
        RE::TESObjectREFR* heldRef,
        const bool heldSettled,
        const float dt,
        const float equipRadiusGameUnits)
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
        next.insideRadius = next.palmToGripDistance <= equipRadiusGameUnits;
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
            *outReason = resolved && !scratch.hasFiringHandWeaponLocal ? "mirroredHoldUnavailable" : scratch.reason;
        }
        if (!resolved || !scratch.hasFiringHandWeaponLocal || !isUsableWorldTransform(testedHandWorld)) {
            return false;
        }

        outHandWorld = testedHandWorld;
        outHandWeaponLocal = scratch.firingHandWeaponLocal;
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
