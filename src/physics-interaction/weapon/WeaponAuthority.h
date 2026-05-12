#pragma once

/*
 * Weapon authority helpers are grouped here because lifecycle, visual authority, muzzle authority, and visual composition all decide final weapon ownership.
 */


// ---- WeaponAuthorityLifecyclePolicy.h ----

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstddef>

namespace rock::weapon_authority_lifecycle_policy
{
    /*
     * Game-stopping menus and world interruptions must make every ROCK-owned
     * authority yield before returning to the game. ROCK keeps FRIK weapon
     * authority, input blocking, and left-hand collision suppression in separate
     * systems, so the interruption decision is centralized here to keep all
     * three lifetimes in lockstep.
     */
    inline bool shouldClearWeaponAuthorityForUpdateInterruption(bool menuOpen, bool rockDisabled, bool skeletonUnavailable)
    {
        return menuOpen || rockDisabled || skeletonUnavailable;
    }

    inline bool isWeaponContactGenerationCurrent(std::uint64_t contactGenerationKey, std::uint64_t currentGenerationKey)
    {
        /*
         * Zero means the older path could not report a source generation. Treat
         * it as usable so legacy non-generated contacts do not get dropped, but
         * reject explicit mismatches because that is the melee/modded-weapon
         * failure mode: a contact from one generated source tree drives a grip
         * solve on another weapon tree after a swap or root change.
         */
        return contactGenerationKey == 0 || currentGenerationKey == 0 || contactGenerationKey == currentGenerationKey;
    }

    enum class GeneratedCollisionRebuildAction : std::uint8_t
    {
        None,
        CreateInitial,
        KeepExistingPending,
        ReplaceExisting,
    };

    struct GeneratedCollisionRebuildInput
    {
        bool hasExistingBodies = false;
        std::uint64_t cachedKey = 0;
        std::uint64_t currentKey = 0;
        bool settingsChanged = false;
        bool sourceReplacementRequired = false;
        std::size_t generatedSourceCount = 0;
        bool hasBuildableSource = false;
    };

    struct GeneratedCollisionRebuildDecision
    {
        GeneratedCollisionRebuildAction action = GeneratedCollisionRebuildAction::None;
        bool pending = false;
        bool keepExistingBodies = false;
        bool destroyExistingBeforeCreate = false;
        bool createReplacementBeforeDestroy = false;
        const char* reason = "unchanged";
    };

    struct GeneratedCollisionScaleInvalidationDecision
    {
        bool destroyExistingBodies = false;
        bool pending = false;
        bool resetCachedIdentity = false;
        bool resetCachedSettings = false;
        std::uint64_t pendingKey = 0;
        const char* reason = "scaleUnchanged";
    };

    inline GeneratedCollisionScaleInvalidationDecision evaluateGeneratedCollisionScaleInvalidation(
        bool hasExistingBodies,
        std::uint64_t cachedKey,
        std::uint64_t pendingKey)
    {
        /*
         * A scale change invalidates generated hknp shapes even when the visual
         * weapon identity is unchanged. The rebuild state must therefore preserve
         * the best known weapon generation as pending while clearing cached
         * identity/settings so the normal mesh-source builder can recreate bodies
         * under the new world conversion on the next update.
         */
        const std::uint64_t nextPendingKey = pendingKey != 0 ? pendingKey : cachedKey;
        return GeneratedCollisionScaleInvalidationDecision{
            .destroyExistingBodies = hasExistingBodies,
            .pending = nextPendingKey != 0,
            .resetCachedIdentity = true,
            .resetCachedSettings = true,
            .pendingKey = nextPendingKey,
            .reason = hasExistingBodies ? "scaleChangedRebuildExisting" : "scaleChangedRebuildPending",
        };
    }

    struct EquippedVisualMissingInput
    {
        bool hasEquippedIdentity = false;
        bool hasExistingBodies = false;
        float missingSeconds = 0.0f;
        float graceSeconds = 0.0f;
    };

    struct EquippedVisualMissingDecision
    {
        bool keepPending = false;
        bool disableExistingBodies = false;
        bool destroyExistingBodies = false;
        bool clearGenerationState = false;
        bool warnPastGrace = false;
        const char* reason = "visualPresent";
    };

    inline EquippedVisualMissingDecision evaluateEquippedVisualMissing(const EquippedVisualMissingInput& input)
    {
        /*
         * An equipped instance without a visible weapon node is a streaming or
         * attach-settle state, not a holster. Generated bodies stay cached so the
         * same equipped instance can resume without a rebuild, but collision is
         * disabled as soon as the drive root is absent because keyframed bodies
         * cannot be safely updated from a missing visual transform.
         */
        if (input.hasEquippedIdentity) {
            return EquippedVisualMissingDecision{
                .keepPending = true,
                .disableExistingBodies = input.hasExistingBodies,
                .destroyExistingBodies = false,
                .clearGenerationState = false,
                .warnPastGrace = input.missingSeconds > input.graceSeconds,
                .reason = input.missingSeconds > input.graceSeconds ? "equippedVisualStillMissing" : "equippedVisualSettling",
            };
        }

        return EquippedVisualMissingDecision{
            .keepPending = false,
            .disableExistingBodies = false,
            .destroyExistingBodies = input.hasExistingBodies,
            .clearGenerationState = true,
            .warnPastGrace = false,
            .reason = "noEquippedIdentity",
        };
    }

    inline GeneratedCollisionRebuildDecision evaluateGeneratedCollisionRebuild(const GeneratedCollisionRebuildInput& input)
    {
        const bool identityChanged = input.currentKey != 0 && input.currentKey != input.cachedKey;
        const bool missingInitialBodies = !input.hasExistingBodies && input.currentKey != 0;
        /*
         * Visible source replacement is a first-class rebuild reason. FO4VR can
         * update a modded weapon's attached geometry while preserving the same
         * equipped identity/visual key, so the source-replacement policy must be
         * able to drive the body-bank swap directly instead of depending on a
         * weapon-key transition that may never happen.
         */
        const bool needsRebuild = missingInitialBodies || identityChanged || input.settingsChanged || input.sourceReplacementRequired;
        if (!needsRebuild) {
            return GeneratedCollisionRebuildDecision{};
        }

        if (!input.hasBuildableSource || input.generatedSourceCount == 0) {
            return GeneratedCollisionRebuildDecision{
                .action = input.hasExistingBodies ? GeneratedCollisionRebuildAction::KeepExistingPending : GeneratedCollisionRebuildAction::None,
                .pending = input.currentKey != 0,
                .keepExistingBodies = input.hasExistingBodies,
                .destroyExistingBeforeCreate = false,
                .createReplacementBeforeDestroy = false,
                .reason = input.hasExistingBodies ? "keepExistingUntilReplacementReady" : "waitingForMeshSources",
            };
        }

        return GeneratedCollisionRebuildDecision{
            .action = input.hasExistingBodies ? GeneratedCollisionRebuildAction::ReplaceExisting : GeneratedCollisionRebuildAction::CreateInitial,
            .pending = false,
            .keepExistingBodies = input.hasExistingBodies,
            .destroyExistingBeforeCreate = false,
            .createReplacementBeforeDestroy = input.hasExistingBodies,
            .reason = input.sourceReplacementRequired && input.hasExistingBodies ? "visibleSourceReplacementReady" :
                                                                               (input.hasExistingBodies ? "replacementReady" : "initialSourcesReady"),
        };
    }

    struct GeneratedCollisionPendingTransitionInput
    {
        bool wasPending = false;
        std::uint64_t previousPendingKey = 0;
        std::uint64_t nextPendingKey = 0;
        bool forceReset = false;
    };

    struct GeneratedCollisionPendingTransitionDecision
    {
        bool pending = false;
        std::uint64_t pendingKey = 0;
        bool startedNewPendingKey = false;
        bool preserveSourceSettle = false;
        bool resetSourceSettle = false;
        bool resetRetryCounter = false;
        bool resetProbeCounters = false;
        const char* reason = "noPendingKey";
    };

    inline GeneratedCollisionPendingTransitionDecision evaluateGeneratedCollisionPendingTransition(
        const GeneratedCollisionPendingTransitionInput& input)
    {
        /*
         * Pending replacement is a state, not a frame-local event. The first
         * frame for a key resets source-settle/retry state; later frames for the
         * same key must preserve settle counters so replacement creation can
         * reach the required stable-source window. This is the exact boundary
         * that prevents "old bodies disabled, replacement never settles".
         */
        if (input.nextPendingKey == 0) {
            return GeneratedCollisionPendingTransitionDecision{};
        }

        const bool startedNewKey =
            !input.wasPending ||
            input.previousPendingKey == 0 ||
            input.previousPendingKey != input.nextPendingKey;
        const bool resetForTransition = startedNewKey || input.forceReset;
        return GeneratedCollisionPendingTransitionDecision{
            .pending = true,
            .pendingKey = input.nextPendingKey,
            .startedNewPendingKey = startedNewKey,
            .preserveSourceSettle = !resetForTransition,
            .resetSourceSettle = resetForTransition,
            .resetRetryCounter = resetForTransition,
            .resetProbeCounters = resetForTransition,
            .reason = resetForTransition ? "pendingKeyStarted" : "pendingKeyContinuing",
        };
    }

    struct ReturnedCachedVisualPendingInput
    {
        bool hasExistingBodies = false;
        bool settingsChanged = false;
        std::uint64_t cachedKey = 0;
        std::uint64_t currentKey = 0;
        std::uint64_t pendingKey = 0;
        std::uint64_t cachedInstanceSignature = 0;
        std::uint64_t pendingInstanceSignature = 0;
        std::uint32_t cachedFormID = 0;
        std::uint32_t pendingFormID = 0;
        std::uintptr_t cachedInstanceDataAddress = 0;
        std::uintptr_t pendingInstanceDataAddress = 0;
        std::uintptr_t cachedInstanceKeywordDataAddress = 0;
        std::uintptr_t pendingInstanceKeywordDataAddress = 0;
        std::uintptr_t cachedObjectInstanceExtraAddress = 0;
        std::uintptr_t pendingObjectInstanceExtraAddress = 0;
    };

    struct ReturnedCachedVisualPendingDecision
    {
        bool cancelPending = false;
        bool keepPendingForStaleVisibleCheck = false;
        const char* reason = "notReturnedCachedPending";
    };

    struct EquippedInstanceRemapWitnessInput
    {
        std::uint64_t cachedInstanceSignature = 0;
        std::uint64_t pendingInstanceSignature = 0;
        std::uint32_t cachedFormID = 0;
        std::uint32_t pendingFormID = 0;
        std::uintptr_t cachedInstanceDataAddress = 0;
        std::uintptr_t pendingInstanceDataAddress = 0;
        std::uintptr_t cachedInstanceKeywordDataAddress = 0;
        std::uintptr_t pendingInstanceKeywordDataAddress = 0;
        std::uintptr_t cachedObjectInstanceExtraAddress = 0;
        std::uintptr_t pendingObjectInstanceExtraAddress = 0;
    };

    inline bool sameFormEquippedInstanceRemapWitnessChanged(const EquippedInstanceRemapWitnessInput& input)
    {
        const bool sameWeaponForm =
            input.cachedFormID != 0 &&
            input.pendingFormID != 0 &&
            input.cachedFormID == input.pendingFormID;
        if (!sameWeaponForm) {
            return false;
        }

        if (input.cachedInstanceSignature != 0 &&
            input.pendingInstanceSignature != 0 &&
            input.cachedInstanceSignature != input.pendingInstanceSignature) {
            return true;
        }

        if (input.cachedInstanceDataAddress != 0 &&
            input.pendingInstanceDataAddress != 0 &&
            input.cachedInstanceDataAddress != input.pendingInstanceDataAddress) {
            return true;
        }

        if (input.cachedInstanceKeywordDataAddress != 0 &&
            input.pendingInstanceKeywordDataAddress != 0 &&
            input.cachedInstanceKeywordDataAddress != input.pendingInstanceKeywordDataAddress) {
            return true;
        }

        return input.cachedObjectInstanceExtraAddress != 0 &&
            input.pendingObjectInstanceExtraAddress != 0 &&
            input.cachedObjectInstanceExtraAddress != input.pendingObjectInstanceExtraAddress;
    }

    inline ReturnedCachedVisualPendingDecision evaluateReturnedCachedVisualPending(
        const ReturnedCachedVisualPendingInput& input)
    {
        /*
         * Manual weapon mod edits can remove the first-person weapon root, then
         * restore the same stale visible tree. Cancelling the pending equipped
         * witness at that point prevents the stale-visible native remap policy
         * from ever running, which leaves the last applied mod without a body.
         * The content signature can remain stable for the exact failure case,
         * so the remap witness also includes the native instance-data and
         * object-instance-extra pointers that the attach remap task must match.
         * Different-form returns are weapon swaps and must stay on the normal
         * native attach path instead of forcing ROCK's remap task.
         */
        if (!input.hasExistingBodies || input.settingsChanged || input.cachedKey == 0 || input.currentKey == 0 ||
            input.pendingKey == 0 || input.currentKey != input.cachedKey || input.pendingKey == input.currentKey) {
            return ReturnedCachedVisualPendingDecision{};
        }

        const bool sameWeaponForm =
            input.cachedFormID != 0 &&
            input.pendingFormID != 0 &&
            input.cachedFormID == input.pendingFormID;
        const bool remapWitnessChanged = sameFormEquippedInstanceRemapWitnessChanged(
            EquippedInstanceRemapWitnessInput{
                .cachedInstanceSignature = input.cachedInstanceSignature,
                .pendingInstanceSignature = input.pendingInstanceSignature,
                .cachedFormID = input.cachedFormID,
                .pendingFormID = input.pendingFormID,
                .cachedInstanceDataAddress = input.cachedInstanceDataAddress,
                .pendingInstanceDataAddress = input.pendingInstanceDataAddress,
                .cachedInstanceKeywordDataAddress = input.cachedInstanceKeywordDataAddress,
                .pendingInstanceKeywordDataAddress = input.pendingInstanceKeywordDataAddress,
                .cachedObjectInstanceExtraAddress = input.cachedObjectInstanceExtraAddress,
                .pendingObjectInstanceExtraAddress = input.pendingObjectInstanceExtraAddress,
            });

        if (remapWitnessChanged) {
            return ReturnedCachedVisualPendingDecision{
                .cancelPending = false,
                .keepPendingForStaleVisibleCheck = true,
                .reason = "sameWeaponRemapWitnessChangedCachedVisualReturned",
            };
        }

        return ReturnedCachedVisualPendingDecision{
            .cancelPending = true,
            .keepPendingForStaleVisibleCheck = false,
            .reason = sameWeaponForm ? "transientMissingVisualReturnedCached" : "differentWeaponReturnedCachedVisual",
        };
    }
}

// ---- WeaponVisualAuthorityMath.h ----

#include "physics-interaction/TransformMath.h"

#include <array>

namespace rock::weapon_visual_authority_math
{
    /*
     * ROCK-owned equipped weapon authority must write the same final visual
     * weapon frame that later drives generated weapon collision bodies. Keeping
     * world-target to parent-local conversion as pure math avoids duplicating
     * node write convention code across two-handed grip, one-handed mesh grip,
     * and debug verification paths. Two-handed support also needs a locked hand
     * frame: controllers guide the weapon solve, while visible hands are
     * recomposed from stored weapon-local frames so the mesh contact point cannot
     * slide along the gun.
     */

    enum class TwoHandedExternalAuthorityStep
    {
        ApplyWeaponVisual,
        PublishHandPose,
        ApplyLockedHandVisual
    };

    enum class LockedHandRole
    {
        Primary,
        Support
    };

    inline constexpr std::array<TwoHandedExternalAuthorityStep, 3> twoHandedExternalAuthorityOrder()
    {
        /*
         * The order is part of the cross-mod contract with FRIK. The weapon must
         * be in its final ROCK-owned frame before hand targets are composed, and
         * the hand pose must be published before FRIK applies/finalizes the
         * external wrist target so right-hand fingers inherit the pivoted wrist.
         */
        return {
            TwoHandedExternalAuthorityStep::ApplyWeaponVisual,
            TwoHandedExternalAuthorityStep::PublishHandPose,
            TwoHandedExternalAuthorityStep::ApplyLockedHandVisual
        };
    }

    inline constexpr int authorityOrderIndex(const TwoHandedExternalAuthorityStep step)
    {
        const auto order = twoHandedExternalAuthorityOrder();
        for (int index = 0; index < static_cast<int>(order.size()); ++index) {
            if (order[static_cast<std::size_t>(index)] == step) {
                return index;
            }
        }
        return -1;
    }

    inline constexpr bool handPosePrecedesLockedHandAuthority()
    {
        return authorityOrderIndex(TwoHandedExternalAuthorityStep::PublishHandPose) <
               authorityOrderIndex(TwoHandedExternalAuthorityStep::ApplyLockedHandVisual);
    }

    inline constexpr bool weaponVisualPrecedesLockedHandAuthority()
    {
        return authorityOrderIndex(TwoHandedExternalAuthorityStep::ApplyWeaponVisual) <
               authorityOrderIndex(TwoHandedExternalAuthorityStep::ApplyLockedHandVisual);
    }

    inline constexpr bool shouldPublishTwoHandedGripPose(const LockedHandRole role)
    {
        /*
         * The primary/right hand already has FRIK's native weapon grip pose. ROCK
         * only needs to move that wrist with the final weapon frame. Publishing a
         * primary mesh pose would replace the user's tuned FRIK grip pose, while
         * the support hand still needs ROCK's mesh/contact pose.
         */
        return role == LockedHandRole::Support;
    }

    inline constexpr bool shouldUseMeshGripFrameRotationAtGrabStart(const LockedHandRole)
    {
        /*
         * Mesh semantics may choose the grip point, but locked hand authority
         * must preserve the wrist rotation already produced by FRIK/INI/native
         * weapon setup at the moment support grip starts. Replacing the right
         * wrist with the mesh-derived grip frame is what causes the visible
         * hand to snap up or down on grab activation.
         */
        return false;
    }

    inline constexpr bool shouldSelectMeshGripPointAtGrabStart(const LockedHandRole role)
    {
        /*
         * The support hand is allowed to select the mesh contact it actually
         * touched. The primary/right hand is not reselected from weapon part
         * names or bounds when support grip starts; the current FRIK-configured
         * hand-to-weapon relationship is already the correct primary grip.
         */
        return role == LockedHandRole::Support;
    }

    template <class Transform>
    inline Transform worldTargetToParentLocal(const Transform& parentWorld, const Transform& targetWorld)
    {
        return transform_math::composeTransforms(transform_math::invertTransform(parentWorld), targetWorld);
    }

    template <class Transform>
    inline Transform weaponLocalFrameToWorld(const Transform& weaponWorld, const Transform& weaponLocalFrame)
    {
        return transform_math::composeTransforms(weaponWorld, weaponLocalFrame);
    }
}

// ---- WeaponMuzzleAuthorityMath.h ----

namespace rock::weapon_muzzle_authority_math
{
    /*
     * ROCK writes the final equipped weapon transform after FRIK's normal weapon
     * update has already run. FRIK's proven muzzle fix copies the projectile node
     * world transform into the fire node so flashes/projectiles originate at the
     * current barrel tip. Keeping the transform rule here gives ROCK the same
     * final-owner behavior without depending on FRIK's weapon adjuster pass.
     */
    template <class Transform>
    [[nodiscard]] Transform fireNodeLocalFromProjectileWorld(const Transform& projectileWorld)
    {
        return projectileWorld;
    }
}

// ---- WeaponVisualCompositionPolicy.h ----

#include <cstdint>
#include <string_view>

namespace rock::weapon_visual_composition_policy
{
    /*
     * Weapon mod swaps can expose the weapon node before the edited part's
     * renderer data has settled. ROCK therefore separates runtime visual
     * composition from transforms: geometry/visibility identity participates in
     * rebuild decisions, while ordinary animation and weapon motion do not.
     */
    inline constexpr std::uint64_t kWeaponVisualCompositionOffset = 1469598103934665603ull;
    inline constexpr std::uint64_t kWeaponVisualCompositionPrime = 1099511628211ull;
    inline constexpr std::uint32_t kRequiredStableVisualFrames = 3;

    struct VisualRecord
    {
        std::uintptr_t nodeAddress{ 0 };
        std::uintptr_t parentAddress{ 0 };
        std::string_view name{};
        std::uint32_t depth{ 0 };
        std::uint32_t childIndex{ 0 };
        std::uint32_t childCount{ 0 };
        bool visible{ false };
        bool triShape{ false };
        std::uintptr_t rendererData{ 0 };
        std::uintptr_t skinInstance{ 0 };
        std::uintptr_t vertexBlock{ 0 };
        std::uintptr_t triangleBlock{ 0 };
        std::uint64_t vertexDesc{ 0 };
        std::uint32_t numTriangles{ 0 };
        std::uint32_t numVertices{ 0 };
        std::uint32_t geometryType{ 0 };
    };

    struct VisualSettleState
    {
        std::uint64_t observedKey{ 0 };
        std::uint32_t stableFrames{ 0 };
    };

    struct VisualSettleResult
    {
        std::uint64_t observedKey{ 0 };
        std::uint32_t stableFrames{ 0 };
        bool keyChanged{ false };
        bool settled{ false };
    };

    inline void mixValue(std::uint64_t& key, std::uint64_t value)
    {
        key ^= value;
        key *= kWeaponVisualCompositionPrime;
    }

    inline void mixString(std::uint64_t& key, std::string_view value)
    {
        for (const char ch : value) {
            mixValue(key, static_cast<unsigned char>(ch));
        }
        mixValue(key, value.size());
    }

    inline void mixVisualRecord(std::uint64_t& key, const VisualRecord& record)
    {
        mixValue(key, record.nodeAddress);
        mixValue(key, record.parentAddress);
        mixString(key, record.name);
        mixValue(key, record.depth);
        mixValue(key, record.childIndex);
        mixValue(key, record.childCount);
        mixValue(key, record.visible ? 1ULL : 0ULL);
        mixValue(key, record.triShape ? 1ULL : 0ULL);
        mixValue(key, record.rendererData);
        mixValue(key, record.skinInstance);
        mixValue(key, record.vertexBlock);
        mixValue(key, record.triangleBlock);
        mixValue(key, record.vertexDesc);
        mixValue(key, record.numTriangles);
        mixValue(key, record.numVertices);
        mixValue(key, record.geometryType);
    }

    inline VisualSettleResult advanceVisualSettle(
        VisualSettleState& state,
        std::uint64_t observedKey,
        std::uint32_t requiredStableFrames = kRequiredStableVisualFrames)
    {
        if (observedKey == 0) {
            const bool changed = state.observedKey != 0 || state.stableFrames != 0;
            state.observedKey = 0;
            state.stableFrames = 0;
            return VisualSettleResult{
                .observedKey = 0,
                .stableFrames = 0,
                .keyChanged = changed,
                .settled = false,
            };
        }

        bool changed = false;
        if (state.observedKey != observedKey) {
            state.observedKey = observedKey;
            state.stableFrames = 1;
            changed = true;
        } else if (state.stableFrames < requiredStableFrames) {
            ++state.stableFrames;
        }

        return VisualSettleResult{
            .observedKey = state.observedKey,
            .stableFrames = state.stableFrames,
            .keyChanged = changed,
            .settled = state.stableFrames >= requiredStableFrames,
        };
    }
}

// ---- WeaponGeneratedSourceCompletenessPolicy.h ----

#include "physics-interaction/weapon/WeaponTypes.h"

#include <cstdint>

namespace rock::weapon_generated_source_completeness_policy
{
    /*
     * FO4VR weapon attachments can become extractable after the first visible
     * weapon root has already stabilized. A spawned modded firearm may therefore
     * produce a partial generated source set, then gain a stock or attachment
     * only after an unrelated mod swap forces a rebuild. ROCK tracks source-set
     * completeness separately from visual identity so firearms can self-enrich
     * when late parts appear while existing bodies remain live until a better
     * replacement set is ready.
     */
    inline constexpr std::uint32_t kRequiredStableGeneratedSourceFrames = 3;

    struct GeneratedSourceCompleteness
    {
        std::uint64_t signature{ 0 };
        std::uint64_t geometryHash{ 0 };
        std::uint64_t boundsExtentScore{ 0 };
        std::size_t sourceCount{ 0 };
        std::size_t pointCount{ 0 };
        std::size_t childClusterCount{ 0 };
        std::uint32_t semanticPartMask{ 0 };
        std::uint32_t gameplayCriticalCount{ 0 };
        std::size_t durableSourceCount{ 0 };
        std::size_t durableChildClusterCount{ 0 };
        std::size_t durablePointCount{ 0 };
        std::uint64_t durableBoundsExtentScore{ 0 };
        std::uint64_t durableGeometryHash{ 0 };
        std::size_t transientReloadSourceCount{ 0 };
        std::uint32_t missingRequiredPackageCoverageMask{ 0 };
        bool firearmLikePackage{ false };
        bool hasRequiredFrontCoverage{ false };
        bool hasRequiredRearCoverage{ false };
    };

    struct GeneratedSourceSettleState
    {
        std::uint64_t observedSignature{ 0 };
        std::uint32_t stableFrames{ 0 };
    };

    struct GeneratedSourceSettleResult
    {
        std::uint64_t observedSignature{ 0 };
        std::uint32_t stableFrames{ 0 };
        bool keyChanged{ false };
        bool settled{ false };
    };

    inline std::uint32_t partMask(WeaponPartKind kind)
    {
        const auto index = static_cast<std::uint32_t>(kind);
        if (index >= 31) {
            return 0;
        }
        return 1u << index;
    }

    inline bool hasPart(const GeneratedSourceCompleteness& completeness, WeaponPartKind kind)
    {
        return (completeness.semanticPartMask & partMask(kind)) != 0;
    }

    inline constexpr std::uint32_t kMissingFrontPackageCoverage = 1u << 0;
    inline constexpr std::uint32_t kMissingRearPackageCoverage = 1u << 1;

    inline bool hasFrontPackageCoverage(const GeneratedSourceCompleteness& completeness)
    {
        return hasPart(completeness, WeaponPartKind::Barrel) ||
               hasPart(completeness, WeaponPartKind::Handguard) ||
               hasPart(completeness, WeaponPartKind::Foregrip) ||
               hasPart(completeness, WeaponPartKind::Pump);
    }

    inline bool hasCompactRearPackageCoverage(const GeneratedSourceCompleteness& completeness)
    {
        return hasPart(completeness, WeaponPartKind::Stock) ||
               hasPart(completeness, WeaponPartKind::Grip);
    }

    inline bool hasLongGunRearPackageCoverage(const GeneratedSourceCompleteness& completeness)
    {
        return hasPart(completeness, WeaponPartKind::Stock);
    }

    inline bool hasFirearmActionPackageEvidence(const GeneratedSourceCompleteness& completeness)
    {
        return hasPart(completeness, WeaponPartKind::Magazine) ||
               hasPart(completeness, WeaponPartKind::Magwell) ||
               hasPart(completeness, WeaponPartKind::Bolt) ||
               hasPart(completeness, WeaponPartKind::Slide) ||
               hasPart(completeness, WeaponPartKind::ChargingHandle) ||
               hasPart(completeness, WeaponPartKind::BreakAction) ||
               hasPart(completeness, WeaponPartKind::Cylinder) ||
               hasPart(completeness, WeaponPartKind::Chamber) ||
               hasPart(completeness, WeaponPartKind::LaserCell) ||
               hasPart(completeness, WeaponPartKind::Lever);
    }

    inline bool isFirearmLikeSourcePackage(const GeneratedSourceCompleteness& completeness)
    {
        /*
         * This is retained as telemetry for logs and debug visualizers only.
         * CommonLib/WEAP classification can be unavailable or Unknown while the
         * visible source package clearly has receiver/action/magazine evidence,
         * but the result no longer gates collision publication.
         */
        if (completeness.sourceCount == 0 || !hasPart(completeness, WeaponPartKind::Receiver)) {
            return false;
        }

        const bool actionOrSocketEvidence = hasFirearmActionPackageEvidence(completeness);
        const bool authoredLongGunCoverage = hasFrontPackageCoverage(completeness) && hasCompactRearPackageCoverage(completeness);
        const bool sightedWeaponPackage = hasPart(completeness, WeaponPartKind::Sight) && (actionOrSocketEvidence || authoredLongGunCoverage);
        return actionOrSocketEvidence || sightedWeaponPackage || authoredLongGunCoverage;
    }

    inline bool hasLongGunPackageEvidence(const GeneratedSourceCompleteness& completeness)
    {
        /*
         * Retained as package telemetry. It used to drive front/rear coherence
         * blocking, but modded weapons can expose valid visible parts without
         * those names. Collision generation now consumes the visible mesh set.
         */
        return hasPart(completeness, WeaponPartKind::Stock) ||
               hasPart(completeness, WeaponPartKind::Handguard) ||
               hasPart(completeness, WeaponPartKind::Foregrip) ||
               hasPart(completeness, WeaponPartKind::Pump) ||
               hasPart(completeness, WeaponPartKind::Bolt) ||
               hasPart(completeness, WeaponPartKind::ChargingHandle) ||
               hasPart(completeness, WeaponPartKind::Lever);
    }

    inline bool hasRequiredRearPackageCoverage(const GeneratedSourceCompleteness& completeness)
    {
        return hasLongGunPackageEvidence(completeness) ?
            hasLongGunRearPackageCoverage(completeness) :
            hasCompactRearPackageCoverage(completeness);
    }

    inline std::uint32_t missingRequiredPackageCoverageMask(const GeneratedSourceCompleteness& completeness)
    {
        /*
         * Collision generation is visibility-driven: if a mesh is present,
         * buildable, and survives geometry dedupe, it gets collision. The old
         * front/rear firearm package gate guessed that a stable source package
         * was incomplete when names like stock/barrel were absent; modded FO4VR
         * weapons routinely violate those lexical assumptions, so the gate
         * blocked exactly the last-applied parts we were trying to recover.
         * Keep the package evidence fields as telemetry only.
         */
        (void)completeness;
        return 0;
    }

    inline std::uint32_t missingExpectedPackageCoverageMask(
        const GeneratedSourceCompleteness& expected,
        const GeneratedSourceCompleteness& observed)
    {
        (void)expected;
        (void)observed;
        return 0;
    }

    inline std::uint32_t replacementMissingRequiredPackageCoverageMask(
        const GeneratedSourceCompleteness& expected,
        const GeneratedSourceCompleteness& observed)
    {
        return missingRequiredPackageCoverageMask(observed) |
               missingExpectedPackageCoverageMask(expected, observed);
    }

    inline bool sourcePackageNeedsMoreCoverage(const GeneratedSourceCompleteness& completeness)
    {
        (void)completeness;
        return false;
    }

    inline const char* sourcePackageCoverageReasonFromMask(std::uint32_t missing)
    {
        if ((missing & kMissingFrontPackageCoverage) != 0 &&
            (missing & kMissingRearPackageCoverage) != 0) {
            return "missingFrontAndRearCoverage";
        }
        if ((missing & kMissingFrontPackageCoverage) != 0) {
            return "missingFrontCoverage";
        }
        if ((missing & kMissingRearPackageCoverage) != 0) {
            return "missingRearCoverage";
        }
        return "sourcePackageCoherent";
    }

    inline const char* sourcePackageCoverageReason(const GeneratedSourceCompleteness& completeness)
    {
        return sourcePackageCoverageReasonFromMask(missingRequiredPackageCoverageMask(completeness));
    }

    inline GeneratedSourceCompleteness withDerivedPackageCoverage(GeneratedSourceCompleteness completeness)
    {
        completeness.firearmLikePackage = isFirearmLikeSourcePackage(completeness);
        completeness.hasRequiredFrontCoverage = hasFrontPackageCoverage(completeness);
        completeness.hasRequiredRearCoverage = hasRequiredRearPackageCoverage(completeness);
        completeness.missingRequiredPackageCoverageMask = missingRequiredPackageCoverageMask(completeness);
        return completeness;
    }

    inline std::uint32_t permanentGameplayCriticalPartMask()
    {
        /*
         * Reload ammo displays are intentionally excluded from this permanence
         * mask. They can appear/disappear during action animation, while stock,
         * receiver, barrel/support, magazine/socket, and action components are
         * persistent weapon structure for collision and support-grip routing.
         */
        return partMask(WeaponPartKind::Receiver) |
               partMask(WeaponPartKind::Barrel) |
               partMask(WeaponPartKind::Handguard) |
               partMask(WeaponPartKind::Foregrip) |
               partMask(WeaponPartKind::Pump) |
               partMask(WeaponPartKind::Stock) |
               partMask(WeaponPartKind::Grip) |
               partMask(WeaponPartKind::Magazine) |
               partMask(WeaponPartKind::Magwell) |
               partMask(WeaponPartKind::Bolt) |
               partMask(WeaponPartKind::Slide) |
               partMask(WeaponPartKind::ChargingHandle) |
               partMask(WeaponPartKind::BreakAction) |
               partMask(WeaponPartKind::Cylinder) |
               partMask(WeaponPartKind::Chamber) |
               partMask(WeaponPartKind::LaserCell) |
               partMask(WeaponPartKind::Lever);
    }

    inline std::uint32_t durableGeneratedCollisionPartMask()
    {
        return permanentGameplayCriticalPartMask() |
               partMask(WeaponPartKind::Sight) |
               partMask(WeaponPartKind::Accessory) |
               partMask(WeaponPartKind::Other);
    }

    inline bool isTransientReloadPart(WeaponPartKind kind)
    {
        return kind == WeaponPartKind::Shell ||
               kind == WeaponPartKind::Round ||
               kind == WeaponPartKind::CosmeticAmmo;
    }

    inline std::uint32_t lostPermanentGameplayCriticalPartMask(
        const GeneratedSourceCompleteness& cached,
        const GeneratedSourceCompleteness& observed)
    {
        return (cached.semanticPartMask & permanentGameplayCriticalPartMask()) & ~observed.semanticPartMask;
    }

    inline bool sourceSetLosesPermanentGameplayCoverage(
        const GeneratedSourceCompleteness& cached,
        const GeneratedSourceCompleteness& observed)
    {
        if (cached.sourceCount == 0 || cached.signature == 0 || observed.sourceCount == 0 || observed.signature == 0) {
            return false;
        }
        return lostPermanentGameplayCriticalPartMask(cached, observed) != 0 ||
               observed.gameplayCriticalCount < cached.gameplayCriticalCount;
    }

    inline std::size_t durablePointCountOrTotal(const GeneratedSourceCompleteness& completeness)
    {
        return completeness.durablePointCount != 0 ? completeness.durablePointCount : completeness.pointCount;
    }

    inline std::uint64_t durableBoundsExtentScoreOrTotal(const GeneratedSourceCompleteness& completeness)
    {
        return completeness.durableBoundsExtentScore != 0 ? completeness.durableBoundsExtentScore : completeness.boundsExtentScore;
    }

    inline std::uint64_t durableGeometryHashOrTotal(const GeneratedSourceCompleteness& completeness)
    {
        return completeness.durableGeometryHash != 0 ? completeness.durableGeometryHash : completeness.geometryHash;
    }

    inline bool sourceSetChangesOnlyTransientReloadGeometry(
        const GeneratedSourceCompleteness& cached,
        const GeneratedSourceCompleteness& observed)
    {
        if (observed.transientReloadSourceCount <= cached.transientReloadSourceCount) {
            return false;
        }

        const std::uint32_t durableMask = durableGeneratedCollisionPartMask();
        return observed.durableSourceCount == cached.durableSourceCount &&
               observed.durableChildClusterCount == cached.durableChildClusterCount &&
               observed.durablePointCount == cached.durablePointCount &&
               observed.durableBoundsExtentScore == cached.durableBoundsExtentScore &&
               durableGeometryHashOrTotal(observed) == durableGeometryHashOrTotal(cached) &&
               (observed.semanticPartMask & durableMask) == (cached.semanticPartMask & durableMask);
    }

    inline bool sourceSetHasMaterialDurableGeometryChange(
        const GeneratedSourceCompleteness& cached,
        const GeneratedSourceCompleteness& observed,
        std::size_t minPointGain = 64,
        std::uint64_t minBoundsExtentGain = 200)
    {
        if (cached.sourceCount == 0 || cached.signature == 0 || observed.sourceCount == 0 || observed.signature == 0) {
            return false;
        }
        if (sourceSetChangesOnlyTransientReloadGeometry(cached, observed)) {
            return false;
        }

        const std::size_t cachedDurablePoints = durablePointCountOrTotal(cached);
        const std::size_t observedDurablePoints = durablePointCountOrTotal(observed);
        const std::uint64_t cachedDurableBounds = durableBoundsExtentScoreOrTotal(cached);
        const std::uint64_t observedDurableBounds = durableBoundsExtentScoreOrTotal(observed);
        const std::uint64_t cachedDurableGeometryHash = durableGeometryHashOrTotal(cached);
        const std::uint64_t observedDurableGeometryHash = durableGeometryHashOrTotal(observed);
        const std::size_t materialPointGain = (std::max)(minPointGain, cachedDurablePoints / 20);
        const std::uint64_t materialBoundsGain = (std::max)(minBoundsExtentGain, cachedDurableBounds / 50);
        const bool preservesSourceCoverage =
            observed.sourceCount >= cached.sourceCount &&
            observed.childClusterCount >= cached.childClusterCount &&
            observed.durableSourceCount >= cached.durableSourceCount &&
            observed.durableChildClusterCount >= cached.durableChildClusterCount;
        const bool preservesPointCoverage = observedDurablePoints + materialPointGain >= cachedDurablePoints;
        const bool preservesBoundsCoverage = observedDurableBounds + materialBoundsGain >= cachedDurableBounds;
        const bool durableGeometryHashChanged = observedDurableGeometryHash != 0 && observedDurableGeometryHash != cachedDurableGeometryHash;
        const bool materialPointImprovement = observedDurablePoints >= cachedDurablePoints + materialPointGain;
        const bool materialBoundsImprovement = observedDurableBounds >= cachedDurableBounds + materialBoundsGain;

        return preservesSourceCoverage &&
               preservesPointCoverage &&
               preservesBoundsCoverage &&
               durableGeometryHashChanged &&
               (materialPointImprovement || materialBoundsImprovement);
    }

    template <class T>
    inline T absoluteDifference(T a, T b)
    {
        return a >= b ? a - b : b - a;
    }

    /*
     * FO4VR can preserve the same first-person source signature while a weapon
     * mod swaps the durable mesh to a smaller or otherwise different shape. A
     * growth-only enrichment rule holds those bodies as stale forever, while
     * broad native refresh/equip calls mutate too much state. Treat comparable
     * durable source sets as a real replacement only when the durable geometry
     * hash changes and the point/bounds delta clears the same material threshold
     * used by enrichment, which keeps transient reload and small skinned jitter
     * out of the rebuild path.
     */
    inline bool sourceSetHasMaterialDurableGeometryDifference(
        const GeneratedSourceCompleteness& cached,
        const GeneratedSourceCompleteness& observed,
        std::size_t minPointDelta = 64,
        std::uint64_t minBoundsExtentDelta = 200)
    {
        if (cached.sourceCount == 0 || cached.signature == 0 || observed.sourceCount == 0 || observed.signature == 0) {
            return false;
        }
        if (sourceSetChangesOnlyTransientReloadGeometry(cached, observed)) {
            return false;
        }

        const bool comparableSourceStructure =
            observed.sourceCount == cached.sourceCount &&
            observed.childClusterCount == cached.childClusterCount &&
            observed.durableSourceCount == cached.durableSourceCount &&
            observed.durableChildClusterCount == cached.durableChildClusterCount;
        if (!comparableSourceStructure) {
            return false;
        }

        const std::size_t cachedDurablePoints = durablePointCountOrTotal(cached);
        const std::size_t observedDurablePoints = durablePointCountOrTotal(observed);
        const std::uint64_t cachedDurableBounds = durableBoundsExtentScoreOrTotal(cached);
        const std::uint64_t observedDurableBounds = durableBoundsExtentScoreOrTotal(observed);
        const std::uint64_t cachedDurableGeometryHash = durableGeometryHashOrTotal(cached);
        const std::uint64_t observedDurableGeometryHash = durableGeometryHashOrTotal(observed);
        const std::size_t materialPointDelta = (std::max)(minPointDelta, cachedDurablePoints / 20);
        const std::uint64_t materialBoundsDelta = (std::max)(minBoundsExtentDelta, cachedDurableBounds / 50);
        const bool durableGeometryHashChanged = observedDurableGeometryHash != 0 && observedDurableGeometryHash != cachedDurableGeometryHash;
        const bool materialPointDifference = absoluteDifference(observedDurablePoints, cachedDurablePoints) >= materialPointDelta;
        const bool materialBoundsDifference = absoluteDifference(observedDurableBounds, cachedDurableBounds) >= materialBoundsDelta;

        return durableGeometryHashChanged && (materialPointDifference || materialBoundsDifference);
    }

    inline bool sourceSetEquivalent(
        const GeneratedSourceCompleteness& cached,
        const GeneratedSourceCompleteness& observed)
    {
        if (cached.sourceCount == 0 || cached.signature == 0 || observed.sourceCount == 0 || observed.signature == 0) {
            return false;
        }

        return observed.signature == cached.signature &&
               observed.geometryHash == cached.geometryHash &&
               observed.boundsExtentScore == cached.boundsExtentScore &&
               observed.sourceCount == cached.sourceCount &&
               observed.pointCount == cached.pointCount &&
               observed.childClusterCount == cached.childClusterCount &&
               observed.semanticPartMask == cached.semanticPartMask &&
               observed.gameplayCriticalCount == cached.gameplayCriticalCount &&
               observed.durableSourceCount == cached.durableSourceCount &&
               observed.durableChildClusterCount == cached.durableChildClusterCount &&
               observed.durablePointCount == cached.durablePointCount &&
               observed.durableBoundsExtentScore == cached.durableBoundsExtentScore &&
               observed.durableGeometryHash == cached.durableGeometryHash &&
               observed.transientReloadSourceCount == cached.transientReloadSourceCount;
    }

    inline bool sourceSetHasSameVisualStructure(
        const GeneratedSourceCompleteness& cached,
        const GeneratedSourceCompleteness& observed)
    {
        if (cached.sourceCount == 0 || cached.signature == 0 || observed.sourceCount == 0 || observed.signature == 0) {
            return false;
        }

        return observed.signature == cached.signature &&
               observed.sourceCount == cached.sourceCount &&
               observed.childClusterCount == cached.childClusterCount &&
               observed.semanticPartMask == cached.semanticPartMask &&
               observed.gameplayCriticalCount == cached.gameplayCriticalCount &&
               observed.durableSourceCount == cached.durableSourceCount &&
               observed.durableChildClusterCount == cached.durableChildClusterCount &&
               observed.transientReloadSourceCount == cached.transientReloadSourceCount;
    }

    inline GeneratedSourceSettleResult advanceGeneratedSourceSettle(
        GeneratedSourceSettleState& state,
        std::uint64_t observedSignature,
        std::uint32_t requiredStableFrames = kRequiredStableGeneratedSourceFrames)
    {
        if (observedSignature == 0) {
            const bool changed = state.observedSignature != 0 || state.stableFrames != 0;
            state.observedSignature = 0;
            state.stableFrames = 0;
            return GeneratedSourceSettleResult{
                .observedSignature = 0,
                .stableFrames = 0,
                .keyChanged = changed,
                .settled = false,
            };
        }

        bool changed = false;
        if (state.observedSignature != observedSignature) {
            state.observedSignature = observedSignature;
            state.stableFrames = 1;
            changed = true;
        } else if (state.stableFrames < requiredStableFrames) {
            ++state.stableFrames;
        }

        return GeneratedSourceSettleResult{
            .observedSignature = state.observedSignature,
            .stableFrames = state.stableFrames,
            .keyChanged = changed,
            .settled = state.stableFrames >= requiredStableFrames,
        };
    }

    inline bool sourceSetImproved(
        const GeneratedSourceCompleteness& cached,
        const GeneratedSourceCompleteness& observed,
        std::size_t minPointGain = 64,
        std::uint64_t minBoundsExtentGain = 200)
    {
        /*
         * Late-source enrichment is allowed to react to a different visible
         * source package, but it must not become a semantic completeness retry
         * loop or rebuild on skinned per-frame point jitter. Runtime replacement
         * decisions handle owner/key/settings changes separately; this predicate
         * is only the low-frequency "did the visible mesh set materially grow?"
         * probe used after bodies are already live.
         */
        if (observed.sourceCount == 0 || observed.signature == 0) {
            return false;
        }
        if (cached.sourceCount == 0 || cached.signature == 0) {
            return true;
        }
        const std::size_t materialPointGain = (std::max)(minPointGain, cached.pointCount / 20);
        const std::uint64_t materialBoundsGain = (std::max)(minBoundsExtentGain, cached.boundsExtentScore / 50);
        const bool preservesSourceCoverage =
            observed.sourceCount >= cached.sourceCount &&
            observed.childClusterCount >= cached.childClusterCount;
        const bool preservesPointCoverage = observed.pointCount + materialPointGain >= cached.pointCount;
        const bool preservesBoundsCoverage = observed.boundsExtentScore + materialBoundsGain >= cached.boundsExtentScore;
        if (!preservesSourceCoverage || !preservesPointCoverage || !preservesBoundsCoverage) {
            return false;
        }
        if (sourceSetChangesOnlyTransientReloadGeometry(cached, observed)) {
            return false;
        }
        if (sourceSetHasMaterialDurableGeometryChange(cached, observed, minPointGain, minBoundsExtentGain)) {
            return true;
        }
        if (observed.signature == cached.signature) {
            return false;
        }
        if (observed.sourceCount > cached.sourceCount) {
            return true;
        }
        if (observed.childClusterCount > cached.childClusterCount) {
            return true;
        }
        if (observed.pointCount >= cached.pointCount + materialPointGain) {
            return true;
        }
        if (observed.boundsExtentScore >= cached.boundsExtentScore + materialBoundsGain) {
            return true;
        }
        return false;
    }

    inline std::uint64_t makeGeneratedSourceReplacementSettleKey(
        const GeneratedSourceCompleteness& cached,
        const GeneratedSourceCompleteness& observed)
    {
        if (observed.signature == 0) {
            return 0;
        }
        if (cached.signature != 0 &&
            observed.signature == cached.signature &&
            (sourceSetHasMaterialDurableGeometryChange(cached, observed) ||
                sourceSetHasMaterialDurableGeometryDifference(cached, observed))) {
            std::uint64_t key = weapon_visual_composition_policy::kWeaponVisualCompositionOffset;
            weapon_visual_composition_policy::mixString(key, "ROCKGeneratedWeaponMaterialSourceSettleV1");
            weapon_visual_composition_policy::mixValue(key, observed.signature);
            weapon_visual_composition_policy::mixValue(key, durableGeometryHashOrTotal(observed));
            weapon_visual_composition_policy::mixValue(key, durablePointCountOrTotal(observed));
            weapon_visual_composition_policy::mixValue(key, durableBoundsExtentScoreOrTotal(observed));
            weapon_visual_composition_policy::mixValue(key, observed.durableSourceCount);
            weapon_visual_composition_policy::mixValue(key, observed.durableChildClusterCount);
            return key;
        }
        return observed.signature;
    }

    inline std::uint64_t makeGeneratedSourcePendingSettleKey(
        const GeneratedSourceCompleteness& cached,
        const GeneratedSourceCompleteness& observed,
        bool ownerIdentityChanged,
        bool settingsChanged)
    {
        /*
         * Pending weapon replacement settles the identity of the visible source
         * package before colliders are rebuilt. Owner/settings changes already
         * require a full replacement, so their settle key must stay structural:
         * skinned or animated heavy weapons can jitter point counts and support
         * fits every frame without changing the actual attached part set.
         */
        if (observed.signature == 0) {
            return 0;
        }
        if (ownerIdentityChanged || settingsChanged || cached.signature == 0 || observed.signature != cached.signature) {
            return observed.signature;
        }
        return makeGeneratedSourceReplacementSettleKey(cached, observed);
    }

    struct GeneratedSourceReplacementInput
    {
        bool hasExistingBodies = false;
        bool ownerIdentityChanged = false;
        bool settingsChanged = false;
        bool allowIncompletePackageWithoutExistingBodies = false;
        GeneratedSourceCompleteness expected{};
        GeneratedSourceCompleteness cached{};
        GeneratedSourceCompleteness observed{};
    };

    struct GeneratedSourceReplacementDecision
    {
        bool allowed = true;
        bool keepExistingBodies = false;
        bool pending = false;
        bool adoptExistingBodies = false;
        std::uint32_t lostPermanentGameplayMask = 0;
        std::uint32_t missingRequiredPackageCoverageMask = 0;
        const char* reason = "sourceReplacementAllowed";
    };

    inline GeneratedSourceReplacementDecision evaluateGeneratedSourceReplacement(
        const GeneratedSourceReplacementInput& input)
    {
        /*
         * Generated collision now follows the visible weapon mesh package.
         * Semantic classifiers still describe contacts, reload roles, and debug
         * evidence, but they no longer decide whether a visible source is allowed
         * to publish collision. That prevents modded firearms from deadlocking on
         * guessed stock/barrel completeness and stops same-owner semantic-loss
         * retries from replacing the user's grab with a periodic rebuild loop.
         */
        if (input.observed.sourceCount == 0 || input.observed.signature == 0) {
            return GeneratedSourceReplacementDecision{
                .allowed = false,
                .keepExistingBodies = input.hasExistingBodies,
                .pending = true,
                .adoptExistingBodies = false,
                .lostPermanentGameplayMask = 0,
                .reason = input.hasExistingBodies ? "waitingForBuildableSources" : "noInitialSources",
            };
        }

        if (!input.hasExistingBodies || input.cached.sourceCount == 0 || input.cached.signature == 0) {
            return GeneratedSourceReplacementDecision{
                .allowed = true,
                .keepExistingBodies = false,
                .pending = false,
                .adoptExistingBodies = false,
                .lostPermanentGameplayMask = 0,
                .reason = "initialOrUntrackedSources",
            };
        }

        if (input.ownerIdentityChanged) {
            return GeneratedSourceReplacementDecision{
                .allowed = true,
                .keepExistingBodies = input.hasExistingBodies,
                .pending = false,
                .adoptExistingBodies = false,
                .lostPermanentGameplayMask = 0,
                .reason = "ownerIdentityChanged",
            };
        }

        if (input.settingsChanged) {
            return GeneratedSourceReplacementDecision{
                .allowed = true,
                .keepExistingBodies = input.hasExistingBodies,
                .pending = false,
                .adoptExistingBodies = false,
                .lostPermanentGameplayMask = 0,
                .reason = "settingsChanged",
            };
        }

        if (sourceSetEquivalent(input.cached, input.observed)) {
            return GeneratedSourceReplacementDecision{
                .allowed = false,
                .keepExistingBodies = true,
                .pending = false,
                .adoptExistingBodies = true,
                .lostPermanentGameplayMask = 0,
                .reason = "ownerUnchangedEquivalentVisibleGeometry",
            };
        }

        return GeneratedSourceReplacementDecision{
            .allowed = true,
            .keepExistingBodies = input.hasExistingBodies,
            .pending = false,
            .adoptExistingBodies = false,
            .lostPermanentGameplayMask = 0,
            .reason = "ownerUnchangedVisibleGeometryChanged",
        };
    }

    inline std::uint64_t makeGeneratedWeaponBodySetKey(
        std::uint64_t equippedWeaponKey,
        const GeneratedSourceCompleteness& sourceCompleteness,
        std::uint64_t bodySetEpoch)
    {
        if (equippedWeaponKey == 0 || sourceCompleteness.signature == 0 || bodySetEpoch == 0) {
            return 0;
        }

        std::uint64_t key = weapon_visual_composition_policy::kWeaponVisualCompositionOffset;
        weapon_visual_composition_policy::mixString(key, "ROCKGeneratedWeaponBodySetV1");
        weapon_visual_composition_policy::mixValue(key, equippedWeaponKey);
        weapon_visual_composition_policy::mixValue(key, sourceCompleteness.signature);
        weapon_visual_composition_policy::mixValue(key, sourceCompleteness.geometryHash);
        weapon_visual_composition_policy::mixValue(key, sourceCompleteness.boundsExtentScore);
        weapon_visual_composition_policy::mixValue(key, sourceCompleteness.sourceCount);
        weapon_visual_composition_policy::mixValue(key, sourceCompleteness.pointCount);
        weapon_visual_composition_policy::mixValue(key, sourceCompleteness.childClusterCount);
        weapon_visual_composition_policy::mixValue(key, sourceCompleteness.semanticPartMask);
        weapon_visual_composition_policy::mixValue(key, sourceCompleteness.gameplayCriticalCount);
        weapon_visual_composition_policy::mixValue(key, sourceCompleteness.durableSourceCount);
        weapon_visual_composition_policy::mixValue(key, sourceCompleteness.durableChildClusterCount);
        weapon_visual_composition_policy::mixValue(key, sourceCompleteness.durablePointCount);
        weapon_visual_composition_policy::mixValue(key, sourceCompleteness.durableBoundsExtentScore);
        weapon_visual_composition_policy::mixValue(key, sourceCompleteness.durableGeometryHash);
        weapon_visual_composition_policy::mixValue(key, sourceCompleteness.transientReloadSourceCount);
        weapon_visual_composition_policy::mixValue(key, sourceCompleteness.missingRequiredPackageCoverageMask);
        weapon_visual_composition_policy::mixValue(key, sourceCompleteness.firearmLikePackage ? 1ULL : 0ULL);
        weapon_visual_composition_policy::mixValue(key, sourceCompleteness.hasRequiredFrontCoverage ? 1ULL : 0ULL);
        weapon_visual_composition_policy::mixValue(key, sourceCompleteness.hasRequiredRearCoverage ? 1ULL : 0ULL);
        weapon_visual_composition_policy::mixValue(key, bodySetEpoch);
        return key;
    }
}

// ---- WeaponGenerationIdentityPolicy.h ----

namespace rock::weapon_generation_identity_policy
{
    /*
     * Generated collision must rebuild from the equipped weapon instance, not
     * only from the visual tree. FO4VR and FRIK can reuse or temporarily hide
     * the same weapon root while a modded instance changes underneath it. Mixing
     * the form, instance, equipped-data object, and visual composition key keeps
     * generated mesh detail tied to the visible tree while still detecting
     * equipped-instance changes when a root is reused or invisible during settle.
     */
    struct EquippedWeaponGenerationIdentity
    {
        std::uint32_t formID{ 0 };
        std::uintptr_t formAddress{ 0 };
        std::uintptr_t instanceDataAddress{ 0 };
        std::uintptr_t instanceKeywordDataAddress{ 0 };
        std::uint64_t instanceContentKey{ 0 };
        std::uintptr_t objectInstanceExtraAddress{ 0 };
        std::uint64_t objectIndexDataSignature{ 0 };
        std::uint32_t objectIndexDataCount{ 0 };
        std::uint32_t activeModCount{ 0 };
        std::uint32_t disabledModCount{ 0 };
        std::uintptr_t equippedDataAddress{ 0 };
        std::uintptr_t equippedObjectAddress{ 0 };
        std::string_view displayName{};
        bool hasEquippedWeapon{ false };
    };

    struct EquippedWeaponInstanceWitness
    {
        std::uint64_t signature{ 0 };
        std::uint32_t formID{ 0 };
        std::uintptr_t formAddress{ 0 };
        std::uintptr_t instanceDataAddress{ 0 };
        std::uintptr_t instanceKeywordDataAddress{ 0 };
        std::uintptr_t objectInstanceExtraAddress{ 0 };
        std::uint64_t instanceContentKey{ 0 };
        std::uint64_t objectIndexDataSignature{ 0 };
        std::uint32_t objectIndexDataCount{ 0 };
        std::uint32_t activeModCount{ 0 };
        std::uint32_t disabledModCount{ 0 };
        bool hasEquippedWeapon{ false };
    };

    struct GeneratedWeaponVisualWitness
    {
        std::uint64_t visualKey{ 0 };
        std::uint64_t generatedSourceSignature{ 0 };
        std::uint64_t generatedGeometryHash{ 0 };
        std::uint64_t generatedBoundsExtentScore{ 0 };
        std::size_t sourceCount{ 0 };
        std::size_t pointCount{ 0 };
        std::size_t childClusterCount{ 0 };
        std::uint32_t semanticPartMask{ 0 };
        std::uint32_t visualRootCount{ 0 };
        std::uint32_t visualNodeCount{ 0 };
        std::uint32_t visualTriShapeCount{ 0 };
        std::uint32_t visibleTriShapeCount{ 0 };
        std::uint32_t missingGeometryCount{ 0 };
        std::uint32_t invisibleNodeCount{ 0 };
        bool hasBuildableSource{ false };
    };

    inline EquippedWeaponInstanceWitness makeEquippedWeaponInstanceWitness(const EquippedWeaponGenerationIdentity& identity)
    {
        EquippedWeaponInstanceWitness witness{};
        if (!identity.hasEquippedWeapon) {
            return witness;
        }

        witness.formID = identity.formID;
        witness.formAddress = identity.formAddress;
        witness.instanceDataAddress = identity.instanceDataAddress;
        witness.instanceKeywordDataAddress = identity.instanceKeywordDataAddress;
        witness.objectInstanceExtraAddress = identity.objectInstanceExtraAddress;
        witness.instanceContentKey = identity.instanceContentKey;
        witness.objectIndexDataSignature = identity.objectIndexDataSignature;
        witness.objectIndexDataCount = identity.objectIndexDataCount;
        witness.activeModCount = identity.activeModCount;
        witness.disabledModCount = identity.disabledModCount;
        witness.hasEquippedWeapon = true;

        std::uint64_t key = weapon_visual_composition_policy::kWeaponVisualCompositionOffset;
        /*
         * This witness deliberately keys stable equipped content instead of the
         * first-person visual tree. It lets generated collision distinguish
         * "the mod list changed but the visible geometry did not" from a real
         * source-package update, preventing ROCK from accepting stale geometry
         * as the completed collider set for the newest modded instance.
         */
        weapon_visual_composition_policy::mixString(key, "ROCKEquippedWeaponInstanceWitnessV1");
        weapon_visual_composition_policy::mixValue(key, identity.formID);
        weapon_visual_composition_policy::mixValue(key, identity.instanceContentKey);
        weapon_visual_composition_policy::mixValue(key, identity.objectIndexDataSignature);
        weapon_visual_composition_policy::mixValue(key, identity.objectIndexDataCount);
        weapon_visual_composition_policy::mixValue(key, identity.activeModCount);
        weapon_visual_composition_policy::mixValue(key, identity.disabledModCount);
        weapon_visual_composition_policy::mixString(key, identity.displayName);
        witness.signature = key;
        return witness;
    }

    inline std::uint64_t makeEquippedWeaponGenerationKey(
        std::uint64_t visualCompositionKey,
        const EquippedWeaponGenerationIdentity& identity)
    {
        if (visualCompositionKey == 0 && !identity.hasEquippedWeapon) {
            return 0;
        }

        std::uint64_t key = weapon_visual_composition_policy::kWeaponVisualCompositionOffset;
        weapon_visual_composition_policy::mixString(key, "ROCKWeaponGenerationV2");
        weapon_visual_composition_policy::mixValue(key, visualCompositionKey);
        weapon_visual_composition_policy::mixValue(key, identity.hasEquippedWeapon ? 1ULL : 0ULL);
        weapon_visual_composition_policy::mixValue(key, identity.formID);
        weapon_visual_composition_policy::mixValue(key, identity.formAddress);
        weapon_visual_composition_policy::mixValue(key, identity.instanceDataAddress);
        weapon_visual_composition_policy::mixValue(key, identity.instanceKeywordDataAddress);
        weapon_visual_composition_policy::mixValue(key, identity.instanceContentKey);
        weapon_visual_composition_policy::mixValue(key, identity.objectIndexDataSignature);
        weapon_visual_composition_policy::mixValue(key, identity.objectIndexDataCount);
        weapon_visual_composition_policy::mixValue(key, identity.activeModCount);
        weapon_visual_composition_policy::mixValue(key, identity.disabledModCount);
        weapon_visual_composition_policy::mixValue(key, identity.equippedDataAddress);
        weapon_visual_composition_policy::mixValue(key, identity.equippedObjectAddress);
        weapon_visual_composition_policy::mixString(key, identity.displayName);
        return key;
    }

    inline std::uint64_t makeEquippedWeaponOwnerKey(const EquippedWeaponGenerationIdentity& identity)
    {
        if (!identity.hasEquippedWeapon) {
            return 0;
        }

        std::uint64_t key = weapon_visual_composition_policy::kWeaponVisualCompositionOffset;
        /*
         * Owner identity is the semantic/content guard used to decide whether a
         * weaker source set may replace live bodies. Keep allocation pointers
         * out of this key; FO4VR can churn equipped-data pointers during reload
         * while the actual weapon/mod content is unchanged.
         */
        weapon_visual_composition_policy::mixString(key, "ROCKWeaponOwnerV1");
        weapon_visual_composition_policy::mixValue(key, identity.formID);
        weapon_visual_composition_policy::mixValue(key, identity.instanceContentKey);
        weapon_visual_composition_policy::mixString(key, identity.displayName);
        return key;
    }
}

// ---- WeaponInstanceVisualWitnessPolicy.h ----

namespace rock::weapon_instance_visual_witness_policy
{
    struct InstanceVisualSyncInput
    {
        bool hasExistingBodies = false;
        bool equippedInstanceChanged = false;
        bool settingsChanged = false;
        bool hasBuildableSource = false;
        weapon_generated_source_completeness_policy::GeneratedSourceCompleteness cachedSource{};
        weapon_generated_source_completeness_policy::GeneratedSourceCompleteness observedSource{};
    };

    struct InstanceVisualSyncDecision
    {
        bool staleVisualSource = false;
        bool keepExistingBodies = false;
        bool pending = false;
        bool requestNextObservation = false;
        const char* reason = "visualSourceFresh";
    };

    inline InstanceVisualSyncDecision evaluateInstanceVisualSync(const InstanceVisualSyncInput& input)
    {
        /*
         * The equipped instance is allowed to lead the first-person visual tree.
         * If the instance/mod witness changes but the generated source witness
         * still matches the cached body bank, rebuilding would only bless stale
         * geometry as current. Keep the existing bodies live and leave the new
         * instance pending until an actual visual source change appears.
         */
        if (!input.hasExistingBodies || !input.equippedInstanceChanged || input.settingsChanged) {
            return InstanceVisualSyncDecision{};
        }

        if (input.cachedSource.sourceCount == 0 || input.cachedSource.signature == 0) {
            return InstanceVisualSyncDecision{};
        }

        if (!input.hasBuildableSource || input.observedSource.sourceCount == 0 || input.observedSource.signature == 0) {
            return InstanceVisualSyncDecision{
                .staleVisualSource = true,
                .keepExistingBodies = true,
                .pending = true,
                .requestNextObservation = true,
                .reason = "instanceChangedNoBuildableVisualSource",
            };
        }

        const bool sameVisualStructure = weapon_generated_source_completeness_policy::sourceSetHasSameVisualStructure(
            input.cachedSource,
            input.observedSource);
        const bool materialVisualChange = weapon_generated_source_completeness_policy::sourceSetHasMaterialDurableGeometryDifference(
            input.cachedSource,
            input.observedSource);
        if (weapon_generated_source_completeness_policy::sourceSetEquivalent(input.cachedSource, input.observedSource) ||
            (sameVisualStructure && !materialVisualChange)) {
            return InstanceVisualSyncDecision{
                .staleVisualSource = true,
                .keepExistingBodies = true,
                .pending = true,
                .requestNextObservation = false,
                .reason = "instanceChangedVisualSourceUnchanged",
            };
        }

        return InstanceVisualSyncDecision{};
    }
}

// ---- WeaponNativeVisualRemapPolicy.h ----

namespace rock::weapon_native_visual_remap_policy
{
    /*
     * Native remap is a task lifecycle, not a delay. ROCK can observe three
     * distinct states after an equipped instance changes: no queueable candidate
     * yet, a queued native attach task whose result has not been observed, and a
     * repeated no-change result after the task ran. Tracking those states keeps
     * the fix narrow while avoiding both per-frame native acquisition and a
     * permanent one-shot dead end for the same pending instance witness.
     */
    struct NativeVisualRemapAttemptState
    {
        std::uint64_t pendingInstanceSignature = 0;
        std::uint32_t queuedAttemptCount = 0;
        std::uint32_t acquireFailureFrames = 0;
        std::uint32_t staleFramesAfterQueuedRequest = 0;
        bool queuedAwaitingVisualChange = false;
        bool acquireFailureBackoff = false;
    };

    struct NativeVisualRemapInput
    {
        bool enabled = false;
        bool authorizedWitness = true;
        bool staleVisualSource = false;
        bool equippedInstanceChanged = false;
        std::uint64_t pendingInstanceSignature = 0;
        std::uint64_t lastRequestedInstanceSignature = 0;
        NativeVisualRemapAttemptState attemptState{};
        std::uint32_t acquireRetryFrameInterval = 4;
        std::uint32_t queuedObservationFrameInterval = 2;
        std::uint32_t maxQueuedAttemptsPerWitness = 2;
    };

    struct NativeVisualRemapDecision
    {
        bool requestRemap = false;
        const char* reason = "visualSourceFresh";
    };

    struct NativeVisualRemapTargetWitness
    {
        std::uint32_t formID{ 0 };
        std::uintptr_t formAddress{ 0 };
        std::uintptr_t instanceDataAddress{ 0 };
        std::uintptr_t objectInstanceExtraAddress{ 0 };
    };

    struct NativeVisualRemapTargetMatchDecision
    {
        bool matches = false;
        const char* reason = "targetMissing";
    };

    inline NativeVisualRemapDecision evaluateNativeVisualRemap(const NativeVisualRemapInput& input)
    {
        /*
         * FO4VR can update the equipped instance before the first-person weapon
         * attach tree reflects the last manual mod change. ROCK only asks the
         * native queued attach path to remap the current equipped instance when
         * a visible generated weapon source is present but stale; a missing
         * first-person visual root is an equip/draw transition and must stay
         * outside this remap policy. The attempt state keeps stale-visible
         * recovery from becoming either an unbounded native spam loop or a
         * one-shot latch that can never recover when the first queued task
         * leaves the visible tree stale.
         */
        if (!input.staleVisualSource) {
            return NativeVisualRemapDecision{};
        }

        if (!input.enabled) {
            return NativeVisualRemapDecision{
                .requestRemap = false,
                .reason = "nativeVisualRemapDisabled",
            };
        }

        if (!input.authorizedWitness) {
            return NativeVisualRemapDecision{
                .requestRemap = false,
                .reason = "nativeVisualRemapAwaitingAuthorizedWitness",
            };
        }

        if (!input.equippedInstanceChanged || input.pendingInstanceSignature == 0) {
            return NativeVisualRemapDecision{
                .requestRemap = false,
                .reason = "nativeVisualRemapMissingPendingWitness",
            };
        }

        const bool sameTrackedWitness =
            input.attemptState.pendingInstanceSignature == input.pendingInstanceSignature;

        if (sameTrackedWitness) {
            if (input.attemptState.queuedAwaitingVisualChange &&
                input.attemptState.staleFramesAfterQueuedRequest < input.queuedObservationFrameInterval) {
                return NativeVisualRemapDecision{
                    .requestRemap = false,
                    .reason = "nativeVisualRemapAwaitingQueuedResult",
                };
            }

            if (input.attemptState.queuedAttemptCount >= input.maxQueuedAttemptsPerWitness &&
                !input.attemptState.acquireFailureBackoff) {
                return NativeVisualRemapDecision{
                    .requestRemap = false,
                    .reason = "nativeVisualRemapExhaustedForInstance",
                };
            }

            if (input.attemptState.acquireFailureBackoff &&
                input.attemptState.acquireFailureFrames < input.acquireRetryFrameInterval) {
                return NativeVisualRemapDecision{
                    .requestRemap = false,
                    .reason = "nativeVisualRemapAcquireBackoff",
                };
            }

            return NativeVisualRemapDecision{
                .requestRemap = true,
                .reason = input.attemptState.acquireFailureBackoff ? "nativeVisualRemapAcquireRetry" :
                                                                     "nativeVisualRemapQueuedRetry",
            };
        }

        if (input.pendingInstanceSignature == input.lastRequestedInstanceSignature) {
            return NativeVisualRemapDecision{
                .requestRemap = false,
                .reason = "nativeVisualRemapAlreadyRequestedForInstance",
            };
        }

        return NativeVisualRemapDecision{
            .requestRemap = true,
            .reason = "instanceChangedVisualSourceStaleNativeRemap",
        };
    }

    inline NativeVisualRemapAttemptState observeNativeVisualRemapStillStale(
        NativeVisualRemapAttemptState state,
        std::uint64_t pendingInstanceSignature)
    {
        if (pendingInstanceSignature == 0 || state.pendingInstanceSignature != pendingInstanceSignature) {
            return state;
        }

        if (state.queuedAwaitingVisualChange) {
            ++state.staleFramesAfterQueuedRequest;
        } else if (state.acquireFailureBackoff) {
            ++state.acquireFailureFrames;
        }
        return state;
    }

    inline NativeVisualRemapAttemptState recordNativeVisualRemapQueued(
        NativeVisualRemapAttemptState state,
        std::uint64_t pendingInstanceSignature)
    {
        if (pendingInstanceSignature == 0) {
            return NativeVisualRemapAttemptState{};
        }

        if (state.pendingInstanceSignature != pendingInstanceSignature) {
            state = NativeVisualRemapAttemptState{};
            state.pendingInstanceSignature = pendingInstanceSignature;
        }

        ++state.queuedAttemptCount;
        state.acquireFailureFrames = 0;
        state.staleFramesAfterQueuedRequest = 0;
        state.queuedAwaitingVisualChange = true;
        state.acquireFailureBackoff = false;
        return state;
    }

    inline NativeVisualRemapAttemptState recordNativeVisualRemapAcquireFailed(
        NativeVisualRemapAttemptState state,
        std::uint64_t pendingInstanceSignature)
    {
        if (pendingInstanceSignature == 0) {
            return NativeVisualRemapAttemptState{};
        }

        if (state.pendingInstanceSignature != pendingInstanceSignature) {
            state = NativeVisualRemapAttemptState{};
            state.pendingInstanceSignature = pendingInstanceSignature;
        }

        state.acquireFailureFrames = 0;
        state.staleFramesAfterQueuedRequest = 0;
        state.queuedAwaitingVisualChange = false;
        state.acquireFailureBackoff = true;
        return state;
    }

    inline NativeVisualRemapTargetMatchDecision evaluateNativeVisualRemapTargetMatch(
        const NativeVisualRemapTargetWitness& expected,
        const NativeVisualRemapTargetWitness& observed)
    {
        /*
         * The native visual remap is narrow only if ROCK queues the same
         * equipped instance that produced the stale-source witness. Treat zero expected
         * addresses as unavailable evidence, but every nonzero witness field
         * must match the currently equipped weapon candidate before the native
         * task can be queued.
         */
        if (observed.formID == 0 || observed.formAddress == 0) {
            return NativeVisualRemapTargetMatchDecision{
                .matches = false,
                .reason = "targetMissingWeapon",
            };
        }

        if (expected.formID != 0 && observed.formID != expected.formID) {
            return NativeVisualRemapTargetMatchDecision{
                .matches = false,
                .reason = "targetFormIDMismatch",
            };
        }

        if (expected.formAddress != 0 && observed.formAddress != expected.formAddress) {
            return NativeVisualRemapTargetMatchDecision{
                .matches = false,
                .reason = "targetFormAddressMismatch",
            };
        }

        if (expected.instanceDataAddress != 0 && observed.instanceDataAddress != expected.instanceDataAddress) {
            return NativeVisualRemapTargetMatchDecision{
                .matches = false,
                .reason = "targetInstanceDataMismatch",
            };
        }

        if (expected.objectInstanceExtraAddress != 0 && observed.objectInstanceExtraAddress != expected.objectInstanceExtraAddress) {
            return NativeVisualRemapTargetMatchDecision{
                .matches = false,
                .reason = "targetObjectInstanceExtraMismatch",
            };
        }

        return NativeVisualRemapTargetMatchDecision{
            .matches = true,
            .reason = "targetMatched",
        };
    }
}
