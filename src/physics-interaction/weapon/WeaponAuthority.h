#pragma once

/*
 * Weapon authority helpers are grouped here because lifecycle, visual authority, muzzle authority, and visual composition all decide final weapon ownership.
 */


// ---- WeaponAuthorityLifecyclePolicy.h ----

#include <cstdint>
#include <cstddef>

namespace rock::weapon_authority_lifecycle_policy
{
    /*
     * HIGGS stops advancing hand interaction when game-stopping menus or world
     * interruptions are active, but it also makes owned interaction state yield
     * before returning to the game. ROCK keeps FRIK weapon authority, input
     * blocking, and left-hand collision suppression in separate systems, so the
     * interruption decision is centralized here to keep all three lifetimes in
     * lockstep.
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

    inline GeneratedCollisionRebuildDecision evaluateGeneratedCollisionRebuild(const GeneratedCollisionRebuildInput& input)
    {
        const bool identityChanged = input.currentKey != 0 && input.currentKey != input.cachedKey;
        const bool missingInitialBodies = !input.hasExistingBodies && input.currentKey != 0;
        const bool needsRebuild = missingInitialBodies || identityChanged || input.settingsChanged;
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
            .reason = input.hasExistingBodies ? "replacementReady" : "initialSourcesReady",
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
     * FRIK-style node write conventions across two-handed grip, one-handed mesh
     * grip, and debug verification paths. Two-handed support also needs a
     * HIGGS-style locked hand frame: controllers guide the weapon solve, while
     * visible hands are recomposed from stored weapon-local frames so the mesh
     * contact point cannot slide along the gun.
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
