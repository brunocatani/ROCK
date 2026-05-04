#pragma once

#include <cstdint>
#include <cstddef>

namespace frik::rock::weapon_authority_lifecycle_policy
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
