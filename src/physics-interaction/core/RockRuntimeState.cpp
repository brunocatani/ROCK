#include "physics-interaction/core/RockRuntimeState.h"

#include "RockConfig.h"
#include "physics-interaction/core/RockRuntimeStatePolicy.h"
#include "physics-interaction/debug/SkeletonBoneDebugMath.h"
#include "physics-interaction/hand/HandSkeleton.h"
#include "physics-interaction/timing/RockGameTiming.h"

#include <string_view>

#include "RE/Bethesda/PlayerCharacter.h"
#include "rock_support/Fo4VrRuntime.h"
#include "rock_support/GameMenus.h"

namespace rock::runtime_state
{
    namespace
    {
        constexpr int kMaxFlattenedBoneTransforms = 768;

        f4vr::GameMenusHandler s_gameMenus;
        bool s_menuHandlerInitialized = false;
        RuntimeFrameSnapshot s_snapshot{};

        /*
         * Menu state is sampled exactly once per frame, in beginFrameTiming,
         * so the timing snapshot's pause flag and the runtime snapshot's menu
         * flags describe the same instant. updateFrame consumes the sample.
         */
        struct FrameMenuSample
        {
            bool valid = false;
            bool inputMenuBlocking = false;
            bool scopeMenuOpen = false;
            bool loadingMenuOpen = false;
            bool gameStopped = false;
        };
        FrameMenuSample s_frameMenuSample{};
        runtime_state_policy::PlayerSpaceTrackerState s_playerSpaceTracker{};
        DirectSkeletonBoneReader s_skeletonReader;

        [[nodiscard]] runtime_state_policy::Vec3 toPolicyVec(const RE::NiPoint3& value)
        {
            return runtime_state_policy::Vec3{
                .x = value.x,
                .y = value.y,
                .z = value.z,
            };
        }

        [[nodiscard]] RE::NiPoint3 fromPolicyVec(const runtime_state_policy::Vec3& value)
        {
            return RE::NiPoint3(value.x, value.y, value.z);
        }

        [[nodiscard]] bool hasPlayer()
        {
            return RE::PlayerCharacter::GetSingleton() != nullptr;
        }

        [[nodiscard]] RE::NiNode* safeWorldRootNode()
        {
            return f4vr::getWorldRootNode();
        }

        [[nodiscard]] RE::NiNode* safeRootNode(RE::NiNode* worldRoot)
        {
            if (!worldRoot || worldRoot->children.empty() || !worldRoot->children[0]) {
                return nullptr;
            }

            return worldRoot->children[0]->IsNode();
        }

        [[nodiscard]] bool sampleWeaponDrawn()
        {
            if (!hasPlayer()) {
                return false;
            }

            return f4vr::IsWeaponDrawn();
        }

        [[nodiscard]] bool flattenedTreeValid(const f4vr::BSFlattenedBoneTree* tree)
        {
            return tree && tree->transforms && tree->numTransforms > 0 && tree->numTransforms <= kMaxFlattenedBoneTransforms;
        }

        [[nodiscard]] bool snapshotHasBone(const DirectSkeletonBoneSnapshot& snapshot, std::string_view name)
        {
            for (const auto& bone : snapshot.bones) {
                if (bone.name == name) {
                    return true;
                }
            }

            return false;
        }

        [[nodiscard]] bool snapshotHasRequiredFingerBones(const DirectSkeletonBoneSnapshot& snapshot)
        {
            for (const auto name : skeleton_bone_debug_math::requiredFingerBoneNames()) {
                if (!snapshotHasBone(snapshot, name)) {
                    return false;
                }
            }

            return true;
        }

        [[nodiscard]] PlayerSpaceFrame samplePlayerSpace()
        {
            PlayerSpaceFrame frame{};
            if (!hasPlayer()) {
                s_playerSpaceTracker = {};
                return frame;
            }

            if (auto* playerNodes = f4vr::getPlayerNodes(); playerNodes && playerNodes->roomnode) {
                frame.valid = true;
                frame.source = "roomNode";
                frame.world = playerNodes->roomnode->world;
            } else if (auto* worldRoot = safeWorldRootNode()) {
                frame.valid = true;
                frame.source = "worldRoot";
                frame.world = worldRoot->world;
            }

            const auto decision = runtime_state_policy::updatePlayerSpaceTracker(
                s_playerSpaceTracker,
                runtime_state_policy::PlayerSpaceTrackerInput{
                    .positionValid = frame.valid,
                    .currentPosition = toPolicyVec(frame.world.translate),
                });
            frame.moving = decision.moving;
            frame.deltaGameUnits = fromPolicyVec(decision.deltaGameUnits);
            return frame;
        }

        [[nodiscard]] bool sampleLocalSkeletonReady(RuntimeFrameSnapshot& snapshot)
        {
            auto readinessInput = runtime_state_policy::SkeletonReadinessInput{
                .playerAvailable = snapshot.playerAvailable,
            };

            if (!snapshot.playerAvailable) {
                s_skeletonReader.resetCache();
                return runtime_state_policy::evaluateSkeletonReadiness(readinessInput);
            }

            auto* worldRoot = safeWorldRootNode();
            auto* rootNode = safeRootNode(worldRoot);
            auto* flattenedTree = rootNode ? reinterpret_cast<f4vr::BSFlattenedBoneTree*>(rootNode) : nullptr;

            snapshot.localSkeletonRootAttached = rootNode && rootNode->parent;
            readinessInput.rootNodeAvailable = worldRoot != nullptr && rootNode != nullptr;
            readinessInput.rootParentAttached = snapshot.localSkeletonRootAttached;
            readinessInput.flattenedTreeValid = flattenedTreeValid(flattenedTree);

            if (!readinessInput.rootNodeAvailable || !readinessInput.rootParentAttached || !readinessInput.flattenedTreeValid) {
                s_skeletonReader.resetCache();
                return runtime_state_policy::evaluateSkeletonReadiness(readinessInput);
            }

            DirectSkeletonBoneSnapshot boneSnapshot{};
            const bool captured = s_skeletonReader.capture(
                skeleton_bone_debug_math::DebugSkeletonBoneMode::HandsAndForearmsOnly,
                skeleton_bone_debug_math::DebugSkeletonBoneSource::GameRootFlattenedBoneTree,
                SkeletonBoneCaptureSpace::Rendered,
                boneSnapshot);

            const bool hasHands = captured && snapshotHasBone(boneSnapshot, "RArm_Hand") && snapshotHasBone(boneSnapshot, "LArm_Hand");
            const bool handBonesReady =
                hasHands &&
                snapshotHasRequiredFingerBones(boneSnapshot);

            snapshot.localSkeletonRequiredHandBonesReady = handBonesReady;
            readinessInput.requiredHandBonesResolved = handBonesReady;

            return runtime_state_policy::evaluateSkeletonReadiness(readinessInput);
        }
    }

    void initialize()
    {
        if (s_menuHandlerInitialized) {
            return;
        }

        s_gameMenus.init();
        s_menuHandlerInitialized = true;
    }

    void resetTransientState()
    {
        game_timing::resetForNewSession();
        s_playerSpaceTracker = {};
        s_skeletonReader.resetCache();
        s_snapshot = {};
        s_frameMenuSample = {};
    }

    const game_frame_timing_policy::GameFrameTiming& beginFrameTiming(const bool menuInputBlocking)
    {
        s_frameMenuSample.valid = true;
        s_frameMenuSample.inputMenuBlocking = menuInputBlocking;
        s_frameMenuSample.scopeMenuOpen = isScopeMenuOpenNow();
        s_frameMenuSample.loadingMenuOpen = s_menuHandlerInitialized && s_gameMenus.isLoadingMenuOpen();
        s_frameMenuSample.gameStopped = s_menuHandlerInitialized && s_gameMenus.isGameStopped();
        return game_timing::beginGameFrame(s_frameMenuSample.gameStopped || menuInputBlocking);
    }

    void updateFrame(const RuntimeFrameInput& input)
    {
        if (!s_frameMenuSample.valid) {
            // The game-loop hook begins frame timing before any phase; this
            // fail-closed path only protects an out-of-order caller from
            // silently reusing a stale frame identity.
            (void)beginFrameTiming(false);
        }

        RuntimeFrameSnapshot next{};
        next.playerAvailable = hasPlayer();
        next.weaponDrawn = sampleWeaponDrawn();
        next.inputMenuBlocking = s_frameMenuSample.inputMenuBlocking;
        next.localScopeMenuOpen = s_frameMenuSample.scopeMenuOpen;
        next.localLoadingMenuOpen = s_frameMenuSample.loadingMenuOpen;
        next.localGameStopped = s_frameMenuSample.gameStopped;
        next.localMenuBlocking = next.localGameStopped || next.inputMenuBlocking;
        next.timing = game_timing::currentFrameTiming();
        next.frameIndex = next.timing.sequence;
        /*
         * Convenience copy of the sanitized measured delta. Zero for an
         * unmeasurable frame — every consumer holds on zero elapsed time; a
         * hitch frame carries the clamped bounded delta with
         * timing.discontinuity set for estimators that must rebase.
         */
        next.deltaSeconds = next.timing.valid ? next.timing.deltaSeconds : 0.0f;
        next.compatibilityConfigBlocking = input.compatibilityConfigBlocking;
        next.visualAuthorityAvailable = input.visualAuthorityAvailable;
        next.visualSkeletonReadyHint = input.visualSkeletonReadyHint;
        next.playerSpace = samplePlayerSpace();
        next.localSkeletonReady = sampleLocalSkeletonReady(next);
        s_frameMenuSample.valid = false;
        s_snapshot = next;
    }

    const RuntimeFrameSnapshot& currentFrame()
    {
        return s_snapshot;
    }

    bool isScopeMenuOpenNow()
    {
        return s_menuHandlerInitialized && s_gameMenus.isInScopeMenu();
    }

    bool isLocalSkeletonReady()
    {
        return s_snapshot.localSkeletonReady;
    }

    bool isPhysicsMenuBlocked()
    {
        return s_snapshot.localMenuBlocking;
    }

    bool isCompatibilityConfigBlocked()
    {
        return s_snapshot.compatibilityConfigBlocking;
    }
}
