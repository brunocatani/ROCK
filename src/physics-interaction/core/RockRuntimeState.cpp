#include "physics-interaction/core/RockRuntimeState.h"

#include "RockConfig.h"
#include "physics-interaction/core/RockRuntimeStatePolicy.h"
#include "physics-interaction/debug/SkeletonBoneDebugMath.h"
#include "physics-interaction/hand/HandSkeleton.h"

#include <chrono>
#include <string_view>

#include "RE/Bethesda/PlayerCharacter.h"
#include "f4vr/F4VRUtils.h"
#include "f4vr/GameMenusHandler.h"
#include "f4vr/PlayerNodes.h"

namespace rock::runtime_state
{
    namespace
    {
        constexpr int kMaxFlattenedBoneTransforms = 768;

        f4cf::f4vr::GameMenusHandler s_gameMenus;
        bool s_menuHandlerInitialized = false;
        bool s_hasLastFrameTime = false;
        std::chrono::steady_clock::time_point s_lastFrameTime{};
        RuntimeFrameSnapshot s_snapshot{};
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

        [[nodiscard]] float sampleFrameDeltaSeconds()
        {
            const auto now = std::chrono::steady_clock::now();
            if (!s_hasLastFrameTime) {
                s_hasLastFrameTime = true;
                s_lastFrameTime = now;
                return runtime_state_policy::kFallbackDeltaSeconds;
            }

            const std::chrono::duration<float> elapsed = now - s_lastFrameTime;
            s_lastFrameTime = now;
            return runtime_state_policy::sanitizeFrameDelta(elapsed.count());
        }

        [[nodiscard]] bool hasPlayer()
        {
            return RE::PlayerCharacter::GetSingleton() != nullptr;
        }

        [[nodiscard]] RE::NiNode* safeWorldRootNode()
        {
            auto* player = f4cf::f4vr::getPlayer();
            if (!player || !player->unkF0) {
                return nullptr;
            }

            return player->unkF0->rootNode;
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

            return f4cf::f4vr::IsWeaponDrawn();
        }

        [[nodiscard]] bool flattenedTreeValid(const f4cf::f4vr::BSFlattenedBoneTree* tree)
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

            if (auto* playerNodes = f4cf::f4vr::getPlayerNodes(); playerNodes && playerNodes->roomnode) {
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
            auto* flattenedTree = rootNode ? reinterpret_cast<f4cf::f4vr::BSFlattenedBoneTree*>(rootNode) : nullptr;

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
                boneSnapshot);

            const bool hasHands = captured && snapshotHasBone(boneSnapshot, "RArm_Hand") && snapshotHasBone(boneSnapshot, "LArm_Hand");
            const bool handBonesReady =
                hasHands &&
                (!g_rockConfig.rockHandBoneCollidersRequireAllFingerBones || snapshotHasRequiredFingerBones(boneSnapshot));

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
        s_hasLastFrameTime = false;
        s_lastFrameTime = {};
        s_playerSpaceTracker = {};
        s_skeletonReader.resetCache();
        s_snapshot = {};
    }

    void updateFrame(const RuntimeFrameInput& input)
    {
        RuntimeFrameSnapshot next{};
        next.frameIndex = s_snapshot.frameIndex + 1;
        next.deltaSeconds = sampleFrameDeltaSeconds();
        next.playerAvailable = hasPlayer();
        next.weaponDrawn = sampleWeaponDrawn();
        next.inputMenuBlocking = input.menuInputBlocking;
        next.localLoadingMenuOpen = s_menuHandlerInitialized && s_gameMenus.isLoadingMenuOpen();
        next.localGameStopped = s_menuHandlerInitialized && s_gameMenus.isGameStopped();
        next.localMenuBlocking = next.localGameStopped || next.inputMenuBlocking;
        next.compatibilityConfigBlocking = input.compatibilityConfigBlocking;
        next.visualAuthorityAvailable = input.visualAuthorityAvailable;
        next.visualSkeletonReadyHint = input.visualSkeletonReadyHint;
        next.playerSpace = samplePlayerSpace();
        next.localSkeletonReady = sampleLocalSkeletonReady(next);
        s_snapshot = next;
    }

    const RuntimeFrameSnapshot& currentFrame()
    {
        return s_snapshot;
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

    float deltaSeconds()
    {
        return s_snapshot.deltaSeconds;
    }
}
