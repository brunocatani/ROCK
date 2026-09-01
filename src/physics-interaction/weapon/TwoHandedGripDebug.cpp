#include "physics-interaction/weapon/TwoHandedGrip.h"
#include "physics-interaction/weapon/TwoHandedGripInternal.h"

#include "api/ROCKProviderApiInternal.h"
#include "physics-interaction/actor/ActorEquipmentGrab.h"
#include "physics-interaction/animation/AuthoredWeaponGripCapturePolicy.h"
#include "physics-interaction/hand/HandSkeleton.h"
#include "physics-interaction/hand/HandVisual.h"
#include "physics-interaction/grab/GrabFinger.h"
#include "physics-interaction/grab/GrabPinchPocket.h"
#include "physics-interaction/hand/HandFrame.h"
#include "physics-interaction/core/RockRuntimeState.h"
#include "physics-interaction/performance/PerformanceProfiler.h"
#include "RockConfig.h"
#include "RockUtils.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/weapon/AuthoredWeaponGripLibrary.h"
#include "physics-interaction/weapon/DynamicWeaponCollisionPolicy.h"
#include "physics-interaction/weapon/EquippedWeaponHandlingRuntime.h"
#include "physics-interaction/weapon/NativeScopeSightAnchorPolicy.h"
#include "physics-interaction/weapon/WeaponAuthority.h"
#include "physics-interaction/weapon/WeaponCollision.h"
#include "physics-interaction/weapon/WeaponGeometry.h"
#include "physics-interaction/weapon/WeaponSupport.h"
#include "rock_support/Fo4VrRuntime.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <span>
#include <string_view>
#include <vector>

namespace rock
{
    using namespace two_handed_grip_internal;

    void TwoHandedGrip::traceNativeScopeTransitionFinalState(RE::NiNode* weaponNode)
    {
        if (!_scope.transitionFinalTracePending) {
            return;
        }
        _scope.transitionFinalTracePending = false;

        struct HmdRelativeTrace
        {
            RE::NiPoint3 position{};
            bool valid{ false };
        };

        const auto* playerNodes = f4vr::getPlayerNodes();
        RE::NiTransform hmdWorld{};
        const bool hmdWorldValid =
            playerNodes && playerNodes->HmdNode &&
            isFiniteTransform(playerNodes->HmdNode->world);
        if (hmdWorldValid) {
            hmdWorld = playerNodes->HmdNode->world;
        }

        const auto captureWorld = [&hmdWorld, hmdWorldValid](
                                      const RE::NiTransform& world,
                                      const bool worldValid) {
            HmdRelativeTrace trace{};
            if (!hmdWorldValid || !worldValid) {
                return trace;
            }
            trace.position = transform_math::worldPointToLocal(
                hmdWorld,
                world.translate);
            trace.valid = std::isfinite(trace.position.x) &&
                          std::isfinite(trace.position.y) &&
                          std::isfinite(trace.position.z);
            return trace;
        };
        const auto captureNode = [&captureWorld](const RE::NiAVObject* node) {
            return captureWorld(
                node ? node->world : RE::NiTransform{},
                node && isFiniteTransform(node->world));
        };

        RE::NiTransform scopeCameraWorld{};
        bool scopeCameraWorldValid = false;
        if (playerNodes && playerNodes->primaryWeaponScopeCamera) {
            const auto* scopeCamera = playerNodes->primaryWeaponScopeCamera;
            scopeCameraWorld = scopeCamera->world;
            if (scopeCamera->parent &&
                isFiniteTransform(scopeCamera->parent->world) &&
                isFiniteTransform(scopeCamera->local)) {
                scopeCameraWorld = transform_math::composeTransforms(
                    scopeCamera->parent->world,
                    scopeCamera->local);
            }
            scopeCameraWorldValid = isFiniteTransform(scopeCameraWorld);
        }

        RE::NiTransform leftRootWorld{};
        RE::NiTransform rightRootWorld{};
        const bool leftRootWorldValid =
            tryGetRootFlattenedHandBoneTransform(true, leftRootWorld);
        const bool rightRootWorldValid =
            tryGetRootFlattenedHandBoneTransform(false, rightRootWorld);
        const auto* playerCamera = f4vr::getPlayerCamera();
        const bool cameraValuesValid =
            playerCamera &&
            std::isfinite(playerCamera->zoomInput) &&
            std::isfinite(playerCamera->worldFOV) &&
            std::isfinite(playerCamera->firstPersonFOV) &&
            std::isfinite(playerCamera->fovAdjustCurrent) &&
            std::isfinite(playerCamera->fovAdjustTarget) &&
            std::isfinite(playerCamera->fovAdjustPerSec) &&
            std::isfinite(playerCamera->fovAnimatorAdjust);

        const HmdRelativeTrace weaponTrace = captureWorld(
            weaponNode ? weaponNode->world : RE::NiTransform{},
            weaponNode && isFiniteTransform(weaponNode->world));
        const HmdRelativeTrace leftRootTrace = captureWorld(
            leftRootWorld,
            leftRootWorldValid);
        const HmdRelativeTrace rightRootTrace = captureWorld(
            rightRootWorld,
            rightRootWorldValid);
        const HmdRelativeTrace scopeCameraTrace = captureWorld(
            scopeCameraWorld,
            scopeCameraWorldValid);
        const HmdRelativeTrace scopeParentTrace = captureNode(
            playerNodes ? playerNodes->ScopeParentNode : nullptr);
        const HmdRelativeTrace cameraRootTrace = captureNode(
            playerCamera ? playerCamera->cameraRoot.get() : nullptr);
        const HmdRelativeTrace roomTrace = captureNode(
            playerNodes ? playerNodes->roomnode : nullptr);
        const HmdRelativeTrace uprightHmdTrace = captureNode(
            playerNodes ? playerNodes->UprightHmdNode : nullptr);
        const HmdRelativeTrace skeletonRootTrace =
            captureNode(f4vr::getRootNode());

        ROCK_LOG_INFO(Weapon,
            "SCOPE-VIEW-FINAL seq={} sample={}/{} buttonRequested={} rendererActive={} menuOpen={} driverAuthority={} state={} hmdValid={} hmdWorld=({:.2f},{:.2f},{:.2f}) weaponHmdValid={} weaponHmd=({:.2f},{:.2f},{:.2f}) leftRootHmdValid={} leftRootHmd=({:.2f},{:.2f},{:.2f}) rightRootHmdValid={} rightRootHmd=({:.2f},{:.2f},{:.2f}) scopeCameraHmdValid={} scopeCameraHmd=({:.2f},{:.2f},{:.2f}) scopeParentHmdValid={} scopeParentHmd=({:.2f},{:.2f},{:.2f}) cameraRootHmdValid={} cameraRootHmd=({:.2f},{:.2f},{:.2f}) roomHmdValid={} roomHmd=({:.2f},{:.2f},{:.2f}) uprightHmdValid={} uprightHmd=({:.2f},{:.2f},{:.2f}) skeletonRootHmdValid={} skeletonRootHmd=({:.2f},{:.2f},{:.2f}) cameraValuesValid={} zoomInput={:.4f} worldFov={:.4f} firstPersonFov={:.4f} fovAdjust=({:.4f},{:.4f},{:.4f},{:.4f})",
            _scope.transitionFinalTraceSequence,
            _scope.transitionFinalTraceSample,
            SCOPE_TRANSITION_TRACE_FRAMES,
            _scope.manualActivationRequested ? "yes" : "no",
            _scope.nativeRequestActive ? "yes" : "no",
            _scope.menuOpenThisFrame ? "yes" : "no",
            _scope.driverFrameAuthorityActive ? "yes" : "no",
            static_cast<std::uint32_t>(_session.state),
            hmdWorldValid ? "yes" : "no",
            hmdWorld.translate.x,
            hmdWorld.translate.y,
            hmdWorld.translate.z,
            weaponTrace.valid ? "yes" : "no",
            weaponTrace.position.x,
            weaponTrace.position.y,
            weaponTrace.position.z,
            leftRootTrace.valid ? "yes" : "no",
            leftRootTrace.position.x,
            leftRootTrace.position.y,
            leftRootTrace.position.z,
            rightRootTrace.valid ? "yes" : "no",
            rightRootTrace.position.x,
            rightRootTrace.position.y,
            rightRootTrace.position.z,
            scopeCameraTrace.valid ? "yes" : "no",
            scopeCameraTrace.position.x,
            scopeCameraTrace.position.y,
            scopeCameraTrace.position.z,
            scopeParentTrace.valid ? "yes" : "no",
            scopeParentTrace.position.x,
            scopeParentTrace.position.y,
            scopeParentTrace.position.z,
            cameraRootTrace.valid ? "yes" : "no",
            cameraRootTrace.position.x,
            cameraRootTrace.position.y,
            cameraRootTrace.position.z,
            roomTrace.valid ? "yes" : "no",
            roomTrace.position.x,
            roomTrace.position.y,
            roomTrace.position.z,
            uprightHmdTrace.valid ? "yes" : "no",
            uprightHmdTrace.position.x,
            uprightHmdTrace.position.y,
            uprightHmdTrace.position.z,
            skeletonRootTrace.valid ? "yes" : "no",
            skeletonRootTrace.position.x,
            skeletonRootTrace.position.y,
            skeletonRootTrace.position.z,
            cameraValuesValid ? "yes" : "no",
            cameraValuesValid ? playerCamera->zoomInput : 0.0f,
            cameraValuesValid ? playerCamera->worldFOV : 0.0f,
            cameraValuesValid ? playerCamera->firstPersonFOV : 0.0f,
            cameraValuesValid ? playerCamera->fovAdjustCurrent : 0.0f,
            cameraValuesValid ? playerCamera->fovAdjustTarget : 0.0f,
            cameraValuesValid ? playerCamera->fovAdjustPerSec : 0.0f,
            cameraValuesValid ? playerCamera->fovAnimatorAdjust : 0.0f);
    }

    bool TwoHandedGrip::getDebugAuthoritySnapshot(TwoHandedGripDebugSnapshot& outSnapshot) const
    {
        const auto& leftGrip = partGrip(true);
        const auto& rightGrip = partGrip(false);
        if (!_hasSolvedWeaponTransform || !_session.weaponNode) {
            return false;
        }
        if (!_firing.hasPrimaryHandWeaponLocal && !leftGrip.active && !rightGrip.active) {
            return false;
        }

        outSnapshot.weaponWorld = _lastSolvedWeaponTransform;
        if (rightGrip.active) {
            outSnapshot.rightRequestedHandWorld = resolvePartGripHandWorld(rightGrip, _session.weaponNode);
            outSnapshot.rightGripWorld = resolvePartGripWorld(rightGrip, _session.weaponNode);
        } else {
            outSnapshot.rightRequestedHandWorld = transform_math::composeTransforms(_lastSolvedWeaponTransform, _firing.primaryHandWeaponLocal);
            outSnapshot.rightGripWorld = transform_math::localPointToWorld(_lastSolvedWeaponTransform, _firing.primaryGripLocal);
        }
        if (leftGrip.active) {
            outSnapshot.leftRequestedHandWorld = resolvePartGripHandWorld(leftGrip, _session.weaponNode);
            outSnapshot.leftGripWorld = resolvePartGripWorld(leftGrip, _session.weaponNode);
        } else {
            outSnapshot.leftRequestedHandWorld = RE::NiTransform{};
            outSnapshot.leftGripWorld = RE::NiPoint3{};
        }
        return true;
    }

    bool TwoHandedGrip::getAuthoredSupportGripDebugSnapshot(
        AuthoredSupportGripDebugSnapshot& outSnapshot) const
    {
        outSnapshot = _support.authoredDebugSnapshot;
        return outSnapshot.valid;
    }
}
