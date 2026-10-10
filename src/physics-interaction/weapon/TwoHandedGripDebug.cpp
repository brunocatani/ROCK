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
#include <chrono>
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

    void TwoHandedGrip::beginGunstockDebug(RE::NiNode* weaponNode, float dt,
        std::uint64_t generation, std::uint64_t ownership, std::uint32_t form)
    {
        using namespace gunstock_debug;
        _gunstockSample = {};
        if (!g_rockConfig.rockDebugDrawGunstock) {
            _gunstockWasSupporting = false;
            _gunstockGrabSample = {};
            return;
        }
        auto& s = _gunstockSample;
        s.frame = runtime_state::currentFrame().frameIndex;
        s.tick = static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
        s.generation = generation; s.ownership = ownership; s.form = form; s.dt = dt;
        if (weaponNode) setPose(s, WeaponBefore, weaponNode->world);
        for (bool left : {true, false}) {
            RE::NiTransform pose{};
            if (frik_visual_authority::tryGetTrackedHandTransform(handFromBool(left), frik_visual_authority::TrackedHandKind::Wand, pose))
                setPose(s, left ? LeftController : RightController, pose);
            if (frik_hand_world_authority::tryGetInputDriverWorld(left, pose))
                setPose(s, left ? LeftDriver : RightDriver, pose);
            if (frik_hand_world_authority::tryGetRawHandWorld(left, pose))
                setPose(s, left ? LeftPhysical : RightPhysical, pose);
        }
        const auto* camera = f4vr::getPlayerCamera();
        if (camera && camera->cameraRoot) setPose(s, Camera, camera->cameraRoot->world);
    }

    void TwoHandedGrip::finishGunstockDebug(RE::NiNode* weaponNode)
    {
        using namespace gunstock_debug;
        if (!g_rockConfig.rockDebugDrawGunstock) return;
        auto& s = _gunstockSample;
        s.state = static_cast<std::uint32_t>(_session.state);
        s.phase = 1; // End of grip update, before final collision/FRIK presentation.
        s.firingLeft = isFiringHandLeft();
        const auto& support = partGrip(isSupportHandLeft());
        s.supportActive = support.active;
        s.grip = support.active ? support.gripSequence : 0;
        s.authored = support.authoredSupportGrip;
        s.part = static_cast<std::uint32_t>(support.partKind);
        s.omod = support.omodFormId; s.attachPoint = support.attachPointFormId;
        std::copy_n(support.sourceName.data(), (std::min)(support.sourceName.size(), sizeof(s.source) - 1), s.source);
        if (weaponNode) {
            setPose(s, WeaponAfter, weaponNode->world);
            if (_firing.hasPrimaryHandWeaponLocal) {
                auto seat = transform_math::composeTransforms(weaponNode->world, _firing.primaryHandWeaponLocal);
                seat.translate = transform_math::localPointToWorld(weaponNode->world, _firing.primaryGripLocal);
                setPose(s, PrimarySeat, seat);
            }
            if (support.active) {
                auto seat = resolvePartGripHandWorld(support, weaponNode);
                seat.translate = resolvePartGripWorld(support, weaponNode);
                setPose(s, SupportSeat, seat);
            }
            const auto* nodes = f4vr::getPlayerNodes();
            if (nodes && nodes->primaryWeaponScopeCamera) setPose(s, Scope, nodes->primaryWeaponScopeCamera->world);
        }
        if (support.active && (!_gunstockWasSupporting || s.grip != _gunstockLastGrip)) {
            ++_gunstockMark;
            s.mark = _gunstockMark;
            _gunstockGrabSample = s;
        }
        s.mark = _gunstockMark;
        _gunstockWasSupporting = support.active;
        _gunstockLastGrip = s.grip;
        if (_gunstockRecorder) (void)_gunstockRecorder->submit(s);
    }

    void TwoHandedGrip::publishGunstockRenderDebug(RE::NiNode* weaponNode)
    {
        using namespace gunstock_debug;
        auto& s = _gunstockSample;
        if (!g_rockConfig.rockDebugDrawGunstock || s.frame != runtime_state::currentFrame().frameIndex || s.phase != 1) return;
        s.phase = 2; // AfterWorldFinal; paired with phase 1 by frame and ownership.
        if (weaponNode) {
            setPose(s, WeaponRendered, weaponNode->world);
            if (s.has(PrimarySeat) && _firing.hasPrimaryHandWeaponLocal) {
                auto seat = transform_math::composeTransforms(weaponNode->world, _firing.primaryHandWeaponLocal);
                seat.translate = transform_math::localPointToWorld(weaponNode->world, _firing.primaryGripLocal);
                setPose(s, PrimarySeat, seat);
            }
            const auto& support = partGrip(isSupportHandLeft());
            if (support.active) {
                auto seat = resolvePartGripHandWorld(support, weaponNode);
                seat.translate = resolvePartGripWorld(support, weaponNode);
                setPose(s, SupportSeat, seat);
            }
        }
        RE::NiTransform hand{};
        if (frik_visual_authority::tryGetPresentedHandWorldTransform(true, hand)) setPose(s, LeftRendered, hand);
        if (frik_visual_authority::tryGetPresentedHandWorldTransform(false, hand)) setPose(s, RightRendered, hand);
        const auto* nodes = f4vr::getPlayerNodes();
        if (nodes && nodes->primaryWeaponScopeCamera) setPose(s, ScopeRendered, nodes->primaryWeaponScopeCamera->world);
        if (_gunstockRecorder) (void)_gunstockRecorder->submit(s);
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
        const bool isLeft,
        AuthoredSupportGripDebugSnapshot& outSnapshot) const
    {
        outSnapshot = authoredSupportHand(isLeft).authoredDebugSnapshot;
        return outSnapshot.valid;
    }

    bool TwoHandedGrip::getFiringGripReattachZoneDebugSnapshot(
        FiringGripReattachZoneDebugSnapshot& outSnapshot) const
    {
        outSnapshot = _firing.reattachDebugSnapshot;
        return outSnapshot.valid;
    }
}
