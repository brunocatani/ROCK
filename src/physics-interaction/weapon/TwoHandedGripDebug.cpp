#include "physics-interaction/weapon/TwoHandedGrip.h"
#include "physics-interaction/weapon/WeaponAimBasis.h"
#include "physics-interaction/weapon/TwoHandedGripInternal.h"

#include "api/ROCKProviderApiInternal.h"
#include "physics-interaction/native/SceneRelation.h"
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
#include <cstdio>
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

    void TwoHandedGrip::beginGunstockDebug(std::uint32_t form)
    {
        using namespace gunstock_debug;
        _gunstockSample = {};
        if (!g_rockConfig.rockDebugDrawGunstock && !g_rockConfig.rockDebugDrawControllerAim) {
            _gunstockWasSupporting = false;
            _gunstockGrabSample = {};
            return;
        }
        auto& s = _gunstockSample;
        s.frame = runtime_state::currentFrame().frameIndex;
        s.form = form;
        for (bool left : {true, false}) {
            RE::NiTransform pose{};
            if (frik_visual_authority::tryGetTrackedHandTransform(handFromBool(left), frik_visual_authority::TrackedHandKind::Wand, pose))
                setPose(s, left ? LeftController : RightController, pose);
        }
        RE::NiTransform physical{};
        if (frik_hand_world_authority::tryGetRawHandWorld(true, physical)) setPose(s, LeftPhysical, physical);
    }

    void TwoHandedGrip::finishGunstockDebug(RE::NiNode* weaponNode)
    {
        using namespace gunstock_debug;
        if (!g_rockConfig.rockDebugDrawGunstock && !g_rockConfig.rockDebugDrawControllerAim) return;
        auto& s = _gunstockSample;
        s.phase = 1; // End of grip update, before final collision/FRIK presentation.
        s.firingLeft = isFiringHandLeft();
        const auto& support = partGrip(isSupportHandLeft());
        s.supportActive = support.active;
        s.gunstockAim = support.active && support.gunstockAimSelected && _hasSolvedWeaponTransform;
        s.grip = support.active ? support.gripSequence : 0;
        if (weaponNode) {
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
        }
        if (support.active && (!_gunstockWasSupporting || s.grip != _gunstockLastGrip)) {
            ++_gunstockMark;
            s.mark = _gunstockMark;
            _gunstockGrabSample = s;
        }
        s.mark = _gunstockMark;
        _gunstockWasSupporting = support.active;
        _gunstockLastGrip = s.grip;
        s.aimYawDegrees=_handlingSettings.leftFiringAimYawDegrees;
        s.aimPitchDegrees=_handlingSettings.leftFiringAimPitchDegrees;
        for(bool left:{true,false}){
            const auto controller=left?LeftController:RightController;
            if(!s.has(controller))continue;
            auto basis=weapon_aim_basis::weaponInController();
            if(left)basis=left_firing_position_only_math::mirrorRightWeaponInWandOrientation(basis);
            setPose(s,left?LeftAimReference:RightAimReference,transform_math::composeTransforms(s.poses[controller],basis));
            if(left)basis=transform_math::composeTransforms(makeLeftFiringWandAimTrim(_handlingSettings),basis);
            setPose(s,left?LeftAimConfigured:RightAimConfigured,transform_math::composeTransforms(s.poses[controller],basis));
            auto carrier=s.poses[controller];
            if(left&&usesLeftFiringCarry()&&hasLeftFiringDampedFollowFrame(weaponNode,_session.weaponGenerationKey,_session.equippedWeaponOwnershipKey)&&s.has(LeftPhysical))
                carrier=left_firing_position_only_math::resolveDampedAimCarrierWorld(carrier,_firing.leftDampedFollowFrame.handInWandOrientation,s.poses[LeftPhysical]);
            setPose(s,left?LeftAimDamped:RightAimDamped,transform_math::composeTransforms(carrier,basis));
        }
    }

    void TwoHandedGrip::publishGunstockRenderDebug(RE::NiNode* weaponNode)
    {
        using namespace gunstock_debug;
        auto& s = _gunstockSample;
        if ((!g_rockConfig.rockDebugDrawGunstock && !g_rockConfig.rockDebugDrawControllerAim) || s.frame != runtime_state::currentFrame().frameIndex || s.phase != 1) return;
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
