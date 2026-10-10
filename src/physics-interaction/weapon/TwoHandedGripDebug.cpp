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
#include <cstdio>
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
        if(g_rockConfig.rockCaptureHoldingGrip&&!_holdingCaptureLatched)_holdingCapturePending=true;
        if(!g_rockConfig.rockCaptureHoldingGrip&&_holdingCaptureLatched)_holdingCapturePending=false;
        _holdingCaptureLatched=g_rockConfig.rockCaptureHoldingGrip;
        if(_gunstockRecorder&&_gunstockRecorder->holdingFailed()){ROCK_LOG_ERROR(Weapon,"Holding capture could not be written; existing gunstock recording remains available");_gunstockRecorder->clearHoldingFailure();_holdingCapturePending=false;}
        if(_holdingCapturePending&&_gunstockRecorder&&_gunstockRecorder->failed()){_holdingCapturePending=false;ROCK_LOG_ERROR(Weapon,"Holding capture writer failed; check the diagnostic output directory");}
        if (!g_rockConfig.rockDebugDrawGunstock&&!_holdingCapturePending) {
            _gunstockWasSupporting = false;
            _gunstockGrabSample = {};
            return;
        }
        auto& s = _gunstockSample;
        s.holding.requested=_holdingCapturePending;s.holding.captureOnly=!g_rockConfig.rockDebugDrawGunstock;
        if(_holdingCapturePending&&_firing.hasRightNaturalBoneInWand&&_firing.hasLeftNaturalBoneInWand&&hasRightFiringHandCanonicalFrame(weaponNode,generation,ownership)){
            s.holding.rightBoneInWand=_firing.rightNaturalBoneInWand;s.holding.leftBoneInWand=_firing.leftNaturalBoneInWand;
            s.holding.rightBoneInWand.translate={};s.holding.rightBoneInWand.scale=1;s.holding.leftBoneInWand.translate={};s.holding.leftBoneInWand.scale=1;
            s.holding.nativeRight=_holdingHasNative?_holdingNativeRight:_firing.rightCanonicalHandWeaponLocal;s.holding.nativeFingers=_holdingSourceRightFingers;s.holding.nativeFingersValid=_holdingSourceRightValid;s.holding.powerArmor=_holdingSourcePowerArmor;if(const auto* handed=f4vr::getIniSetting("bLeftHandedMode:VR"))s.holding.gameLeftHanded=handed->GetBinary();s.holding.hasSupport=_holdingHasSourceSupport;s.holding.nativeSupport=_holdingSourceSupport;s.holding.supportFingers=_holdingSourceSupportFingers;
            RE::NiTransform identity;identity.MakeIdentity();const auto pr=computeGrabLegacyPalmPivotAWorldFromHandBasis(identity,false),pl=computeGrabLegacyPalmPivotAWorldFromHandBasis(identity,true);
            s.holding.palmRight={pr.x,pr.y,pr.z};s.holding.palmLeft={pl.x,pl.y,pl.z};
            const auto lo=g_rockConfig.rockLeftFiringGripOffsetGameUnits,so=g_rockConfig.rockRightSupportGripOffsetGameUnits;s.holding.leftOffset={lo.x,lo.y,lo.z};s.holding.rightSupportOffset={so.x,so.y,so.z};
            auto* item=RE::TESForm::GetFormByID(form);const auto* file=item?item->GetFile(0):nullptr;
            if(file){std::snprintf(s.holding.weapon,sizeof(s.holding.weapon),"%.*s:%06x",static_cast<int>(file->GetFilename().size()),file->GetFilename().data(),item->GetLocalFormID());s.holding.ready=true;}
            auto* mount=weaponNode?f4vr::findNode(weaponNode,"P-Grip"):nullptr;auto* child=mount?f4vr::getFirstChild(mount):nullptr;if(child&&child->name.c_str())std::snprintf(s.holding.gripNode,sizeof(s.holding.gripNode),"%s",child->name.c_str());
        }
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
        if (!g_rockConfig.rockDebugDrawGunstock&&!_holdingCapturePending) return;
        auto& s = _gunstockSample;
        s.state = static_cast<std::uint32_t>(_session.state);
        s.phase = 1; // End of grip update, before final collision/FRIK presentation.
        s.firingLeft = isFiringHandLeft();
        const auto& support = partGrip(isSupportHandLeft());
        s.supportActive = support.active;
        s.gunstockAim = support.active && support.gunstockAimSelected && _hasSolvedWeaponTransform;
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
        if (_gunstockRecorder&&!s.holding.captureOnly) (void)_gunstockRecorder->submit(s);
    }

    void TwoHandedGrip::publishGunstockRenderDebug(RE::NiNode* weaponNode)
    {
        using namespace gunstock_debug;
        auto& s = _gunstockSample;
        if ((!g_rockConfig.rockDebugDrawGunstock&&!_holdingCapturePending) || s.frame != runtime_state::currentFrame().frameIndex || s.phase != 1) return;
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
        if(s.holding.requested&&s.holding.ready&&s.has(WeaponRendered)){
            // Fingers can exist only in the flattened skinning tree, without
            // scene NiNodes. Read the final rendered array, including its wrist,
            // so each parent-local joint uses the same presentation authority.
            DirectSkeletonBoneReader reader;
            DirectSkeletonBoneSnapshot bones;
            const bool captured = reader.capture(
                skeleton_bone_debug_math::DebugSkeletonBoneMode::HandsAndForearmsOnly,
                skeleton_bone_debug_math::DebugSkeletonBoneSource::GameRootFlattenedBoneTree,
                SkeletonBoneCaptureSpace::Rendered, bones);
            const auto findBone = [&](std::string_view name) -> const DirectSkeletonBoneEntry* {
                if (!captured) return nullptr;
                for (const auto& bone : bones.bones) {
                    if (bone.name == name) return &bone;
                }
                return nullptr;
            };
            for (int h = 0; h < 2; ++h) {
                const bool left = h == 0;
                const auto* wrist = findBone(left ? "LArm_Hand" : "RArm_Hand");
                bool valid = wrist != nullptr;
                if (wrist) setPose(s, left ? LeftRendered : RightRendered, wrist->world);
                for (int finger = 0; finger < 5 && valid; ++finger) {
                    auto parent = wrist->world;
                    for (int joint = 0; joint < 3; ++joint) {
                        const auto* bone = findBone(root_flattened_finger_skeleton_runtime::fingerBoneName(left, finger, joint));
                        if (!bone) { valid = false; break; }
                        s.holding.renderedFingers[h][finger * 3 + joint] =
                            transform_math::composeTransforms(transform_math::invertTransform(parent), bone->world);
                        parent = bone->world;
                    }
                }
                s.holding.fingerValid[h] = valid;
                if (!valid) ROCK_LOG_WARN(Weapon, "Holding capture has no complete rendered {} finger chain", left ? "left" : "right");
            }
            if(_gunstockRecorder&&_gunstockRecorder->submit(s)){_holdingCapturePending=false;ROCK_LOG_INFO(Weapon,"Holding grip capture queued weapon={} frame={}",s.holding.weapon,s.frame);}
        }else if(_gunstockRecorder&&!s.holding.captureOnly)(void)_gunstockRecorder->submit(s);

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
