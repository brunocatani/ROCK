#include "WeaponPrimaryGripAuthority.h"

#include "PalmTransform.h"
#include "PhysicsLog.h"
#include "RockUtils.h"
#include "TransformMath.h"
#include "WeaponPrimaryGripAuthorityPolicy.h"
#include "WeaponPrimaryGripMath.h"
#include "WeaponPrimaryGripRuntime.h"
#include "WeaponVisualAuthorityMath.h"
#include "api/FRIKApi.h"
#include "f4vr/F4VRUtils.h"

namespace frik::rock
{
    namespace
    {
        const char* primaryGripSourceName(weapon_primary_grip_math::PrimaryGripSource source)
        {
            switch (source) {
            case weapon_primary_grip_math::PrimaryGripSource::Mesh:
                return "mesh";
            case weapon_primary_grip_math::PrimaryGripSource::IniOverride:
                return "ini-override";
            case weapon_primary_grip_math::PrimaryGripSource::FallbackCurrent:
            default:
                return "fallback-current";
            }
        }

        RE::NiPoint3 weaponLocalToWorld(const RE::NiPoint3& localPos, const RE::NiNode* weaponNode)
        {
            if (!weaponNode) {
                return {};
            }
            return transform_math::localPointToWorld(weaponNode->world, localPos);
        }

        RE::NiPoint3 worldToWeaponLocal(const RE::NiPoint3& worldPos, const RE::NiNode* weaponNode)
        {
            if (!weaponNode) {
                return {};
            }
            return transform_math::worldPointToLocal(weaponNode->world, worldPos);
        }
    }

    void WeaponPrimaryGripAuthority::update(RE::NiNode* weaponNode, bool supportGripActive, const WeaponReloadRuntimeState& reloadState, bool authorityEnabled)
    {
        (void)reloadState;
        _hasAppliedWeaponTransform = false;

        if (!authorityEnabled) {
            if (_authorityPublished) {
                ROCK_LOG_DEBUG(Weapon, "PrimaryGripAuthority: disabled by config; returning one-handed weapon authority to FRIK");
            }
            reset();
            return;
        }

        auto* api = frik::api::FRIKApi::inst;
        if (!api || !api->isSkeletonReady() || !weaponNode) {
            reset();
            return;
        }

        const bool weaponDrawn = api->isWeaponDrawn();
        const bool meleeWeaponDrawn = api->isMeleeWeaponDrawn();
        if (weaponNode != _activeWeaponNode) {
            ++_generation;
            _activeWeaponNode = weaponNode;
            _authorityPublished = false;
        }

        const bool primaryHandIsLeft = f4vr::isLeftHandedMode();
        const RE::NiTransform primaryHandTransform = api->getHandWorldTransform(handFromBool(primaryHandIsLeft));
        const RE::NiPoint3 primaryPalmPos = computeGrabPivotAPositionFromHandBasis(primaryHandTransform, primaryHandIsLeft);
        RE::NiTransform fallbackPrimaryGripFrame = transform_math::makeIdentityTransform<RE::NiTransform>();
        fallbackPrimaryGripFrame.translate = worldToWeaponLocal(primaryPalmPos, weaponNode);

        const auto runtimeSelection = weapon_primary_grip_runtime::selectPrimaryGripFrameForWeapon(weaponNode, fallbackPrimaryGripFrame, false);
        const auto& selection = runtimeSelection.selection;
        const bool meshSelected = selection.source == weapon_primary_grip_math::PrimaryGripSource::Mesh;
        if (!weapon_primary_grip_authority_policy::shouldPublishPrimaryGripAuthority(meshSelected, supportGripActive, weaponDrawn, meleeWeaponDrawn, authorityEnabled)) {
            if (_authorityPublished) {
                ROCK_LOG_DEBUG(Weapon,
                    "PrimaryGripAuthority: clearing source={} supportActive={} weaponDrawn={} melee={}",
                    primaryGripSourceName(selection.source),
                    supportGripActive ? "yes" : "no",
                    weaponDrawn ? "yes" : "no",
                    meleeWeaponDrawn ? "yes" : "no");
            }
            _authorityPublished = false;
            return;
        }

        const RE::NiPoint3 gripWorldPoint = weaponLocalToWorld(selection.gripWeaponLocal.translate, weaponNode);
        const RE::NiTransform adjustedHandTransform = weapon_primary_grip_runtime::alignHandFrameToGripFrame(
            primaryHandTransform,
            primaryHandIsLeft,
            weaponNode->world,
            selection.gripWeaponLocal,
            primaryPalmPos,
            gripWorldPoint,
            true);
        const RE::NiTransform handWeaponLocal = transform_math::composeTransforms(transform_math::invertTransform(weaponNode->world), adjustedHandTransform);
        const RE::NiTransform solvedWeaponWorld =
            weapon_primary_grip_math::solveWeaponWorldFromPrimaryGrip(primaryHandTransform, handWeaponLocal);

        if (!applyWeaponVisualAuthority(weaponNode, solvedWeaponWorld)) {
            _authorityPublished = false;
            return;
        }

        _lastAppliedWeaponTransform = weaponNode->world;
        _hasAppliedWeaponTransform = true;

        if (!_authorityPublished || ++_logCounter >= 180) {
            _logCounter = 0;
            ROCK_LOG_DEBUG(Weapon,
                "PrimaryGripAuthority: active weapon='{}' source={} confidence={:.2f} candidates={} hand={} generation={}",
                weaponNode->name.c_str(),
                primaryGripSourceName(selection.source),
                selection.confidence,
                runtimeSelection.candidateCount,
                primaryHandIsLeft ? "left" : "right",
                _generation);
        }
        _authorityPublished = true;
    }

    void WeaponPrimaryGripAuthority::reset()
    {
        _activeWeaponNode = nullptr;
        _authorityPublished = false;
        _hasAppliedWeaponTransform = false;
        _lastAppliedWeaponTransform = {};
        _logCounter = 0;
    }

    bool WeaponPrimaryGripAuthority::getAppliedWeaponTransform(RE::NiTransform& outTransform) const
    {
        if (!_hasAppliedWeaponTransform) {
            return false;
        }

        outTransform = _lastAppliedWeaponTransform;
        return true;
    }

    bool WeaponPrimaryGripAuthority::applyWeaponVisualAuthority(RE::NiNode* weaponNode, const RE::NiTransform& weaponWorldTarget)
    {
        if (!weaponNode) {
            return false;
        }

        if (weaponNode->parent) {
            weaponNode->local = weapon_visual_authority_math::worldTargetToParentLocal(weaponNode->parent->world, weaponWorldTarget);
            f4vr::updateTransformsDown(weaponNode, true);
        } else {
            weaponNode->local = weaponWorldTarget;
            weaponNode->world = weaponWorldTarget;
            f4vr::updateTransformsDown(weaponNode, false);
        }
        return true;
    }

}
