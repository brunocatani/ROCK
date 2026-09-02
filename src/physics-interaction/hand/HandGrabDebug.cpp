#include "physics-interaction/hand/Hand.h"
#include "physics-interaction/hand/HandGrabInternal.h"

#include "physics-interaction/body/BodyBoneColliderSet.h"
#include "physics-interaction/native/HavokOffsets.h"
#include "physics-interaction/native/HeldScenePresentation.h"

#include "physics-interaction/native/BodyCollisionControl.h"
#include "physics-interaction/grab/GrabCore.h"
#include "physics-interaction/collision/CollisionSuppressionRegistry.h"
#include "physics-interaction/debug/DebugMath.h"
#include "physics-interaction/grenade/LooseGrenadeRuntime.h"
#include "physics-interaction/grab/GrabAuthorityProxy.h"
#include "physics-interaction/grab/GrabConstraint.h"
#include "physics-interaction/grab/GrabConstraintMath.h"
#include "physics-interaction/grab/GrabCore.h"
#include "physics-interaction/grab/SavedGrabOffsetStore.h"
#include "physics-interaction/grab/GrabFinger.h"
#include "physics-interaction/grab/GrabMassPolicy.h"
#include "physics-interaction/grab/GrabMotionController.h"
#include "physics-interaction/grab/GrabPinchPocket.h"
#include "physics-interaction/grab/GrabThreePhase.h"
#include "physics-interaction/grab/GrabHeldObject.h"
#include "physics-interaction/grab/MeshGrab.h"
#include "physics-interaction/object/MechanicalConnectedBodySet.h"
#include "physics-interaction/object/CarInteractionPolicy.h"
#include "physics-interaction/object/ObjectPhysicsBodySet.h"
#include "physics-interaction/weapon/AuthoredWeaponGripLibrary.h"
#include "physics-interaction/weapon/LooseWeaponGripZone.h"
#include "physics-interaction/weapon/WeaponTypePolicy.h"
#include "physics-interaction/object/SkinnedBodyResolver.h"
#include "physics-interaction/performance/PerformanceProfiler.h"
#include "physics-interaction/visual/FrikVisualAuthorityBridge.h"
#include "physics-interaction/hand/HandFrame.h"
#include "physics-interaction/hand/HandVisual.h"
#include "physics-interaction/core/PhysicsHooks.h"
#include "physics-interaction/core/RockRuntimeState.h"
#include "physics-interaction/PhysicsBodyFrame.h"
#include "physics-interaction/native/PhysicsShapeCast.h"
#include "physics-interaction/native/PhysicsRecursiveWrappers.h"
#include "physics-interaction/native/PhysicsUtils.h"
#include "physics-interaction/native/PhysicsScale.h"
#include "physics-interaction/native/HavokMaterialRegistry.h"
#include "physics-interaction/native/HavokRefCount.h"
#include "RE/Havok/hkVector4.h"
#include "RE/Havok/hknpMotion.h"
#include "RE/Bethesda/PlayerCharacter.h"
#include "RE/Bethesda/TESBoundObjects.h"
#include "RE/NetImmerse/NiUpdateData.h"
#include "RockConfig.h"
#include "RockUtils.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/VectorMath.h"
#include "rock_support/Fo4VrRuntime.h"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <format>
#include <algorithm>
#include <array>
#include <atomic>
#include <initializer_list>
#include <limits>
#include <string>
#include <string_view>
#include <xmmintrin.h>

namespace rock
{
    using namespace hand_grab_internal;

    bool Hand::getGrabPivotDebugSnapshot(RE::hknpWorld* world, GrabPivotDebugSnapshot& out) const
    {
        out = {};

        if (!world || !isHolding() || _savedObjectState.bodyId.value == INVALID_BODY_ID) {
            return false;
        }

        RE::NiTransform objectBodyWorld{};
        if (!tryGetGrabDriveObjectWorldTransform(world, _savedObjectState.bodyId, objectBodyWorld)) {
            return false;
        }

        if (!_activeConstraint.isValid() || !_activeConstraint.constraintData) {
            return false;
        }

        RE::NiTransform anchorBodyWorld{};
        if (!_grabAuthorityProxy.isValid() || _grabAuthorityProxy.getBodyId().value == INVALID_BODY_ID ||
            !tryGetGrabAuthorityBodyWorldTransform(world, _grabAuthorityProxy.getBodyId(), anchorBodyWorld)) {
            return false;
        }

        auto* constraintData = static_cast<const char*>(_activeConstraint.constraintData);
        auto* pivotALocal = reinterpret_cast<const float*>(constraintData + GRAB_TRANSFORM_A_POS);
        auto* pivotBLocal = reinterpret_cast<const float*>(constraintData + GRAB_TRANSFORM_B_POS);
        const RE::NiPoint3 pivotALocalGame{ pivotALocal[0] * havokToGameScale(), pivotALocal[1] * havokToGameScale(), pivotALocal[2] * havokToGameScale() };
        const RE::NiPoint3 pivotBLocalGame{ pivotBLocal[0] * havokToGameScale(), pivotBLocal[1] * havokToGameScale(), pivotBLocal[2] * havokToGameScale() };
        out.handPivotWorld = transform_math::localPointToWorld(anchorBodyWorld, pivotALocalGame);
        out.objectPivotWorld = transform_math::localPointToWorld(objectBodyWorld, pivotBLocalGame);
        out.handBodyWorld = anchorBodyWorld.translate;
        out.objectBodyWorld = objectBodyWorld.translate;

        const RE::NiPoint3 error = out.handPivotWorld - out.objectPivotWorld;
        out.pivotErrorGameUnits = std::sqrt(error.x * error.x + error.y * error.y + error.z * error.z);
        return true;
    }

    bool Hand::tryGetHeldDesiredBodyWorld(RE::hknpWorld* world, RE::NiTransform& outDesiredBodyWorld) const
    {
        outDesiredBodyWorld = {};
        if (!world || !isHolding() || _savedObjectState.bodyId.value == INVALID_BODY_ID || !_grabFrame.authority.hasFrozenPivotB) {
            return false;
        }

        RE::NiTransform proxyWorld{};
        const char* proxySource = "none";
        bool proxyFrameOk = resolveGrabAuthorityProxyFrame(
            world,
            proxyWorld,
            proxySource,
            GrabAuthorityProxyFramePolicy::LivePalmOnly);
        if (!proxyFrameOk && _grabAuthorityProxy.isValid() && _grabAuthorityProxy.getBodyId().value != INVALID_BODY_ID) {
            proxyFrameOk = tryGetGrabAuthorityBodyWorldTransform(world, _grabAuthorityProxy.getBodyId(), proxyWorld);
        }
        if (!proxyFrameOk) {
            return false;
        }

        outDesiredBodyWorld =
            grab_frame_math::objectFromGeneratedProxyLocalSpace(proxyWorld, _grabFrame.proxyAuthorityBodyHandSpace);
        return grab_authority_frame_math::isFiniteTransform(outDesiredBodyWorld);
    }
}
