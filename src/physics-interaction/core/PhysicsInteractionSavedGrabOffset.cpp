#include "physics-interaction/core/PhysicsInteraction.h"
#include "physics-interaction/core/PhysicsInteractionInternal.h"

#include "RockConfig.h"
#include "physics-interaction/PhysicsBodyFrame.h"
#include "physics-interaction/grab/GrabNodeInfoMath.h"
#include "physics-interaction/input/InputRemapRuntime.h"
#include "physics-interaction/native/query/PhysicsUtils.h"
#include "physics-interaction/visual/FrikVisualAuthorityBridge.h"
#include "rock_support/Fo4VrRuntime.h"
#include "rock_support/VRControllers.h"

namespace rock
{
    using namespace physics_interaction_detail;

    void PhysicsInteraction::saveGrabOffsetForHand(Hand& hand, bool isLeft, RE::hknpWorld* hknpWorld)
    {
        if (!hknpWorld) {
            ROCK_LOG_WARN(Hand, "Saved grab offset: aborted, no hknpWorld this frame ({} hand)", isLeft ? "left" : "right");
            return;
        }
        if (!hand.isHolding()) {
            ROCK_LOG_WARN(Hand, "Saved grab offset: aborted, {} hand is not holding anything", isLeft ? "left" : "right");
            return;
        }

        auto* heldRef = hand.getHeldRef();
        auto* rootNode = heldRef ? heldRef->Get3D() : nullptr;
        auto* baseForm = heldRef ? heldRef->GetObjectReference() : nullptr;
        if (!rootNode || !baseForm) {
            ROCK_LOG_WARN(Hand,
                "Saved grab offset: aborted, held ref missing 3D root or base form ({} hand, refr={:08X})",
                isLeft ? "left" : "right",
                heldRef ? heldRef->GetFormID() : 0);
            return;
        }

        const auto* weaponForm = baseForm->As<RE::TESObjectWEAP>();
        const bool throwableWeapon = weaponForm &&
            (weaponForm->weaponData.type == RE::WEAPON_TYPE::kGrenade ||
                weaponForm->weaponData.type == RE::WEAPON_TYPE::kMine);
        if (!saved_grab_offset::participatesInSavedGrabOffsets(weaponForm != nullptr, throwableWeapon)) {
            ROCK_LOG_INFO(Hand,
                "Saved grab offset: skipped for {} hand, '{}' ({:08X}) is a weapon and weapons seat through FRIK weapon offsets only",
                isLeft ? "left" : "right",
                heldRef->GetDisplayFullName() ? heldRef->GetDisplayFullName() : "",
                baseForm->GetFormID());
            return;
        }

        const auto formRef = saved_grab_offset::formRefFromRuntimeId(baseForm->GetFormID());
        if (formRef.empty()) {
            ROCK_LOG_WARN(Hand, "Saved grab offset: aborted, could not resolve load-order-independent identity for base form {:08X}",
                baseForm->GetFormID());
            return;
        }

        RE::NiTransform proxyWorld{};
        if (!hand.tryComputeGrabProxyLocalPalmPocketFrameWorld(hknpWorld, proxyWorld)) {
            ROCK_LOG_WARN(Hand, "Saved grab offset: could not resolve live proxy frame for {} hand", isLeft ? "left" : "right");
            return;
        }

        const RE::NiTransform objectProxyLocal = grab_frame_math::objectInGeneratedProxyLocalSpace(proxyWorld, rootNode->world);

        saved_grab_offset::SavedGrabOffsetFile file{};
        std::string loadError;
        if (!saved_grab_offset::load(formRef, file, &loadError) && !loadError.empty()) {
            ROCK_LOG_WARN(Hand, "Saved grab offset: existing file for {:08X} unreadable ({}), overwriting", baseForm->GetFormID(), loadError);
        }
        file.object = formRef;
        file.objectName = heldRef->GetDisplayFullName() ? heldRef->GetDisplayFullName() : std::string{};
        file.formatVersion = saved_grab_offset::kFormatVersion;

        auto& handOffset = isLeft ? file.left : file.right;
        handOffset.present = true;
        handOffset.translateGame[0] = objectProxyLocal.translate.x;
        handOffset.translateGame[1] = objectProxyLocal.translate.y;
        handOffset.translateGame[2] = objectProxyLocal.translate.z;
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                handOffset.rotate[row * 3 + column] = objectProxyLocal.rotate.entry[row][column];
            }
        }

        /*
         * Finger pose is only captured from a live organic mesh-curl grab
         * (see Hand::tryGetLiveGrabFingerPoseSnapshot). If this hold never
         * ran the mesh solve - e.g. re-saving position only while the object
         * is already attached via a previously-saved offset (pull-catch/
         * force-grab) - there is no fresh finger data this frame, so any
         * finger pose already on disk for this hand is left untouched
         * instead of being cleared.
         */
        Hand::GrabFingerPoseSnapshot fingerSnapshot{};
        if (hand.tryGetLiveGrabFingerPoseSnapshot(fingerSnapshot)) {
            handOffset.hasFingerPose = true;
            handOffset.fingerValues[0] = fingerSnapshot.values[0];
            handOffset.fingerValues[1] = fingerSnapshot.values[1];
            handOffset.fingerValues[2] = fingerSnapshot.values[2];
            handOffset.fingerValues[3] = fingerSnapshot.values[3];
            handOffset.fingerValues[4] = fingerSnapshot.values[4];
            handOffset.hasFingerJointValues = fingerSnapshot.hasJointValues;
            if (fingerSnapshot.hasJointValues) {
                for (std::size_t i = 0; i < fingerSnapshot.jointValues.size(); ++i) {
                    handOffset.fingerJointValues[i] = fingerSnapshot.jointValues[i];
                }
            }
        }

        saved_grab_offset::save(file);

        /*
         * Ground-truth capture companion. The offset records WHERE the object
         * ends up; this records what it was posed against - the mesh ROCK
         * scored, the hand colliders it was posed relative to, the physics,
         * the contacts the pose actually makes, and the seat ROCK itself
         * committed before the user corrected it. That is what makes a saved
         * pose replayable and scorable offline instead of only reproducible
         * in-game. Physics and identity are filled here because this scope
         * owns the body-mass reader and the form identity.
         */
        saved_grab_capture::SavedGrabCaptureFile capture{};
        if (hand.tryBuildSavedGrabCapture(hknpWorld, proxyWorld, capture.capture)) {
            capture.object = formRef;
            capture.objectName = file.objectName;
            capture.hand = isLeft ? "left" : "right";
            capture.rockVersion = std::string(Version::NAME);

            const std::time_t capturedAt = std::time(nullptr);
            std::tm capturedUtc{};
            if (gmtime_s(&capturedUtc, &capturedAt) == 0) {
                char timeText[32]{};
                if (std::strftime(timeText, sizeof(timeText), "%Y-%m-%dT%H:%M:%SZ", &capturedUtc) > 0) {
                    capture.capturedUtc = timeText;
                }
            }

            const std::uint32_t heldBodyId = hand.getSavedObjectState().bodyId.value;
            capture.capture.physics.bodyId = heldBodyId;
            const float heldMass = readGrabEventBodyMass(hknpWorld, heldBodyId);
            if (std::isfinite(heldMass) && heldMass > 0.0f) {
                capture.capture.physics.mass = heldMass;
                capture.capture.physics.valid = true;
            }

            saved_grab_offset::saveCapture(capture);
            ROCK_LOG_INFO(Hand,
                "Saved grab capture for {:08X} ({} hand): triangles={} colliders={} mass={:.2f} com={} shape={} seatReasons=[align={} roll={} depth={} backstop={}]",
                baseForm->GetFormID(),
                isLeft ? "left" : "right",
                capture.capture.mesh.triangleCount,
                capture.capture.fingerSegments.size() + (capture.capture.palm.valid ? 1u : 0u),
                capture.capture.physics.mass,
                capture.capture.physics.hasCenterOfMass ? (capture.capture.physics.comTrusted ? "trusted" : "UNTRUSTED") : "none",
                capture.capture.seat.shapeClass,
                capture.capture.seat.alignmentReason,
                capture.capture.seat.rollReason,
                capture.capture.seat.depthReason,
                capture.capture.seat.penetrationBackstopReason);
        } else {
            ROCK_LOG_WARN(Hand,
                "Saved grab offset: ground-truth capture unavailable for {} hand ({:08X}); the offset itself was still saved",
                isLeft ? "left" : "right",
                baseForm->GetFormID());
        }

        ROCK_LOG_INFO(Hand,
            "Saved grab offset for {:08X} ({} hand, finger pose {})",
            baseForm->GetFormID(),
            isLeft ? "left" : "right",
            handOffset.hasFingerPose ? "captured" : "unchanged");

        const char* itemName = heldRef->GetDisplayFullName();
        f4vr::showNotification(std::string("Saved grab offset: ") + (itemName && *itemName ? itemName : "item"));
    }

    void PhysicsInteraction::updateSavedGrabOffsetGesture(const PhysicsFrameContext& frame)
    {
        /*
         * The actual press detection (developer mode, Activate/WandAccept
         * edge, which hand is engaged) lives in InputRemapRuntime, which
         * already tracks per-hand held-object state and native-event
         * dispatch; this just consumes the resulting per-hand request.
         */
        if (input_remap_runtime::consumePendingSavedGrabOffsetRequest(false)) {
            saveGrabOffsetForHand(_rightHand, false, frame.hknpWorld);
        }
        if (input_remap_runtime::consumePendingSavedGrabOffsetRequest(true)) {
            saveGrabOffsetForHand(_leftHand, true, frame.hknpWorld);
        }
    }

}
