/*
 * The EQUIPPED WEAPON half of the frame orchestrator. Its three phases run back
 * to back, in this order, from update() in PhysicsInteractionFrame.cpp:
 *
 *   serviceWeaponContactAcquisition - collect hand evidence against the weapon
 *   serviceEquippedWeaponGripFrame  - solve stash, grip, drop, hand assignment
 *   finishDynamicWeaponFrame        - publish authority once the solve is final
 *
 * The order is load bearing. Evidence must be collected before a grip can change
 * ownership, and authority must be published only after the solve is final. Do
 * not reorder these, and do not call one without the others.
 *
 * They share one EquippedWeaponFrame. Each phase reads what the previous phase
 * decided and writes its own answer back into it, so that struct is the whole
 * contract between them.
 *
 * The anonymous-namespace helpers below serve only these three phases. Keep them
 * here with internal linkage; do not promote them.
 */


#include "physics-interaction/core/PhysicsInteraction.h"
#include "physics-interaction/core/PhysicsInteractionInternal.h"
#include "physics-interaction/core/PhysicsInteractionTransformValidation.h"

#include "api/ProviderColliderVisualizationRuntime.h"
#include "api/ProviderDebugOverlayRuntime.h"
#include "api/ROCKProviderApiInternal.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <numbers>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>

#include "physics-interaction/native/BethesdaPhysicsBody.h"
#include "physics-interaction/native/BodyCollisionControl.h"
#include "physics-interaction/hand/HeldBodyRenderPose.h"
#include "physics-interaction/actor/ActorEquipmentGrab.h"
#include "physics-interaction/animation/AuthoredWeaponGripCapturePolicy.h"
#include "physics-interaction/api/InteractionCommandQueue.h"
#include "physics-interaction/collision/CollisionLayerPolicy.h"
#include "physics-interaction/collision/CollisionSuppressionRegistry.h"
#include "physics-interaction/collision/ContactPipelinePolicy.h"
#include "physics-interaction/collision/ContactSignalSubscriptionPolicy.h"
#include "physics-interaction/consume/MouthConsumeDetector.h"
#include "physics-interaction/consume/MouthConsumePolicy.h"
#include "physics-interaction/consume/MouthConsumeTransfer.h"
#include "physics-interaction/feedback/FeedbackHaptics.h"
#include "physics-interaction/hand/skeleton/HandSkeleton.h"
#include "physics-interaction/native/havok/HavokOffsets.h"
#include "physics-interaction/debug/overlay/DebugBodyOverlay.h"
#include "physics-interaction/debug/overlay/DebugOverlayPolicy.h"
#include "physics-interaction/feedback/HapticPolicy.h"
#include "physics-interaction/grab/GrabCore.h"
#include "physics-interaction/grab/GrabConstraint.h"
#include "physics-interaction/grab/CustomOGA.h"
#include "physics-interaction/grab/GrabEvent.h"
#include "physics-interaction/grab/GrabTelemetry.h"
#include "physics-interaction/grab/GrabHeldObject.h"
#include "physics-interaction/grab/GrabMassPolicy.h"
#include "physics-interaction/grab/GrabNodeInfoMath.h"
#include "physics-interaction/grab/GrabPinchPocket.h"
#include "physics-interaction/grab/GrabThreePhase.h"
#include "physics-interaction/grab/HeldMassMovement.h"
#include "physics-interaction/hand/HandLifecycle.h"
#include "physics-interaction/native/havok/HavokRuntime.h"
#include "physics-interaction/native/CharacterControllerRuntime.h"
#include "physics-interaction/native/HeldWeaponInstantTransition.h"
#include "physics-interaction/input/InputRemapPolicy.h"
#include "physics-interaction/input/InputRemapRuntime.h"
#include "physics-interaction/input/GrabInputIntentPolicy.h"
#include "physics-interaction/object/ObjectDetection.h"
#include "physics-interaction/object/CarInteractionPolicy.h"
#include "physics-interaction/object/ObjectPhysicsBodySet.h"
#include "physics-interaction/stash/ShoulderStashDetector.h"
#include "physics-interaction/stash/ShoulderStashPolicy.h"
#include "physics-interaction/stash/ShoulderStashTransfer.h"
#include "physics-interaction/weapon/LooseWeaponGripZone.h"
#include "physics-interaction/weapon/collision/DynamicWeaponCollisionPolicy.h"
#include "physics-interaction/weapon/equip/EquippedWeaponHandlingRuntime.h"
#include "physics-interaction/weapon/native_anim/NativeScopeSightAnchorPolicy.h"
#include "physics-interaction/weapon/native_anim/NativeIdleGripPreharvest.h"
#include "physics-interaction/weapon/native_anim/NativeEquippedWeaponDraw.h"
#include "physics-interaction/weapon/equip/WeaponTransitionAnimationAcceleration.h"
#include "physics-interaction/weapon/equip/PipboyEquipRuntime.h"
#include "physics-interaction/weapon/equip/HeldWeaponEquipStatePolicy.h"
#include "physics-interaction/weapon/equip/WeaponEquipTransfer.h"
#include "physics-interaction/weapon/WeaponInteraction.h"
#include "physics-interaction/hand/skeleton/HandFrame.h"
#include "physics-interaction/core/PhysicsHooks.h"
#include "physics-interaction/core/RockRuntimeState.h"
#include "physics-interaction/native/query/PhysicsRecursiveWrappers.h"
#include "physics-interaction/native/query/PhysicsRayCast.h"
#include "physics-interaction/native/query/PhysicsScale.h"
#include "physics-interaction/native/query/PhysicsUtils.h"
#include "physics-interaction/performance/PerformanceProfiler.h"
#include "physics-interaction/debug/PhysicsWorldOriginDiagnostics.h"
#include "physics-interaction/collision/PushAssist.h"
#include "physics-interaction/hand/selection/HandSelection.h"
#include "physics-interaction/visual/FrikVisualAuthorityBridge.h"
#include "physics-interaction/weapon/WeaponSupport.h"
#include "physics-interaction/weapon/WeaponAuthority.h"
#include "physics-interaction/PhysicsBodyFrame.h"
#include "physics-interaction/TransformMath.h"

#include "RE/Bethesda/ActorValueInfo.h"
#include "RE/Bethesda/BSHavok.h"
#include "RE/Bethesda/Events.h"
#include "RE/Bethesda/FormComponents.h"
#include "RE/Bethesda/TESBoundObjects.h"
#include "RE/Bethesda/TESForms.h"
#include "RE/Bethesda/TESObjectREFRs.h"
#include "RE/Bethesda/UI.h"
#include "RE/Havok/hknpMotion.h"
#include "RE/Havok/hknpWorld.h"

#include "ROCKMain.h"
#include "RockConfig.h"
#include "RockUtils.h"
#include "rock_support/Fo4VrActorStatePolicy.h"
#include "rock_support/Fo4VrRuntime.h"
#include "rock_support/VRControllers.h"
#include <windows.h>



namespace rock
{
    using namespace physics_interaction_detail;

    namespace
    {
        bool tryReadNativeScopeRequestState(bool& outActive)
        {
            using GetScopeRequestState = bool (*)(const void*);
            static REL::Relocation<GetScopeRequestState> getScopeRequestState{ REL::Offset(offsets::kFunc_NativeScopeRequestStateGet) };
            static REL::Relocation<std::uintptr_t> rendererState{ REL::Offset(offsets::kData_NativeScopeRendererState) };
            if (!getScopeRequestState.address() || !rendererState.address()) {
                return false;
            }
            outActive = getScopeRequestState(reinterpret_cast<const void*>(rendererState.address()));
            return true;
        }

        ::rock::provider::RockProviderWeaponPartTargetQueryV1 makeProviderWeaponPartTargetQuery(
            const WeaponInteractionContact& contact,
            const WeaponCollision& weaponCollision)
        {
            ::rock::provider::RockProviderWeaponPartTargetQueryV1 query{};
            query.weaponGenerationKey = contact.weaponGenerationKey;
            query.bodyId = contact.bodyId;
            query.partKind = static_cast<std::uint32_t>(contact.partKind);
            query.reloadRole = static_cast<std::uint32_t>(contact.reloadRole);
            query.supportRole = static_cast<std::uint32_t>(contact.supportGripRole);
            query.socketRole = static_cast<std::uint32_t>(contact.socketRole);
            query.actionRole = static_cast<std::uint32_t>(contact.actionRole);
            query.sourceRoot = reinterpret_cast<std::uintptr_t>(contact.sourceRoot);
            WeaponCollisionProfileEvidenceDescriptor descriptor{};
            RE::NiAVObject* sourceNode = nullptr;
            if (weaponCollision.tryGetProfileEvidenceDescriptorForBodyId(contact.bodyId, descriptor, sourceNode) &&
                descriptor.weaponGenerationKey == contact.weaponGenerationKey) {
                query.sourceRoot = descriptor.sourceRootAddress;
                copyProviderString(query.sourceName, sizeof(query.sourceName), descriptor.sourceName);
            }
            return query;
        }

        WeaponProviderPartAuthority makeWeaponProviderPartAuthority(
            const ::rock::provider::RockProviderWeaponPartTargetQueryV1& query,
            const ::rock::provider::RockProviderWeaponPartTargetResolutionV1& resolution)
        {
            WeaponProviderPartAuthority authority{};
            authority.active = resolution.matched != 0;
            authority.ownerToken = resolution.ownerToken;
            authority.weaponGenerationKey = query.weaponGenerationKey;
            authority.bodyId = query.bodyId;
            authority.sourceRoot = query.sourceRoot;
            authority.partKind = query.partKind;
            authority.reloadRole = query.reloadRole;
            authority.supportRole = query.supportRole;
            authority.socketRole = query.socketRole;
            authority.actionRole = query.actionRole;
            authority.groupId = resolution.groupId;
            authority.grabMode = static_cast<std::uint32_t>(resolution.grabMode);
            static_assert(WeaponProviderPartAuthority{}.sourceName.size() == ::rock::provider::ROCK_PROVIDER_MAX_EVIDENCE_NAME);
            std::memcpy(authority.sourceName.data(), query.sourceName, authority.sourceName.size());
            authority.sourceName[authority.sourceName.size() - 1] = '\0';
            return authority;
        }


        const char* weaponDiagnosticNodeName(const RE::NiAVObject* node)
        {
            if (!node) {
                return "";
            }

            const char* name = node->name.c_str();
            return name ? name : "";
        }

        WeaponInteractionDebugInfo makeWeaponInteractionDebugInfo(
            const WeaponCollision& weaponCollision,
            RE::NiNode* weaponNode,
            const WeaponInteractionContact& contact)
        {
            WeaponInteractionDebugInfo info{};
            info.weaponNodeName = weaponDiagnosticNodeName(weaponNode);

            auto* equipData = f4vr::getEquippedWeaponItem();
            auto* weaponForm = equipData ? equipData->item.object : nullptr;
            if (weaponForm) {
                info.weaponFormId = weaponForm->formID;
                const auto fullName = RE::TESFullName::GetFullName(*weaponForm);
                if (!fullName.empty()) {
                    info.weaponName = fullName;
                }
            }

            if (contact.valid) {
                WeaponInteractionDebugInfo sourceInfo{};
                if (weaponCollision.tryGetWeaponContactDebugInfo(contact.bodyId, sourceInfo)) {
                    info.sourceName = sourceInfo.sourceName;
                    info.interactionRootName = sourceInfo.interactionRootName;
                    info.sourceRootName = sourceInfo.sourceRootName;
                }
                if (info.interactionRootName.empty()) {
                    info.interactionRootName = weaponDiagnosticNodeName(contact.interactionRoot);
                }
                if (info.sourceRootName.empty()) {
                    info.sourceRootName = weaponDiagnosticNodeName(contact.sourceRoot);
                }
            }

            return info;
        }


        f4vr::MuzzleFlash* getEquippedMuzzleFlashNodes()
        {
            /*
             * ROCK is the final weapon visual owner during mesh/hand authority.
             * Any ROCK weapon write after the normal first-person weapon update
             * must re-own the fire node from the current projectile node so the
             * muzzle origin remains at the barrel tip.
             */
            const auto equipWeaponData =
                getValidatedEquippedWeaponData();
            if (!equipWeaponData) {
                return nullptr;
            }

            const auto muzzle = reinterpret_cast<f4vr::MuzzleFlash*>(equipWeaponData->muzzleFlash);
            if (!muzzle || !muzzle->fireNode || !muzzle->projectileNode) {
                return nullptr;
            }

            return muzzle;
        }

        void applyFinalWeaponMuzzleAuthority()
        {
            auto* muzzle = getEquippedMuzzleFlashNodes();
            if (!muzzle) {
                return;
            }

            muzzle->fireNode->local = weapon_muzzle_authority_math::fireNodeLocalFromProjectileWorld(muzzle->projectileNode->world);
            f4vr::updateTransformsDown(muzzle->fireNode, true);
        }
    }

    void PhysicsInteraction::serviceWeaponContactAcquisition(
        const PhysicsFrameContext& frame,
        EquippedWeaponFrame& weaponFrame)
    {
        auto* weaponNode = weaponFrame.weaponNode;
        auto& leftWeaponContact = weaponFrame.leftContact;
        auto& rightWeaponContact = weaponFrame.rightContact;
        auto& leftWeaponContactSource = weaponFrame.leftContactSource;

        auto consumeWeaponContactForHand = [&](bool isLeft, const HandFrameInput& handInput, bool probeAllowed, WeaponInteractionContact& outContact) {
            auto& bodyIdAtomic = isLeft ? _leftWeaponContactBodyId : _rightWeaponContactBodyId;
            auto& missedFrames = isLeft ? _leftWeaponContactMissedFrames : _rightWeaponContactMissedFrames;
            auto& acquisitionState = _weaponInteractionAcquisitionStates[isLeft ? 0u : 1u];

            // Drain the physics-thread notification, but do not use an
            // arbitrary finger/body callback as palm-touch provenance.
            // Touch is the deterministic overlap below for both physical
            // hands and for either firing/support role.
            (void)bodyIdAtomic.exchange(INVALID_CONTACT_BODY_ID, std::memory_order_acquire);

            const RE::NiPoint3 legacyPalmPivotWorld =
                computeGrabLegacyPalmPivotAWorldFromHandBasis(
                    handInput.rawHandWorld,
                    isLeft);
            const bool touchObserved = weaponNode &&
                _weaponCollision.tryFindInteractionContactNearPoint(
                    weaponNode,
                    legacyPalmPivotWorld,
                    g_rockConfig.rockWeaponInteractionTouchRadius,
                    outContact);
            if (touchObserved) {
                publishWeaponInteractionContact(isLeft, outContact);
            } else if (weaponNode && probeAllowed) {
                if (_weaponCollision.tryFindInteractionContactNearPoint(
                        weaponNode,
                        handInput.grabAnchorWorld,
                        g_rockConfig.rockWeaponInteractionProbeRadius,
                        outContact)) {
                    publishWeaponInteractionContact(isLeft, outContact);
                    if (g_rockConfig.rockDebugVerboseLogging && ++_weaponInteractionProbeLogCounter >= 90) {
                        _weaponInteractionProbeLogCounter = 0;
                        ROCK_LOG_DEBUG(Weapon,
                            "WeaponInteractionProbe: hand={} bodyId={} partKind={} supportRole={} reloadRole={} actionRole={} radius={:.1f}",
                            isLeft ? "left" : "right",
                            outContact.bodyId,
                            static_cast<int>(outContact.partKind),
                            static_cast<int>(outContact.supportGripRole),
                            static_cast<int>(outContact.reloadRole),
                            static_cast<int>(outContact.actionRole),
                            g_rockConfig.rockWeaponInteractionProbeRadius);
                    }
                } else {
                    const auto missed = missedFrames.fetch_add(1, std::memory_order_acq_rel) + 1;
                    if (missed > WEAPON_CONTACT_TIMEOUT_FRAMES) {
                        clearWeaponContact(isLeft);
                    }
                }
            } else {
                const auto missed = missedFrames.fetch_add(1, std::memory_order_acq_rel) + 1;
                if (missed > WEAPON_CONTACT_TIMEOUT_FRAMES) {
                    clearWeaponContact(isLeft);
                }
            }

            outContact.acquisitionSource = weapon_interaction_acquisition_policy::resolve(
                acquisitionState,
                touchObserved,
                outContact.valid);
            switch (outContact.acquisitionSource) {
            case WeaponInteractionAcquisitionSource::PhysicalContact:
                return weapon_debug_notification_policy::WeaponContactSource::Contact;
            case WeaponInteractionAcquisitionSource::ProximityProbe:
                return weapon_debug_notification_policy::WeaponContactSource::Probe;
            case WeaponInteractionAcquisitionSource::None:
            default:
                return weapon_debug_notification_policy::WeaponContactSource::None;
            }
        };

        // A loose-weapon equip carries the originating physical hand into
        // the first equipped frame; use it immediately so input/contact
        // routing never spends a frame under the default right-hand role.
        if (_pendingEquippedWeaponPrimaryOnlyGripStart.pending) {
            _pendingEquippedWeaponPrimaryOnlyGripStart.remainingSeconds -=
                (std::max)(0.0f, frame.deltaSeconds);
            if (_pendingEquippedWeaponPrimaryOnlyGripStart.remainingSeconds <= 0.0f) {
                ROCK_LOG_WARN(Weapon,
                    "Held weapon manual ownership handoff expired targetForm={:08X} targetInstance={:#x}",
                    _pendingEquippedWeaponPrimaryOnlyGripStart.targetWeaponFormID,
                    _pendingEquippedWeaponPrimaryOnlyGripStart.targetWeaponInstanceData);
                _pendingEquippedWeaponPrimaryOnlyGripStart = {};
            }
        }
        auto* observedEquippedWeapon = currentEquippedWeaponForm();
        const std::uint32_t observedEquippedWeaponFormID =
            observedEquippedWeapon ? observedEquippedWeapon->formID : 0;
        const auto observedEquippedWeaponInstanceData =
            reinterpret_cast<std::uintptr_t>(
                currentEquippedWeaponInstanceData(observedEquippedWeapon));
        const bool equippedWeaponShoulderStashActive =
            equipped_weapon_drop_policy::equippedWeaponShoulderStashAvailable(
                _equippedWeaponHandlingSettings.equippedWeaponShoulderStashEnabled);
        const bool inputBlockingMenuActive =
            input_remap_runtime::isMenuInputActive();
        serviceEquippedWeaponShoulderSheathRetrieval(
            frame,
            equippedWeaponShoulderStashActive,
            inputBlockingMenuActive,
            observedEquippedWeaponFormID,
            observedEquippedWeaponInstanceData);
        const bool pendingPrimaryStartMatchesCurrentWeapon =
            _pendingEquippedWeaponPrimaryOnlyGripStart.pending &&
            (_pendingEquippedWeaponPrimaryOnlyGripStart.targetWeaponFormID == 0 ||
                equipped_weapon_transition_policy::matchesExpectedIdentity(
                    observedEquippedWeaponFormID,
                    observedEquippedWeaponInstanceData,
                    _pendingEquippedWeaponPrimaryOnlyGripStart.targetWeaponFormID,
                    _pendingEquippedWeaponPrimaryOnlyGripStart.targetWeaponInstanceData,
                    _pendingEquippedWeaponPrimaryOnlyGripStart.previousWeaponFormID,
                    _pendingEquippedWeaponPrimaryOnlyGripStart.previousWeaponInstanceData));
        const bool firingHandIsLeft = pendingPrimaryStartMatchesCurrentWeapon ?
            _pendingEquippedWeaponPrimaryOnlyGripStart.isLeft :
            _twoHandedGrip.isFiringHandLeft();
        const bool supportHandIsLeft = !firingHandIsLeft;

        /*
         * While the LEFT hand carries the weapon, the node still sits at
         * FRIK's offhand glue pose here; the ranked part probes below
         * convert real palm points into node-local space, so glue space
         * made a forend grab select the scope's sight body ~10gu away
         * (fallback wrap pose, grab churn). Publish the canonical carry
         * pose first so both hands probe the weapon where it actually is.
         */
        (void)_twoHandedGrip.publishLeftFiringFeedForwardWeaponPose(weaponNode);

        leftWeaponContactSource = consumeWeaponContactForHand(true, frame.left, weaponNode != nullptr, leftWeaponContact);
        // The free firing hand needs weapon-part probes for part grips and
        // for the reattach squeeze's proximity check, exactly like the
        // offhand; while the LEFT hand fires, the right hand is the
        // support/free hand and probes unconditionally.
        const bool rightWeaponContactProbeAllowed = weaponNode != nullptr &&
            (_twoHandedGrip.isPartCarryActive() || firingHandIsLeft);
        (void)consumeWeaponContactForHand(false, frame.right, rightWeaponContactProbeAllowed, rightWeaponContact);

        weaponFrame.observedWeapon = observedEquippedWeapon;
        weaponFrame.observedFormId = observedEquippedWeaponFormID;
        weaponFrame.observedInstanceData =
            observedEquippedWeaponInstanceData;
        weaponFrame.shoulderStashActive =
            equippedWeaponShoulderStashActive;
        weaponFrame.menuInputActive = inputBlockingMenuActive;
        weaponFrame.pendingPrimaryStartMatches =
            pendingPrimaryStartMatchesCurrentWeapon;
        weaponFrame.firingHandIsLeft = firingHandIsLeft;
        weaponFrame.supportHandIsLeft = supportHandIsLeft;
    }


    void PhysicsInteraction::serviceEquippedWeaponGripFrame(
        const PhysicsFrameContext& frame,
        EquippedWeaponFrame& weaponFrame)
    {
        auto* hknp = frame.hknpWorld;
        auto* weaponNode = weaponFrame.weaponNode;
        const auto currentWeaponGenerationKey =
            weaponFrame.generationKey;
        const auto currentEquippedWeaponOwnershipKey =
            weaponFrame.ownershipKey;
        auto& leftWeaponContact = weaponFrame.leftContact;
        auto& rightWeaponContact = weaponFrame.rightContact;
        const auto leftWeaponContactSource =
            weaponFrame.leftContactSource;
        auto* observedEquippedWeapon =
            weaponFrame.observedWeapon;
        const auto observedEquippedWeaponFormID =
            weaponFrame.observedFormId;
        const auto observedEquippedWeaponInstanceData =
            weaponFrame.observedInstanceData;
        const bool equippedWeaponShoulderStashActive =
            weaponFrame.shoulderStashActive;
        const bool inputBlockingMenuActive =
            weaponFrame.menuInputActive;
        const bool pendingPrimaryStartMatchesCurrentWeapon =
            weaponFrame.pendingPrimaryStartMatches;
        const bool firingHandIsLeft =
            weaponFrame.firingHandIsLeft;
        const bool supportHandIsLeft =
            weaponFrame.supportHandIsLeft;
        const bool rightHandWeaponEquipped =
            weaponFrame.rightHandWeaponEquipped;
        const bool retainedWeaponCollisionActive =
            weaponFrame.retainedWeaponCollisionActive;
        const bool rightHandWeaponAuthorityActiveBeforeGrip =
            weaponFrame.rightHandWeaponAuthorityActiveBeforeGrip;
        auto& rightHandWeaponAuthorityActive =
            weaponFrame.rightHandWeaponAuthorityActive;
        auto& leftSupportGripActive =
            weaponFrame.leftSupportGripActive;
        auto& rightPartGripActive =
            weaponFrame.rightPartGripActive;
        auto& drivenSourceNodes =
            weaponFrame.drivenSourceNodes;
        auto& drivenSourceNodeCount =
            weaponFrame.drivenSourceNodeCount;
        auto*& gunstockProjectileNode =
            weaponFrame.gunstockProjectileNode;
        const auto& runtime = runtime_state::currentFrame();

        const bool gripPressed = readGrabButtonHeld(true, g_rockConfig.rockGrabButtonID);
        const bool rightGripHeld = readGrabButtonHeld(false, g_rockConfig.rockGrabButtonID);
        const bool gripConfirmPressed = readGrabButtonPressedEdge(true, g_rockConfig.rockGrabButtonID);
        (void)gripConfirmPressed;

        WeaponInteractionRuntimeState providerInteractionState{};

        ::rock::provider::RockProviderWeaponPartTargetResolutionV1 weaponPartResolution{};
        const auto weaponPartQuery = makeProviderWeaponPartTargetQuery(leftWeaponContact, _weaponCollision);
        const bool weaponPartResolved = leftWeaponContact.valid &&
            ::rock::provider::resolveWeaponPartTargetV1(weaponPartQuery, weaponPartResolution);
        const bool weaponPartWhitelistActive = weaponPartResolved && weaponPartResolution.whitelistActive != 0;
        const bool weaponPartMatched = weaponPartResolved && weaponPartResolution.matched != 0;
        if (weaponPartWhitelistActive && !weaponPartMatched) {
            providerInteractionState.supportGripAllowed = false;
        } else if (weaponPartMatched) {
            providerInteractionState.providerPartAuthority = makeWeaponProviderPartAuthority(weaponPartQuery, weaponPartResolution);
        }

        WeaponInteractionRuntimeState rightHandInteractionState{};
        ::rock::provider::RockProviderWeaponPartTargetResolutionV1 rightWeaponPartResolution{};
        const auto rightWeaponPartQuery = makeProviderWeaponPartTargetQuery(rightWeaponContact, _weaponCollision);
        const bool rightWeaponPartResolved = rightWeaponContact.valid &&
            ::rock::provider::resolveWeaponPartTargetV1(rightWeaponPartQuery, rightWeaponPartResolution);
        const bool rightWeaponPartWhitelistActive = rightWeaponPartResolved && rightWeaponPartResolution.whitelistActive != 0;
        const bool rightWeaponPartMatched = rightWeaponPartResolved && rightWeaponPartResolution.matched != 0;
        if (rightWeaponPartWhitelistActive && !rightWeaponPartMatched) {
            rightHandInteractionState.supportGripAllowed = false;
        } else if (rightWeaponPartMatched) {
            rightHandInteractionState.providerPartAuthority = makeWeaponProviderPartAuthority(rightWeaponPartQuery, rightWeaponPartResolution);
        }

        /*
         * The offhand reservation is a SUPPORT-ROLE gate, not a physical
         * left-hand gate: it constrains whichever hand currently plays the
         * support role. Part grips by the free firing hand stay gated by
         * the provider part whitelist alone so PAPER reload sessions still
         * constrain which parts the free hand may take.
         */
        const auto offhandReservation = offhand_interaction_reservation::fromProvider(::rock::provider::currentOffhandReservation());
        if (!offhand_interaction_reservation::allowsSupportGrip(offhandReservation)) {
            (supportHandIsLeft ? providerInteractionState : rightHandInteractionState).supportGripAllowed = false;
        }

        const WeaponInteractionDecision leftWeaponDecision = routeWeaponInteraction(leftWeaponContact, providerInteractionState);
        const auto weaponNotificationKey = weapon_debug_notification_policy::makeWeaponNotificationKey(
            leftWeaponContact,
            leftWeaponDecision,
            leftWeaponContactSource);

        const bool leftHandHoldingObject = _leftHand.isHolding();
        auto supportAuthorityMode = weapon_support_authority_policy::WeaponSupportAuthorityMode::FullTwoHandedSolver;
        bool supportAuthorityProviderOverride = false;
        // The grab-mode override follows the SUPPORT-ROLE hand's provider
        // resolution: that is the hand whose grip the mode describes.
        const bool supportWeaponPartMatched = supportHandIsLeft ? weaponPartMatched : rightWeaponPartMatched;
        const auto& supportWeaponPartResolution = supportHandIsLeft ? weaponPartResolution : rightWeaponPartResolution;
        if (supportWeaponPartMatched) {
            if (supportWeaponPartResolution.grabMode == ::rock::provider::RockProviderWeaponPartGrabModeV1::FullTwoHandAuthority) {
                supportAuthorityMode = weapon_support_authority_policy::WeaponSupportAuthorityMode::FullTwoHandedSolver;
                supportAuthorityProviderOverride = true;
            } else if (supportWeaponPartResolution.grabMode == ::rock::provider::RockProviderWeaponPartGrabModeV1::AttachOnly) {
                supportAuthorityMode = weapon_support_authority_policy::WeaponSupportAuthorityMode::VisualOnlySupport;
                supportAuthorityProviderOverride = true;
            }
        }
        const bool firingGripProximityAuthorityEnabled = weapon_support_authority_policy::canApplyFiringGripProximityAuthority(
            supportAuthorityProviderOverride);
        EquippedWeaponPrimaryGripInput primaryGripInput{};
        GrabButtonState primaryGrabState{};
        bool primaryGrabStateRead = false;
        _firingHandGrabButtonFrameState = {};
        auto readPrimaryGrabState = [&]() -> const GrabButtonState& {
            if (!primaryGrabStateRead) {
                primaryGrabState = readGrabButtonState(firingHandIsLeft, g_rockConfig.rockGrabButtonID);
                // Menu rearm intentionally masks gameplay edges, but
                // firing-grip ownership still follows the physical hand
                // state after the menu closes.
                primaryGrabState.held = input_remap_runtime::isRawButtonPhysicallyHeld(firingHandIsLeft, g_rockConfig.rockGrabButtonID);
                primaryGrabStateRead = true;
                // Publish the consumed snapshot so the normal grab pipeline
                // sees the same edges instead of re-consuming cleared ones.
                _firingHandGrabButtonFrameState = SharedGrabButtonFrameState{
                    .valid = true,
                    .isLeft = firingHandIsLeft,
                    .held = primaryGrabState.held,
                    .pressed = primaryGrabState.pressed,
                    .released = primaryGrabState.released,
                };
            }
            return primaryGrabState;
        };
        const bool primaryPoseBlockerAvailable = frik_visual_authority::canBlockPrimaryHandWeaponPose();
        const bool ambidextrousHandoffAvailable =
            _equippedWeaponHandlingSettings.ambidextrousHandoffEnabled &&
            TwoHandedGrip::canBeginPrimaryOnlyGripForHand(true);
        const bool firingGripOwnershipFeatureAvailable = equipped_weapon_manual_ownership_policy::featureAvailable(
            !_equippedWeaponShoulderSheath.active &&
                _equippedWeaponHandlingSettings.firingGripOwnershipEnabled,
            primaryPoseBlockerAvailable,
            weaponNode != nullptr,
            currentEquippedWeaponOwnershipKey);
        const bool primaryDetachFeatureAvailable = equipped_weapon_manual_ownership_policy::featureAvailable(
            !_equippedWeaponShoulderSheath.active &&
                _equippedWeaponHandlingSettings.primaryDetachEnabled,
            primaryPoseBlockerAvailable,
            weaponNode != nullptr,
            currentEquippedWeaponOwnershipKey);
        if (inputBlockingMenuActive) {
            _pendingEquippedWeaponPrimaryOnlyGripStart = {};
        } else if (_pendingEquippedWeaponPrimaryOnlyGripStart.pending &&
            !equipped_weapon_manual_ownership_policy::shouldKeepPendingPrimaryOnlyStart(
                equipped_weapon_manual_ownership_policy::PendingPrimaryOnlyStartInput{
                    .pending = _pendingEquippedWeaponPrimaryOnlyGripStart.pending,
                    .gripHeld = input_remap_runtime::isRawButtonPhysicallyHeld(firingHandIsLeft, g_rockConfig.rockGrabButtonID),
                    .committedTransfer =
                        _pendingEquippedWeaponPrimaryOnlyGripStart.
                            committedTransfer,
                    .ownershipModeEnabled = _equippedWeaponHandlingSettings.firingGripOwnershipEnabled,
                    .primaryPoseBlockerAvailable = primaryPoseBlockerAvailable,
                })) {
            _pendingEquippedWeaponPrimaryOnlyGripStart = {};
        }
        const input_remap_policy::EquippedWeaponFiringGripInputGate firingGripInputGate{
            .featureAvailable = firingGripOwnershipFeatureAvailable,
            .canUseFiringGripInput = _twoHandedGrip.canUseFiringGripInput(),
            .menuInputActive = inputBlockingMenuActive,
        };
        if (input_remap_policy::shouldConsumeEquippedWeaponFiringGripInput(firingGripInputGate)) {
            const auto& primaryState = readPrimaryGrabState();
            if (input_remap_policy::shouldUseEquippedWeaponFiringGripInput(firingGripInputGate)) {
                primaryGripInput = EquippedWeaponPrimaryGripInput{
                    .held = primaryState.held,
                    .pressed = primaryState.pressed,
                    .released = primaryState.released,
                };
            }
        }

        bool primaryOnlyGripStartedThisFrame = false;
        if (firingGripOwnershipFeatureAvailable && !inputBlockingMenuActive && !_twoHandedGrip.isManualOwnershipActive()) {
            const auto& primaryState = readPrimaryGrabState();
            if (_pendingEquippedWeaponPrimaryOnlyGripStart.pending &&
                !primaryState.held &&
                !_pendingEquippedWeaponPrimaryOnlyGripStart.
                    committedTransfer) {
                _pendingEquippedWeaponPrimaryOnlyGripStart = {};
            }

            const bool pendingPrimaryOnlyStartRequested =
                equipped_weapon_manual_ownership_policy::
                    shouldStartPendingPrimaryOnlyGrip(
                        pendingPrimaryStartMatchesCurrentWeapon,
                        primaryState.held,
                        _pendingEquippedWeaponPrimaryOnlyGripStart.
                            committedTransfer);
            const bool primaryOnlyStartRequested =
                weaponNode != nullptr &&
                currentEquippedWeaponOwnershipKey != 0 &&
                ((primaryDetachFeatureAvailable && primaryState.held &&
                     primaryState.pressed) ||
                    pendingPrimaryOnlyStartRequested);
            if (pendingPrimaryStartMatchesCurrentWeapon &&
                _pendingEquippedWeaponPrimaryOnlyGripStart.isLeft &&
                (!_pendingEquippedWeaponPrimaryOnlyGripStart.hasFiringHandWeaponLocal ||
                    !_pendingEquippedWeaponPrimaryOnlyGripStart.hasFiringGripWeaponLocal)) {
                _pendingEquippedWeaponPrimaryOnlyGripStart.hasFiringHandWeaponLocal =
                    _twoHandedGrip.tryBuildCurrentLeftFiringGripCapture(
                        weaponNode,
                        currentWeaponGenerationKey,
                        currentEquippedWeaponOwnershipKey,
                        _pendingEquippedWeaponPrimaryOnlyGripStart.firingHandWeaponLocal,
                        _pendingEquippedWeaponPrimaryOnlyGripStart.firingGripWeaponLocal);
                _pendingEquippedWeaponPrimaryOnlyGripStart.hasFiringGripWeaponLocal =
                    _pendingEquippedWeaponPrimaryOnlyGripStart.hasFiringHandWeaponLocal;
            }
            const RE::NiTransform* capturedFiringHandWeaponLocal =
                pendingPrimaryStartMatchesCurrentWeapon &&
                    _pendingEquippedWeaponPrimaryOnlyGripStart.hasFiringHandWeaponLocal ?
                &_pendingEquippedWeaponPrimaryOnlyGripStart.firingHandWeaponLocal :
                nullptr;
            const RE::NiPoint3* capturedFiringGripWeaponLocal =
                pendingPrimaryStartMatchesCurrentWeapon &&
                    _pendingEquippedWeaponPrimaryOnlyGripStart.hasFiringGripWeaponLocal ?
                &_pendingEquippedWeaponPrimaryOnlyGripStart.firingGripWeaponLocal :
                nullptr;
            const bool committedTransfer =
                pendingPrimaryStartMatchesCurrentWeapon &&
                _pendingEquippedWeaponPrimaryOnlyGripStart.
                    committedTransfer;
            if (primaryOnlyStartRequested &&
                _twoHandedGrip.beginPrimaryOnlyGrip(
                    weaponNode,
                    currentWeaponGenerationKey,
                    currentEquippedWeaponOwnershipKey,
                    firingHandIsLeft,
                    capturedFiringHandWeaponLocal,
                    capturedFiringGripWeaponLocal,
                    committedTransfer)) {
                primaryOnlyGripStartedThisFrame = true;
                _pendingEquippedWeaponPrimaryOnlyGripStart = {};
                primaryGripInput = EquippedWeaponPrimaryGripInput{
                    .held = primaryState.held,
                    .pressed = primaryState.pressed,
                    .released = primaryState.released,
                };
            }
        } else if (inputBlockingMenuActive) {
            _pendingEquippedWeaponPrimaryOnlyGripStart = {};
        }

        if (weaponNode) {
            drivenSourceNodeCount = applyProviderWeaponPartDrives(
                weaponNode,
                currentWeaponGenerationKey,
                frame,
                drivenSourceNodes);
        } else {
            _providerWeaponPartDriveResultCount = 0;
        }

        /*
         * Firing-grip reattach is the squeeze gesture (grab held with the
         * palm on the grip); distance is evaluated by TwoHandedGrip. This
         * only gates whether each free hand may be captured at all -
         * either hand can take the firing grip when ambidextrous takeover
         * is available.
         */
        bool leftReattachEligible = false;
        bool rightReattachEligible = false;
        if (_twoHandedGrip.isPartCarryActive() && primaryDetachFeatureAvailable) {
            leftReattachEligible = weapon_two_handed_grip_math::canAttemptFiringGripReattach(
                weapon_two_handed_grip_math::FiringGripReattachInput{
                    .partCarryActive = true,
                    .menuInputActive = inputBlockingMenuActive,
                    .handHoldingObject = _leftHand.isHolding(),
                });
            rightReattachEligible = weapon_two_handed_grip_math::canAttemptFiringGripReattach(
                weapon_two_handed_grip_math::FiringGripReattachInput{
                    .partCarryActive = true,
                    .menuInputActive = inputBlockingMenuActive,
                    .handHoldingObject = _rightHand.isHolding(),
                });
        }

        /*
         * Equipped-weapon shoulder stash: evaluated before update() so the
         * release frame has a fresh in-zone decision. Provider-owned
         * physical detach follows the single manual carry hand; ROCK's
         * native path follows the current firing hand without granting
         * world-drop authority. The idle hand is always reset so stale
         * dwell can never confirm a later release.
         */
        std::array<shoulder_stash::Decision, 2> equippedWeaponStashCommitDecisions{};
        bool nativeShoulderSheathRequested = false;
        auto nativeShoulderSheathSourceHand =
            equipped_weapon_drop_policy::SourceHand::None;
        {
            // Carry-authority grips remain authoritative for provider
            // detach mode. Without that mode, the natively attached
            // firing hand owns ROCK's dedicated shoulder release gesture.
            const auto manualStashCarryHand =
                equipped_weapon_drop_policy::resolveEquippedWeaponStashCarryHand(
                    _twoHandedGrip.isPrimaryOnlyActive(),
                    _twoHandedGrip.isPartCarryActive(),
                    _twoHandedGrip.isHandPartCarryGripping(true),
                    _twoHandedGrip.isHandPartCarryGripping(false),
                    _twoHandedGrip.isFiringHandLeft());
            const bool nativeShoulderGestureAvailable =
                equippedWeaponShoulderStashActive &&
                !_equippedWeaponHandlingSettings.primaryDetachEnabled;
            const auto nativeShoulderGestureHand = firingHandIsLeft ?
                equipped_weapon_drop_policy::SourceHand::Left :
                equipped_weapon_drop_policy::SourceHand::Right;
            auto stashCarryHand =
                equipped_weapon_drop_policy::SourceHand::None;
            if (equippedWeaponShoulderStashActive) {
                stashCarryHand = manualStashCarryHand !=
                        equipped_weapon_drop_policy::SourceHand::None ?
                    manualStashCarryHand :
                    nativeShoulderGestureAvailable ?
                    nativeShoulderGestureHand :
                    equipped_weapon_drop_policy::SourceHand::None;
            }
            const bool stashCarryEligible = !inputBlockingMenuActive &&
                                            stashCarryHand != equipped_weapon_drop_policy::SourceHand::None;
            for (const bool stashHandIsLeft : { true, false }) {
                const std::size_t stashHandIndex = stashHandIsLeft ? 1u : 0u;
                auto& stashState = _equippedWeaponStashStates[stashHandIndex];
                auto& commitLease = _equippedWeaponStashCommitLeases[stashHandIndex];
                if (!stashCarryEligible || equipped_weapon_drop_policy::isLeft(stashCarryHand) != stashHandIsLeft) {
                    shoulder_stash::resetRuntime(stashState);
                    commitLease = {};
                    continue;
                }

                const HandFrameInput& carryInput = stashHandIsLeft ? frame.left : frame.right;
                const auto stashConfig = makeEquippedWeaponStashDetectorConfig(equippedWeaponShoulderStashActive);
                shoulder_stash::DetectorInput stashInput{
                        .isLeftHand = stashHandIsLeft,
                        .probe = shoulder_stash::Probe{ .pointGame = carryInput.grabAnchorWorld },
                        .hmdProbe = makeShoulderStashHmdProbe(carryInput),
                        .hasHmdProbe = true,
                        .hasHmdFrame = frame.hasHmdFrame,
                        .hmdPositionWorld = frame.hmdPositionWorld,
                        .hmdForwardWorld = frame.hmdForwardWorld,
                        .deltaSeconds = frame.deltaSeconds,
                        .config = stashConfig,
                    };
                const shoulder_stash::RuntimeState stashStateBeforeEvaluation = stashState;
                const auto stashDecision = shoulder_stash::evaluate(stashInput, stashState);
                equippedWeaponStashCommitDecisions[stashHandIndex] = stashDecision;

                const bool gripPhysicallyHeld =
                    input_remap_runtime::isRawButtonPhysicallyHeld(stashHandIsLeft, g_rockConfig.rockGrabButtonID);
                if (gripPhysicallyHeld) {
                    commitLease = {};
                } else if (!stashDecision.confirmedForCommit) {
                    const bool speedLimitExceeded =
                        shoulder_stash::exceedsShoulderStashSpeedLimit(
                            stashDecision.speedGameUnitsPerSecond,
                            stashConfig.maxSpeedGameUnitsPerSecond);
                    const bool canArmFastReleaseLease = stashStateBeforeEvaluation.confirmed && speedLimitExceeded;
                    if (commitLease.active || canArmFastReleaseLease) {
                        /*
                         * The normal detector remains the speed authority.
                         * A speed-unlimited copy is used only to prove that
                         * the already-dwelled hand stayed in the same back
                         * volume during the two-frame physical release
                         * debounce; it cannot acquire a new stash candidate.
                         */
                        auto spatialInput = stashInput;
                        spatialInput.config.maxSpeedGameUnitsPerSecond = 0.0f;
                        auto spatialState = commitLease.active ? commitLease.spatialState : stashStateBeforeEvaluation;
                        const auto spatialDecision = shoulder_stash::evaluate(spatialInput, spatialState);
                        const auto expectedZone = commitLease.active ? commitLease.zone : stashStateBeforeEvaluation.zone;
                        const auto expectedSource = commitLease.active ? commitLease.source : stashStateBeforeEvaluation.source;
                        const bool sameSpatialCandidate =
                            spatialDecision.candidate &&
                            spatialDecision.zone == expectedZone &&
                            spatialDecision.source == expectedSource;

                        if (!commitLease.active &&
                            shoulder_stash::shouldArmEquippedWeaponFastReleaseCommitLease(
                                stashStateBeforeEvaluation.confirmed,
                                speedLimitExceeded,
                                gripPhysicallyHeld,
                                sameSpatialCandidate)) {
                            commitLease.active = true;
                            commitLease.ownershipKey = currentEquippedWeaponOwnershipKey;
                            commitLease.remainingOpenFrames =
                                equipped_weapon_manual_ownership_policy::kPrimaryReleaseConfirmFrames;
                            commitLease.zone = spatialDecision.zone;
                            commitLease.source = spatialDecision.source;
                        }

                        if (shoulder_stash::equippedWeaponFastReleaseCommitLeaseIsUsable(
                                commitLease.active,
                                commitLease.ownershipKey,
                                currentEquippedWeaponOwnershipKey,
                                commitLease.remainingOpenFrames,
                                gripPhysicallyHeld,
                                sameSpatialCandidate)) {
                            commitLease.spatialState = spatialState;
                            equippedWeaponStashCommitDecisions[stashHandIndex] = spatialDecision;
                            equippedWeaponStashCommitDecisions[stashHandIndex].confirmedForCommit = true;
                            --commitLease.remainingOpenFrames;
                        } else {
                            commitLease = {};
                        }
                    }
                } else {
                    commitLease = {};
                }

                if (nativeShoulderGestureAvailable &&
                    stashHandIsLeft == firingHandIsLeft &&
                    equippedWeaponStashCommitDecisions[stashHandIndex].
                        confirmedForCommit) {
                    Hand& stashHand = stashHandIsLeft ?
                        _leftHand : _rightHand;
                    const bool stashHandEmpty =
                        !stashHand.isHolding() &&
                        !_touchGrabRuntime.isHandActive(
                            stashHandIsLeft) &&
                        !_pendingForceGrabCommits[stashHandIndex].active &&
                        !stashHand.hasActivePullCatchIntent() &&
                        !stashHand.
                            hasPendingActorEquipmentDropHandoff();
                    const auto& primaryState = readPrimaryGrabState();
                    nativeShoulderSheathRequested =
                        equipped_weapon_drop_policy::
                            canCommitNativeShoulderSheath(
                                equipped_weapon_drop_policy::
                                    NativeShoulderSheathInput{
                                        .handlingEnabled =
                                            equippedWeaponShoulderStashActive,
                                        .primaryDetachEnabled =
                                            _equippedWeaponHandlingSettings.
                                                primaryDetachEnabled,
                                        .weaponAvailable =
                                            weaponNode != nullptr &&
                                            currentEquippedWeaponOwnershipKey != 0,
                                        .menuInputActive =
                                            inputBlockingMenuActive,
                                        .handDisabled =
                                            carryInput.disabled,
                                        .handEmpty = stashHandEmpty,
                                        .detectorConfirmed = true,
                                        .gripReleased =
                                            primaryState.released,
                                    });
                    if (nativeShoulderSheathRequested) {
                        nativeShoulderSheathSourceHand =
                            nativeShoulderGestureHand;
                    }
                }

                if (stashDecision.candidate &&
                    g_rockConfig.rockShoulderStashHapticsEnabled &&
                    shouldEmitShoulderStashCandidatePulse(
                        stashDecision,
                        stashState,
                        _dynamicPushElapsedSeconds)) {
                        (void)_feedbackHaptics.queue(
                            stashHandIsLeft ? feedback_haptics::FeedbackHand::Left : feedback_haptics::FeedbackHand::Right,
                            g_rockConfig.rockShoulderStashCandidateHapticDurationSeconds,
                            shoulder_stash_haptic_policy::computeCandidatePulseIntensity(stashDecision.confidence,
                                shoulder_stash_haptic_policy::CandidatePulseConfig{
                                    .enabled = true,
                                    .baseIntensity = g_rockConfig.rockShoulderStashCandidateHapticBaseIntensity,
                                    .maxIntensity = g_rockConfig.rockShoulderStashCandidateHapticIntensity,
                                }));
                }
            }
        }

        const auto captureScopeHandDriverFrame = [](RE::NiNode* driverNode) {
            EquippedWeaponScopeHandDriverFrame result{};
            if (driverNode && finiteNiTransform(driverNode->world)) {
                result.valid = true;
                result.world = driverNode->world;
            }
            return result;
        };
        auto* playerNodes = f4vr::getPlayerNodes();
        const auto scopeHandDriverNode = [playerNodes](bool isLeft) -> RE::NiNode* {
            if (!playerNodes) {
                return nullptr;
            }
            return isLeft ?
                playerNodes->SecondaryMeleeWeaponOffsetNode2 :
                playerNodes->primaryWeaponOffsetNOde;
        };
        const EquippedWeaponScopeHandDriverFrame leftHandDriverFrame{
            !frame.left.disabled && finiteNiTransform(frame.left.rawHandWorld),
            frame.left.rawHandWorld,
        };
        const EquippedWeaponScopeHandDriverFrame rightHandDriverFrame{
            !frame.right.disabled && finiteNiTransform(frame.right.rawHandWorld),
            frame.right.rawHandWorld,
        };
        const EquippedWeaponScopeHandDriverFrame leftScopeHandDriverFrame = captureScopeHandDriverFrame(scopeHandDriverNode(true));
        const EquippedWeaponScopeHandDriverFrame rightScopeHandDriverFrame = captureScopeHandDriverFrame(scopeHandDriverNode(false));
        bool nativeScopeRequestActive = false;
        const bool nativeScopeRequestStateValid =
            tryReadNativeScopeRequestState(nativeScopeRequestActive);
        const bool manualScopeActivationRequested =
            input_remap_runtime::isManualScopeActivationRequested();
        const bool gunstockObservationActive =
            g_rockConfig.rockGunstockModeEnabled ||
            g_rockConfig.rockDebugDrawGunstockAlignment;
        gunstockProjectileNode =
            gunstockObservationActive ?
            getEquippedProjectileNode() :
            nullptr;
        /*
         * FO4VR 1.2.72 binary verification (2026-08-06): native
         * TESObjectWEAP paths at 0x14033FF00 and 0x140334260 read the type
         * byte at object +0x2CF; the native type-label table's index 9 is
         * referenced by CombatBehaviorTreeGun. Pair that kGun witness with
         * the collision observer's form boundary before it may establish
         * generation-latched gunstock eligibility.
         */
        const bool gunstockGunTypeObserved =
            gunstockObservationActive &&
            observedEquippedWeapon &&
            currentWeaponGenerationKey != 0 &&
            _weaponCollision.getCurrentObservedEquippedWeaponFormID() ==
                observedEquippedWeapon->formID &&
            observedEquippedWeapon->weaponData.type ==
                RE::WEAPON_TYPE::kGun;

        const EquippedWeaponGripFrameInput gripFrameInput{
            .leftGripHeld = gripPressed,
            .rightGripHeld = rightGripHeld,
            .leftHandHoldingObject = leftHandHoldingObject,
            .rightHandHoldingObject = _rightHand.isHolding(),
            .leftReattachEligible = leftReattachEligible,
            .rightReattachEligible = rightReattachEligible,
            .scopeMenuOpen = runtime.localScopeMenuOpen,
            .manualScopeActivationRequested = manualScopeActivationRequested,
            .nativeScopeRequestStateValid = nativeScopeRequestStateValid,
            .nativeScopeRequestActive = nativeScopeRequestActive,
            .nativeReloadHandAuthorityActive =
                frame.nativeReloadHandAuthorityActive,
            .gunstockPresentationBlocked =
                frame.menuBlocked ||
                frame.nativeReloadHandAuthorityActive,
            .leftHandDriverFrame = leftHandDriverFrame,
            .rightHandDriverFrame = rightHandDriverFrame,
            .leftScopeHandDriverFrame = leftScopeHandDriverFrame,
            .rightScopeHandDriverFrame = rightScopeHandDriverFrame,
            .primaryGripInput = primaryGripInput,
        };
        auto effectiveHandlingSettings = _equippedWeaponHandlingSettings;
        effectiveHandlingSettings.firingGripOwnershipEnabled =
            firingGripOwnershipFeatureAvailable;
        effectiveHandlingSettings.ambidextrousHandoffEnabled =
            ambidextrousHandoffAvailable;
        effectiveHandlingSettings.primaryDetachEnabled =
            primaryDetachFeatureAvailable;
        _twoHandedGrip.update(
            weaponNode,
            gunstockProjectileNode,
            gunstockGunTypeObserved,
            leftWeaponContact,
            rightWeaponContact,
            gripFrameInput,
            frame.deltaSeconds,
            _currentPreFrikSchedulerSequence,
            currentWeaponGenerationKey,
            currentEquippedWeaponOwnershipKey,
            _weaponCollision,
            providerInteractionState,
            rightHandInteractionState,
            supportAuthorityMode,
            firingGripProximityAuthorityEnabled,
            effectiveHandlingSettings);
        synchronizeDynamicWeaponHandCollisionRoles(hknp);
        const bool gunstockNeutralSampleBlocked =
            g_rockConfig.rockGunstockModeEnabled &&
            (input_remap_runtime::isRawButtonPhysicallyHeld(
                 _twoHandedGrip.isFiringHandLeft(),
                 input_remap_policy::
                     kOpenVrSteamVrTriggerButtonId) ||
                 frame.nativeReloadHandAuthorityActive);
        const bool gunstockPresentationBlocked =
            frame.menuBlocked ||
            frame.nativeReloadHandAuthorityActive;
        _twoHandedGrip.prepareGunstockAlignmentDebugSnapshot(
            weaponNode,
            gunstockProjectileNode,
            _weaponCollision.
                getCurrentObservedEquippedWeaponFormID(),
            currentWeaponGenerationKey,
            gunstockNeutralSampleBlocked,
            gunstockPresentationBlocked);
        (void)_twoHandedGrip.applyGunstockAlignment(
            weaponNode,
            gunstockProjectileNode,
            currentWeaponGenerationKey,
            gunstockNeutralSampleBlocked,
            gunstockPresentationBlocked);
        reconcileEquippedWeaponHandAssignmentAfterGrip();
        if (_twoHandedGrip.hasVisualAuthorityForHand(false)) {
            _rightHand.cancelGrabVisualReturn("equipped-weapon-visual-authority");
        }
        if (_twoHandedGrip.hasVisualAuthorityForHand(true)) {
            _leftHand.cancelGrabVisualReturn("equipped-weapon-visual-authority");
        }
        if (primaryOnlyGripStartedThisFrame) {
            ROCK_LOG_DEBUG(Weapon, "Equipped weapon firing-grip ownership started from grip input or held-weapon equip");
        }
        const auto gripHapticEvents = _twoHandedGrip.consumeHapticEvents();
        const auto queueGripHaptic = [this](bool isLeft, float intensity) {
            (void)_feedbackHaptics.queue(
                isLeft ? feedback_haptics::FeedbackHand::Left : feedback_haptics::FeedbackHand::Right,
                _equippedWeaponHandlingSettings.weaponGripHapticDurationSeconds,
                intensity);
        };
        if (_equippedWeaponHandlingSettings.externalAuthorityActive) {
            if (gripHapticEvents.firingGripAttached) {
                queueGripHaptic(
                    gripHapticEvents.firingGripAttachedHandIsLeft,
                    _equippedWeaponHandlingSettings.firingGripAttachHapticIntensity);
            }
            if (gripHapticEvents.firingGripDetached) {
                queueGripHaptic(
                    gripHapticEvents.firingGripDetachedHandIsLeft,
                    _equippedWeaponHandlingSettings.firingGripDetachHapticIntensity);
            }
            if (gripHapticEvents.leftPartGripCaptured) {
                queueGripHaptic(
                    true,
                    _equippedWeaponHandlingSettings.supportGripHapticIntensity);
            }
            if (gripHapticEvents.rightPartGripCaptured) {
                queueGripHaptic(
                    false,
                    _equippedWeaponHandlingSettings.supportGripHapticIntensity);
            }
        }
        /*
         * Continuous hover feedback while the open firing palm sits inside
         * the reattach radius during part carry: re-queued every frame so
         * the vibration holds until the squeeze reattaches (which flips
         * the state and hands off to the firingGripAttached pulse above).
         */
        if (_equippedWeaponHandlingSettings.gripZoneHoverHapticsEnabled &&
            _twoHandedGrip.isFiringGripReattachHoverInsideRadius()) {
            (void)_feedbackHaptics.queue(
                _twoHandedGrip.isFiringGripReattachHoverHandLeft() ? feedback_haptics::FeedbackHand::Left : feedback_haptics::FeedbackHand::Right,
                grip_zone_hover_haptic_policy::kContinuousQueueSeconds,
                _equippedWeaponHandlingSettings.gripZoneHoverHapticIntensity);
        }
        bool nativeShoulderSheathSelected = false;
        if (nativeShoulderSheathRequested &&
            nativeShoulderSheathSourceHand !=
                equipped_weapon_drop_policy::SourceHand::None) {
            nativeShoulderSheathSelected = true;
            const bool sheathHandIsLeft =
                equipped_weapon_drop_policy::isLeft(
                    nativeShoulderSheathSourceHand);
            const std::size_t sheathHandIndex =
                sheathHandIsLeft ? 1u : 0u;
            _equippedWeaponSheathCommittedThisFrame[sheathHandIndex] = true;
            (void)submitEquippedWeaponShoulderSheath(
                observedEquippedWeaponFormID,
                observedEquippedWeaponInstanceData,
                nativeShoulderSheathSourceHand,
                equippedWeaponStashCommitDecisions[sheathHandIndex],
                weaponNode,
                currentWeaponGenerationKey,
                currentEquippedWeaponOwnershipKey);
        }
        const auto equippedWeaponDropRequest = _twoHandedGrip.consumeEquippedWeaponDropRequest();
        if (equippedWeaponDropRequest.requested) {
            const auto sourceHand = equippedWeaponDropRequest.sourceHand;
            const bool sourceHandKnown = sourceHand == equipped_weapon_drop_policy::SourceHand::Right ||
                                          sourceHand == equipped_weapon_drop_policy::SourceHand::Left;
            const RE::NiPoint3 dropLoc = sourceHandKnown ?
                                            (equipped_weapon_drop_policy::isLeft(sourceHand) ? frame.left.grabAnchorWorld : frame.right.grabAnchorWorld) :
                                            (weaponNode ? weaponNode->world.translate : frame.right.grabAnchorWorld);
            if (inputBlockingMenuActive) {
                ROCK_LOG_INFO(Weapon,
                    "Equipped weapon manual release suppressed because an input-blocking menu is active sourceHand={} releaseLoc=({:.1f},{:.1f},{:.1f})",
                    equipped_weapon_drop_policy::sourceHandName(sourceHand),
                    dropLoc.x,
                    dropLoc.y,
                    dropLoc.z);
                _pendingEquippedWeaponPrimaryOnlyGripStart = {};
                clearEquippedWeaponFiringGripInputState();
            } else {
                const std::size_t sourceHandIndex =
                    sourceHandKnown &&
                        equipped_weapon_drop_policy::isLeft(sourceHand) ?
                    1u : 0u;
                const bool manualStashCommitSelected =
                    sourceHandKnown &&
                    equippedWeaponShoulderStashActive &&
                    equippedWeaponStashCommitDecisions[sourceHandIndex].
                        confirmedForCommit;
                const bool stashCommitSelected =
                    nativeShoulderSheathSelected ||
                    manualStashCommitSelected;
                if (manualStashCommitSelected &&
                    !nativeShoulderSheathSelected) {
                    /*
                     * Native sheathe is terminal for this release gesture. The
                     * exact equipped stack remains equipped and no world
                     * reference is created. Once this action is selected, a
                     * failed native transition must never become a world drop.
                     */
                    (void)submitEquippedWeaponShoulderSheath(
                        observedEquippedWeaponFormID,
                        observedEquippedWeaponInstanceData,
                        sourceHand,
                        equippedWeaponStashCommitDecisions[
                            sourceHandIndex],
                        weaponNode,
                        currentWeaponGenerationKey,
                        currentEquippedWeaponOwnershipKey);
                }
                const bool physicalDropRequested =
                    equipped_weapon_drop_policy::shouldAttemptPhysicalDrop(stashCommitSelected);
                const bool dropHandoffAvailable = hasAvailableEquippedWeaponDropHandoff();
                if (physicalDropRequested && !dropHandoffAvailable) {
                    ROCK_LOG_WARN(Weapon,
                        "Equipped weapon physical drop blocked because all native handoffs are active: capacity={}",
                        _equippedWeaponDropMomentumHandoffs.size());
                    f4vr::showNotification("ROCK: Cannot drop weapon - drop handoff queue is full.");
                }
                if (physicalDropRequested && dropHandoffAvailable) {
                    /*
                     * Seamless drop: spawn the world ref at the weapon's last
                     * visually-published pose (equipped and dropped weapons
                     * share the same nif) and hand the captured release
                     * momentum to the spawned physics bodies once they
                     * resolve. The previous-frame capture is preferred over
                     * the live node because the release transition restores
                     * the weapon node to the FRIK hand baseline before this
                     * code runs.
                     */
                    RE::NiPoint3 releaseLoc = dropLoc;
                    RE::NiPoint3 releaseRot{};
                    RE::NiTransform releaseWeaponWorld{};
                    bool hasReleaseRot = false;
                    if (_equippedWeaponReleaseCapture.hasWeaponWorld &&
                        finiteNiTransform(_equippedWeaponReleaseCapture.weaponWorld)) {
                        releaseWeaponWorld = _equippedWeaponReleaseCapture.weaponWorld;
                        releaseLoc = _equippedWeaponReleaseCapture.weaponWorld.translate;
                        releaseRot = grab_node_info_math::nifskopeMatrixToEulerRadians<RE::NiMatrix3, RE::NiPoint3>(_equippedWeaponReleaseCapture.weaponWorld.rotate);
                        hasReleaseRot = true;
                    } else if (weaponNode && finiteNiTransform(weaponNode->world)) {
                        releaseWeaponWorld = weaponNode->world;
                        releaseLoc = weaponNode->world.translate;
                        releaseRot = grab_node_info_math::nifskopeMatrixToEulerRadians<RE::NiMatrix3, RE::NiPoint3>(weaponNode->world.rotate);
                        hasReleaseRot = true;
                    }
                    const std::size_t releaseHandIndex = equipped_weapon_drop_policy::isLeft(sourceHand) ? 1u : 0u;
                    const auto& releaseHandInput = releaseHandIndex == 1u ? frame.left : frame.right;
                    const RE::NiPoint3 releaseGripWorld = _equippedWeaponReleaseCapture.hasPreviousHandWorld[releaseHandIndex] ?
                                                             _equippedWeaponReleaseCapture.previousHandWorld[releaseHandIndex].translate :
                                                             releaseHandInput.grabAnchorWorld;
                    // Consume the equipped body's generated points before
                    // the drop transaction retires that bank. They only
                    // bound long-object angular release speed; the frozen
                    // transform is the native body's placement authority.
                    const auto releaseGeometry = hasReleaseRot ?
                                                     _weaponCollision.getCurrentWeaponReleaseGeometry(releaseGripWorld, releaseWeaponWorld) :
                                                     WeaponCollision::ReleaseGeometrySnapshot{};
                    if (!releaseGeometry.hasCapturedWeaponWorld) {
                        ROCK_LOG_WARN(Weapon,
                            "Equipped weapon physical drop blocked because no finite frozen release pose is available: sourceHand={}",
                            equipped_weapon_drop_policy::sourceHandName(sourceHand));
                        f4vr::showNotification("ROCK: Cannot drop weapon - release pose is not ready.");
                    } else {
                        const auto dropResult = weapon_equip_transfer::dropEquippedWeaponFromPlayer(weapon_equip_transfer::EquippedDropInput{
                            .dropLoc = releaseLoc,
                            .dropRot = releaseRot,
                            .hasDropLoc = true,
                            .hasDropRot = true,
                        });
                        const bool dropCommitted = equipped_weapon_drop_policy::physicalDropCommitted(
                            equipped_weapon_drop_policy::PhysicalDropCommitInput{
                                .dropSucceeded = dropResult.success,
                                .droppedReferenceUnavailable =
                                    dropResult.reason == weapon_equip_transfer::DropReason::DroppedReferenceUnavailable,
                            });
                        if (dropCommitted) {
                            enforceNoBareFistState(true);
                            /*
                             * RemoveItem creates the native layer-5 weapon at
                             * the last layer-44 equipped-collider pose. Retire
                             * ROCK's generated representation in this same
                             * transaction so no physics step can solve the two
                             * coincident weapon body sets before the native
                             * handoff takes ownership.
                             */
                            _weaponCollision.destroyWeaponBody(hknp);
                        }
                        if (dropCommitted && dropResult.handle) {
                            armEquippedWeaponDropMomentumHandoff(
                                dropResult.handle,
                                dropResult.droppedFormID,
                                sourceHand,
                                releaseGeometry);
                        }
                        if (dropCommitted) {
                            ROCK_LOG_INFO(Weapon,
                                "Equipped weapon manual release committed formID={:08X} dropped={:08X} reference={} sourceHand={} dropLoc=({:.1f},{:.1f},{:.1f}) lever={:.1f}gu stack={} instanceMatch={}",
                                dropResult.formID,
                                dropResult.droppedFormID,
                                dropResult.success ? "ready" : "pending",
                                equipped_weapon_drop_policy::sourceHandName(sourceHand),
                                releaseLoc.x,
                                releaseLoc.y,
                                releaseLoc.z,
                                releaseGeometry.leverGameUnits,
                                dropResult.stackID,
                                dropResult.matchedInstanceData ? "yes" : "no");
                        } else {
                            ROCK_LOG_WARN(Weapon,
                                "Equipped weapon manual release drop failed formID={:08X} reason={} sourceHand={} attempted={} stack={} instanceMatch={}",
                                dropResult.formID,
                                weapon_equip_transfer::dropReasonName(dropResult.reason),
                                equipped_weapon_drop_policy::sourceHandName(sourceHand),
                                dropResult.attempted ? "yes" : "no",
                                dropResult.stackID,
                                dropResult.matchedInstanceData ? "yes" : "no");
                        }
                        if (sourceHandKnown && dropCommitted) {
                            suppressHandCollisionAfterEquippedWeaponDrop(hknp, sourceHand);
                        }
                    }
                }
                _pendingEquippedWeaponPrimaryOnlyGripStart = {};
                clearEquippedWeaponFiringGripInputState();
            }
        }
        updateEquippedWeaponReleaseCapture(frame, weaponNode);
        const bool weaponSupportGripActive = _twoHandedGrip.isHandPartGripping(true);
        const input_remap_policy::EquippedWeaponFiringGripInputGate updatedFiringGripInputGate{
            .featureAvailable = firingGripOwnershipFeatureAvailable,
            .canUseFiringGripInput = _twoHandedGrip.canUseFiringGripInput(),
            .menuInputActive = inputBlockingMenuActive,
        };
        input_remap_runtime::setEquippedWeaponFiringGripInputActive(
            input_remap_policy::shouldUseEquippedWeaponFiringGripInput(updatedFiringGripInputGate));
        input_remap_runtime::setEquippedWeaponPrimaryDetached(_twoHandedGrip.isPartCarryActive());
        /*
         * Left-hand fire publication: while the LEFT hand occupies the
         * firing grip, the OpenVR-level trigger remap presents the left
         * trigger to the game as the primary (right) wand's trigger.
         */
        const bool leftHandFiringActiveAfterGrip = _twoHandedGrip.isFiringHandLeft() && _twoHandedGrip.isFiringGripOccupied();
        input_remap_runtime::setEquippedWeaponLeftHandFiringActive(leftHandFiringActiveAfterGrip);
        ::rock::provider::setEquippedWeaponFiringHandIsLeft(_twoHandedGrip.isFiringHandLeft());

        bool rightHandWeaponAuthorityActiveAfterGrip = rightHandWeaponEquipped || retainedWeaponCollisionActive;
        // A visible part-carry frees the right hand even while weapon bodies exist (see the pre-grip gate).
        if (rightHandWeaponEquipped && _twoHandedGrip.isPartCarryActive()) {
            rightHandWeaponAuthorityActiveAfterGrip = false;
        }
        // Left-firing carry frees the right hand the same way (see the pre-grip gate).
        if (rightHandWeaponEquipped && leftHandFiringActiveAfterGrip) {
            rightHandWeaponAuthorityActiveAfterGrip = false;
        }
        if (rightHandWeaponAuthorityActiveAfterGrip != rightHandWeaponAuthorityActiveBeforeGrip) {
            if (rightHandWeaponAuthorityActiveAfterGrip) {
                suppressRightHandCollisionForDominantWeapon(hknp);
            } else {
                restoreRightHandCollisionAfterDominantWeapon(hknp);
            }
        }
        rightHandWeaponAuthorityActive = rightHandWeaponAuthorityActiveAfterGrip;
        const bool rightPartGripActiveAfterGrip = _twoHandedGrip.isHandPartGripping(false);
        if (rightPartGripActiveAfterGrip != rightPartGripActive) {
            if (rightPartGripActiveAfterGrip) {
                suppressHandCollisionForWeaponSupport(hknp, false);
            } else {
                restoreHandCollisionAfterWeaponSupport(hknp, false);
            }
        }
        rightPartGripActive = rightPartGripActiveAfterGrip;

        if (g_rockConfig.rockDebugShowWeaponNotifications) {
            const auto gripNotificationEvent =
                weapon_debug_notification_policy::observeWeaponSupportGrip(_weaponDebugNotificationState, weaponSupportGripActive);
            if (gripNotificationEvent != weapon_debug_notification_policy::WeaponGripNotificationEvent::None) {
                if (gripNotificationEvent == weapon_debug_notification_policy::WeaponGripNotificationEvent::Started) {
                    const auto weaponDebugInfo = makeWeaponInteractionDebugInfo(_weaponCollision, weaponNode, leftWeaponContact);
                    f4vr::showNotification(
                        weapon_debug_notification_policy::formatWeaponGripNotification(gripNotificationEvent, weaponNotificationKey, weaponDebugInfo));
                    ROCK_LOG_INFO(Weapon,
                        "WeaponGripDiagnostics: weapon='{}' formID={:08X} node='{}' driveRoot='{}' sourceRoot='{}' nif='{}' part={} route={} pose={} body={} source={}",
                        weapon_debug_notification_policy::debugTextOrUnknown(weaponDebugInfo.weaponName),
                        weaponDebugInfo.weaponFormId,
                        weapon_debug_notification_policy::debugTextOrUnknown(weaponDebugInfo.weaponNodeName),
                        weapon_debug_notification_policy::debugTextOrUnknown(weaponDebugInfo.interactionRootName),
                        weapon_debug_notification_policy::debugTextOrUnknown(weaponDebugInfo.sourceRootName),
                        weapon_debug_notification_policy::debugTextOrUnknown(weaponDebugInfo.sourceName),
                        weapon_debug_notification_policy::nameOf(weaponNotificationKey.partKind),
                        weapon_debug_notification_policy::nameOf(weaponNotificationKey.interactionKind),
                        weapon_debug_notification_policy::nameOf(weaponNotificationKey.gripPose),
                        weaponNotificationKey.bodyId,
                        weapon_debug_notification_policy::nameOf(weaponNotificationKey.source));
                } else {
                    f4vr::showNotification(weapon_debug_notification_policy::formatWeaponGripNotification(gripNotificationEvent, weaponNotificationKey));
                }
            }
        } else {
            _weaponDebugNotificationState.supportGripActive = weaponSupportGripActive;
        }
        leftSupportGripActive = weaponSupportGripActive;

        /*
         * A LEFT hand occupying the firing grip is weapon-engaged exactly
         * like a support hand from the collision standpoint: its generated
         * colliders must not become a second physical owner while the
         * weapon rides the hand. Reuses the per-hand support lease.
         */
        if (weaponSupportGripActive || leftHandFiringActiveAfterGrip) {
            suppressHandCollisionForWeaponSupport(hknp, true);
        } else {
            restoreHandCollisionAfterWeaponSupport(hknp, true);
        }
    }


    void PhysicsInteraction::finishDynamicWeaponFrame(
        const PhysicsFrameContext& frame,
        EquippedWeaponFrame& weaponFrame)
    {
        auto* hknp = frame.hknpWorld;
        auto* weaponNode = weaponFrame.weaponNode;
        const auto currentWeaponGenerationKey =
            weaponFrame.generationKey;
        const auto& drivenSourceNodes =
            weaponFrame.drivenSourceNodes;
        const auto drivenSourceNodeCount =
            weaponFrame.drivenSourceNodeCount;
        auto* gunstockProjectileNode =
            weaponFrame.gunstockProjectileNode;

        const auto dynamicWeaponFrame =
            _dynamicWeaponCollision.finishFrame(
                frame,
                physicsWritesAllowedForWorld(frame.hknpWorld),
                weaponNode,
                currentWeaponGenerationKey,
                _weaponCollision);
        if (dynamicWeaponFrame.contactEpisodeStarted &&
            g_rockConfig.rockDebugDynamicWeaponLogging) {
            auto* otherRef = resolveBodyToRef(
                frame.bhkWorld,
                frame.hknpWorld,
                RE::hknpBodyId{ dynamicWeaponFrame.otherBodyId });
            const auto* otherBase = otherRef ? otherRef->GetObjectReference() : nullptr;
            const auto otherNameView = otherBase ?
                RE::TESFullName::GetFullName(*otherBase, false) :
                std::string_view{};
            const std::string otherName = otherNameView.empty() ?
                std::string("(unresolved)") :
                std::string(otherNameView);
            const char* otherType = otherBase ?
                otherBase->GetFormTypeString() :
                "unresolved";

            WeaponCollision::WeaponSurfaceProximityWitness partWitness{};
            WeaponInteractionDebugInfo partInfo{};
            constexpr float kContactPartSearchRadiusGameUnits = 24.0f;
            const bool partWitnessValid =
                dynamicWeaponFrame.rawContactPointValid &&
                _weaponCollision.tryFindCurrentWeaponSurfaceNearPoint(
                    weaponNode,
                    dynamicWeaponFrame.rawContactPointGame,
                    kContactPartSearchRadiusGameUnits,
                    partWitness);
            const bool partInfoValid =
                partWitnessValid &&
                _weaponCollision.tryGetWeaponContactDebugInfo(
                    partWitness.bodyId,
                    partInfo);

            ROCK_LOG_INFO(
                Weapon,
                "DWC contact witness: episode={} solveAge={} generation={:016X} weaponForm={:08X} other(body/layer/motion/ref/form/type/name)=({}/{}/{}/{:p}/{:08X}/{}/{}) native(collObj/owner)=({:p}/{:p}) raw(valid/proxyWasA/points/index/weight)={}/{}/{}/{}/{:.3f} pointGame=({:.2f},{:.2f},{:.2f}) normalRaw=({:.3f},{:.3f},{:.3f}) nearestPart(valid/body/source/distance/current)={}/{}/{}/{:.3f}/{}",
                dynamicWeaponFrame.contactEpisode,
                dynamicWeaponFrame.contactSolveAge,
                currentWeaponGenerationKey,
                _weaponCollision.getCurrentObservedEquippedWeaponFormID(),
                dynamicWeaponFrame.otherBodyId,
                dynamicWeaponFrame.otherLayer,
                dynamicWeaponFrame.otherMotionIndex,
                static_cast<void*>(otherRef),
                otherRef ? otherRef->GetFormID() : 0,
                otherType,
                otherName,
                reinterpret_cast<void*>(dynamicWeaponFrame.otherCollisionObject),
                reinterpret_cast<void*>(dynamicWeaponFrame.otherOwnerNode),
                dynamicWeaponFrame.rawContactPointValid,
                dynamicWeaponFrame.rawContactProxyWasBodyA,
                dynamicWeaponFrame.rawContactPointCount,
                dynamicWeaponFrame.rawContactPointIndex,
                dynamicWeaponFrame.rawContactPointWeightSum,
                dynamicWeaponFrame.rawContactPointGame.x,
                dynamicWeaponFrame.rawContactPointGame.y,
                dynamicWeaponFrame.rawContactPointGame.z,
                dynamicWeaponFrame.rawContactNormalHavok.x,
                dynamicWeaponFrame.rawContactNormalHavok.y,
                dynamicWeaponFrame.rawContactNormalHavok.z,
                partWitnessValid,
                partWitnessValid ? partWitness.bodyId : 0x7FFF'FFFFu,
                partInfoValid ? partInfo.sourceName : std::string("(unresolved)"),
                partWitnessValid ? partWitness.distanceGameUnits : -1.0f,
                partWitnessValid && partWitness.sourceNodeCurrent);
            ROCK_LOG_INFO(
                Weapon,
                "DWC contact transforms: episode={} requestedBody=({:.2f},{:.2f},{:.2f}) liveBody=({:.2f},{:.2f},{:.2f}) otherReadable={} otherBody=({:.2f},{:.2f},{:.2f}) correction=({:.2f}gu,{:.2f}deg)",
                dynamicWeaponFrame.contactEpisode,
                dynamicWeaponFrame.requestedContactBodyWorld.translate.x,
                dynamicWeaponFrame.requestedContactBodyWorld.translate.y,
                dynamicWeaponFrame.requestedContactBodyWorld.translate.z,
                dynamicWeaponFrame.liveContactBodyWorld.translate.x,
                dynamicWeaponFrame.liveContactBodyWorld.translate.y,
                dynamicWeaponFrame.liveContactBodyWorld.translate.z,
                dynamicWeaponFrame.otherBodyWorldValid,
                dynamicWeaponFrame.otherBodyWorld.translate.x,
                dynamicWeaponFrame.otherBodyWorld.translate.y,
                dynamicWeaponFrame.otherBodyWorld.translate.z,
                dynamicWeaponFrame.translationCorrectionGameUnits,
                dynamicWeaponFrame.rotationCorrectionDegrees);
        }
        if (dynamicWeaponFrame.publishVisualAuthority) {
            const bool visualPublishSucceeded = _twoHandedGrip.applyWeaponCollisionResolvedAuthority(
                weaponNode,
                dynamicWeaponFrame.requestedWeaponWorld,
                dynamicWeaponFrame.resolvedWeaponWorld,
                currentWeaponGenerationKey);
            const float immediateTranslationError =
                visualPublishSucceeded && weaponNode ?
                    dynamic_weapon_collision_policy::translationDeltaGameUnits(
                        weaponNode->world,
                        dynamicWeaponFrame.resolvedWeaponWorld) :
                    -1.0f;
            const float immediateRotationError =
                visualPublishSucceeded && weaponNode ?
                    dynamic_weapon_collision_policy::rotationDeltaDegrees(
                        weaponNode->world,
                        dynamicWeaponFrame.resolvedWeaponWorld) :
                    -1.0f;
            if (g_rockConfig.rockDebugDynamicWeaponLogging) {
                ROCK_LOG_SAMPLE_INFO(
                    Weapon,
                    500,
                    "DWC visual publication: bodyActive={} publishSucceeded={} immediateError=({:.3f}gu,{:.3f}deg) requestedCorrection=({:.3f}gu,{:.3f}deg)",
                    dynamicWeaponFrame.proxyActive,
                    visualPublishSucceeded,
                    immediateTranslationError,
                    immediateRotationError,
                    dynamicWeaponFrame.translationCorrectionGameUnits,
                    dynamicWeaponFrame.rotationCorrectionDegrees);
            }
        }
        _twoHandedGrip.finishWeaponCollisionPresentationFrame(
            dynamicWeaponFrame.publishVisualAuthority);
        (void)_twoHandedGrip.applyFiringWeaponRecoilPresentation(
            weaponNode,
            currentWeaponGenerationKey);
        if (weaponNode) {
            performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::WeaponCollisionTransforms);
            _weaponCollision.updateBodiesFromCurrentSourceTransforms(
                hknp,
                weaponNode,
                frame.deltaSeconds,
                drivenSourceNodes.data(),
                drivenSourceNodeCount);
        }
        if (f4vr::isNodeVisible(weaponNode)) {
            applyFinalWeaponMuzzleAuthority();
        }
        _twoHandedGrip.finalizeGunstockAlignmentDebugSnapshot(
            weaponNode,
            gunstockProjectileNode,
            currentWeaponGenerationKey);
    }
}
