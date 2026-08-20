#define ROCK_API_EXPORTS
// Build the DLL side of the public provider ABI.
#include "api/detail/ProviderAnimationAuthority.h"
#include "api/detail/ProviderApiCore.h"
#include "api/detail/ProviderApiState.h"
#include "api/detail/ProviderAuthorityLeases.h"
#include "api/detail/ProviderFrameDiff.h"
#include "api/detail/ProviderWeaponParts.h"

#include "api/ProviderColliderVisualizationRuntime.h"
#include "api/ProviderDebugOverlayRuntime.h"

#include <algorithm>
#include <array>
#include <cstring>
#include <mutex>

#include "monitor/GrabClockMonitor.h"
#include "physics-interaction/core/PhysicsInteraction.h"

namespace rock::provider::detail
{
    std::uint64_t currentProviderFrameIndex()
    {
        const auto nextFrameIndex = s_nextFrameIndex.load(std::memory_order_acquire);
        return nextFrameIndex > 0 ? nextFrameIndex - 1 : 0;
    }

    void publishProviderEvent(RockProviderEventV1 event)
    {
        event.size = sizeof(RockProviderEventV1);
        event.version = ROCK_PROVIDER_API_VERSION;
        if (event.frameIndex == 0) {
            event.frameIndex = currentProviderFrameIndex();
        }
        if (s_generationStateAvailable.load(std::memory_order_acquire)) {
            if (event.worldGeneration == 0) {
                event.worldGeneration = s_currentWorldGeneration.load(
                    std::memory_order_acquire);
            }
            if (event.skeletonGeneration == 0) {
                event.skeletonGeneration = s_currentSkeletonGeneration.load(
                    std::memory_order_acquire);
            }
            if (event.providerGeneration == 0) {
                event.providerGeneration = s_currentProviderGeneration.load(
                    std::memory_order_acquire);
            }
        }

        std::scoped_lock lock(s_providerEventMutex);
        event.sequence = s_nextProviderEventSequence++;
        if (s_providerEventCount < s_providerEvents.size()) {
            const auto index =
                (s_providerEventHead + s_providerEventCount) %
                s_providerEvents.size();
            s_providerEvents[index] = event;
            ++s_providerEventCount;
        } else {
            s_providerEvents[s_providerEventHead] = event;
            s_providerEventHead =
                (s_providerEventHead + 1) % s_providerEvents.size();
            ++s_overwrittenProviderEventCount;
        }
    }

    void publishAuthorityLostEvent(
        const std::uint64_t ownerToken,
        const RockProviderAuthorityKindV1 authorityKind,
        const std::uint32_t reason)
    {
        RockProviderEventV1 event{};
        event.kind = RockProviderEventKindV1::AuthorityLost;
        event.ownerToken = ownerToken;
        event.result = reason;
        event.data[0] = static_cast<std::uint32_t>(authorityKind);
        publishProviderEvent(event);
    }


    struct FrameDispatchChanges
    {
        bool hadPrevious{ false };
        bool lifecycleChanged{ false };
        bool equippedTerminalChanged{ false };
        std::array<bool, 2> handChanged{};
    };

    [[nodiscard]] RockProviderFrameSnapshot fillFrameSnapshot(
        rock::PhysicsInteraction& pi)
    {
        RockProviderFrameSnapshot snapshot{};
        snapshot.frameIndex = s_nextFrameIndex.fetch_add(1, std::memory_order_acq_rel);
        pi.fillProviderFrameSnapshot(snapshot);
        snapshot.externalBodyCount = currentExternalBodyCount();
        return snapshot;
    }

    void publishFrameGenerations(
        const RockProviderFrameSnapshot& snapshot)
    {
        // These atomics are the allocation-free generation authority used by
        // lease validation during this frame. Publishing them before pruning
        // makes a generation transition revoke stale state in the same frame.
        s_currentWorldGeneration.store(
            snapshot.worldGeneration,
            std::memory_order_release);
        s_currentSkeletonGeneration.store(
            snapshot.skeletonGeneration,
            std::memory_order_release);
        s_currentProviderGeneration.store(
            snapshot.providerGeneration,
            std::memory_order_release);
        s_generationStateAvailable.store(true, std::memory_order_release);
    }

    void pruneFrameLeases(
        rock::PhysicsInteraction& pi,
        RockProviderFrameSnapshot& snapshot)
    {
        {
            std::scoped_lock lock(s_handInputSuppressionMutex);
            pruneExpiredHandInputSuppressionsLocked(snapshot.frameIndex);
        }
        {
            std::scoped_lock lock(s_nativeAnimationAuthorityMutex);
            pruneExpiredNativeAnimationAuthorityLocked(snapshot.frameIndex);
        }
        {
            std::scoped_lock lock(s_nativeAnimationRuntimePublicationMutex);
            pruneExpiredNativeAnimationRuntimePublicationLocked(
                snapshot.frameIndex);
        }
        {
            std::scoped_lock lock(s_equippedWeaponHandlingAuthorityMutex);
            pruneExpiredEquippedWeaponHandlingAuthorityLocked(
                snapshot.frameIndex);
        }
        {
            std::scoped_lock lock(s_weaponPartMutex);
            pruneExpiredWeaponPartDrivesLocked(snapshot.frameIndex);
        }
        {
            std::scoped_lock lock(s_offhandReservationMutex);
            pruneExpiredOffhandReservationLocked(snapshot.frameIndex);
        }
        snapshot.offhandReservation = currentOffhandReservation();

        provider_debug_overlay::PruneResult overlayPrune{};
        provider_debug_overlay::prune(
            snapshot.frameIndex,
            snapshot.worldGeneration,
            snapshot.skeletonGeneration,
            snapshot.providerGeneration,
            overlayPrune);
        for (std::uint32_t index = 0; index < overlayPrune.count; ++index) {
            publishAuthorityLostEvent(
                overlayPrune.publishers[index].ownerToken,
                RockProviderAuthorityKindV1::DebugOverlay,
                static_cast<std::uint32_t>(
                    overlayPrune.publishers[index].reason));
        }

        provider_collider_visualization::Invalidation
            colliderVisualizationInvalidation{};
        provider_collider_visualization::Snapshot
            colliderVisualizationSnapshot{};
        const bool colliderVisualizationActive =
            provider_collider_visualization::copySnapshot(
                colliderVisualizationSnapshot);
        const bool colliderVisualizationBodyCurrent =
            !colliderVisualizationActive ||
            pi.isProviderWeaponBodyCurrentV1(
                colliderVisualizationSnapshot.weaponGenerationKey,
                colliderVisualizationSnapshot.bodyId);
        provider_collider_visualization::prune(
            snapshot.frameIndex,
            snapshot.worldGeneration,
            snapshot.skeletonGeneration,
            snapshot.providerGeneration,
            snapshot.weaponGenerationKey,
            colliderVisualizationBodyCurrent,
            colliderVisualizationInvalidation);
        if (colliderVisualizationInvalidation.ownerToken != 0) {
            publishAuthorityLostEvent(
                colliderVisualizationInvalidation.ownerToken,
                RockProviderAuthorityKindV1::ColliderVisualization,
                static_cast<std::uint32_t>(
                    colliderVisualizationInvalidation.reason));
        }
    }

    void fillFrameObservability(
        rock::PhysicsInteraction& pi,
        const RockProviderFrameSnapshot& snapshot,
        std::array<RockProviderWeaponPartGripStateV1, 2>& partGripStates,
        std::array<RockProviderHandInteractionStateV1, 2>& handInteractionStates,
        RockProviderEquippedWeaponStateV1& equippedWeaponState)
    {
        partGripStates = {};
        pi.fillProviderWeaponPartGripStates(partGripStates);

        handInteractionStates = {};
        pi.fillProviderHandInteractionStates(handInteractionStates);
        for (auto& state : handInteractionStates) {
            state.frameIndex = snapshot.frameIndex;
        }

        equippedWeaponState = {};
        (void)pi.queryProviderEquippedWeaponStateV1(equippedWeaponState);
        equippedWeaponState.frameIndex = snapshot.frameIndex;
    }

    [[nodiscard]] FrameDispatchChanges updatePublishedFrameState(
        RockProviderFrameSnapshot& snapshot,
        const std::array<RockProviderWeaponPartGripStateV1, 2>& partGripStates,
        std::array<RockProviderHandInteractionStateV1, 2>& handInteractionStates,
        RockProviderEquippedWeaponStateV1& equippedWeaponState)
    {
        bool hadPrevious = false;
        bool lifecycleChanged = false;
        bool equippedTerminalChanged = false;
        std::array<bool, 2> handChanged{};

        {
            std::scoped_lock lock(s_snapshotMutex);
            hadPrevious = s_hasSnapshot;
            const auto previousSnapshot = s_lastSnapshot;
            const auto previousHands = s_lastHandInteractionStates;
            const auto previousEquipped = s_lastEquippedWeaponState;

            for (std::size_t index = 0;
                 index < handInteractionStates.size();
                 ++index) {
                auto& currentHand = handInteractionStates[index];
                const auto& previousHand = previousHands[index];
                if (hadPrevious &&
                    handGripActive(previousHand) &&
                    !handGripActive(currentHand) &&
                    currentHand.phase ==
                        RockProviderHandInteractionPhaseV1::Idle) {
                    currentHand.phase =
                        RockProviderHandInteractionPhaseV1::Releasing;
                    currentHand.targetKind = previousHand.targetKind;
                    currentHand.reservedTargetIdentity =
                        previousHand.reservedTargetIdentity;
                    currentHand.targetFormId = previousHand.targetFormId;
                    currentHand.primaryBodyId = previousHand.primaryBodyId;
                    constexpr std::uint32_t touchGrabClassificationFlags =
                        static_cast<std::uint32_t>(
                            RockProviderHandInteractionFlagV1::TouchGrab) |
                        static_cast<std::uint32_t>(
                            RockProviderHandInteractionFlagV1::
                                FixedSurfaceLatch) |
                        static_cast<std::uint32_t>(
                            RockProviderHandInteractionFlagV1::
                                GlobalSurfaceLatch) |
                        static_cast<std::uint32_t>(
                            RockProviderHandInteractionFlagV1::
                                SurfaceAnchorValid) |
                        static_cast<std::uint32_t>(
                            RockProviderHandInteractionFlagV1::
                                MeshSurfaceAnchor) |
                        static_cast<std::uint32_t>(
                            RockProviderHandInteractionFlagV1::
                                MeshFingerPose) |
                        static_cast<std::uint32_t>(
                            RockProviderHandInteractionFlagV1::
                                MeshCollisionFallback);
                    currentHand.flags |=
                        previousHand.flags &
                        touchGrabClassificationFlags;
                    currentHand.surfaceAnchorGame =
                        previousHand.surfaceAnchorGame;
                    currentHand.surfaceGripMode =
                        previousHand.surfaceGripMode;
                    currentHand.heldBodyCount = previousHand.heldBodyCount;
                    std::copy(
                        std::begin(previousHand.heldBodyIds),
                        std::end(previousHand.heldBodyIds),
                        std::begin(currentHand.heldBodyIds));
                }
                assignHandInteractionSequences(
                    currentHand,
                    previousHand,
                    hadPrevious);
                handChanged[index] = !hadPrevious ||
                    !sameHandInteractionPayload(
                        currentHand,
                        previousHand);
            }

            lifecycleChanged = !hadPrevious ||
                !sameLifecyclePayload(snapshot, previousSnapshot);
            const bool weaponChanged = !hadPrevious ||
                !sameWeaponPayload(snapshot, previousSnapshot) ||
                !sameEquippedWeaponPayload(
                    equippedWeaponState,
                    previousEquipped);
            const bool transitionChanged = !hadPrevious ||
                equippedWeaponState.transitionSequence !=
                    previousEquipped.transitionSequence ||
                equippedWeaponState.terminalSequence !=
                    previousEquipped.terminalSequence;
            const bool collisionChanged = !hadPrevious ||
                snapshot.collisionGeneration !=
                    previousSnapshot.collisionGeneration;
            const bool handRolesChanged = !hadPrevious ||
                snapshot.primaryHand != previousSnapshot.primaryHand ||
                snapshot.offhandHand != previousSnapshot.offhandHand ||
                snapshot.offhandReservation !=
                    previousSnapshot.offhandReservation;

            if (lifecycleChanged) {
                snapshot.stateChangeMask |= static_cast<std::uint32_t>(
                    RockProviderFrameStateChangeFlagV1::Lifecycle);
            }
            if (handChanged[0]) {
                snapshot.stateChangeMask |= static_cast<std::uint32_t>(
                    RockProviderFrameStateChangeFlagV1::RightHand);
            }
            if (handChanged[1]) {
                snapshot.stateChangeMask |= static_cast<std::uint32_t>(
                    RockProviderFrameStateChangeFlagV1::LeftHand);
            }
            if (weaponChanged) {
                snapshot.stateChangeMask |= static_cast<std::uint32_t>(
                    RockProviderFrameStateChangeFlagV1::Weapon);
            }
            if (transitionChanged) {
                snapshot.stateChangeMask |= static_cast<std::uint32_t>(
                    RockProviderFrameStateChangeFlagV1::EquippedTransition);
            }
            if (collisionChanged) {
                snapshot.stateChangeMask |= static_cast<std::uint32_t>(
                    RockProviderFrameStateChangeFlagV1::Collision);
            }
            if (handRolesChanged) {
                snapshot.stateChangeMask |= static_cast<std::uint32_t>(
                    RockProviderFrameStateChangeFlagV1::HandRoles);
            }
            snapshot.stateSequence = !hadPrevious ?
                1 :
                (snapshot.stateChangeMask != 0 ?
                        advanceSequence(previousSnapshot.stateSequence) :
                        previousSnapshot.stateSequence);
            snapshot.enrichmentFlags |= static_cast<std::uint32_t>(
                RockProviderFrameEnrichmentFlagV1::StateSequenceValid);
            snapshot.equippedWeaponTransitionSequence =
                equippedWeaponState.transitionSequence;

            equippedTerminalChanged = hadPrevious &&
                equippedWeaponState.terminalSequence != 0 &&
                equippedWeaponState.terminalSequence !=
                    previousEquipped.terminalSequence;
            s_lastSnapshot = snapshot;
            s_hasSnapshot = true;
            s_lastPartGripStates = partGripStates;
            s_lastHandInteractionStates = handInteractionStates;
            s_lastEquippedWeaponState = equippedWeaponState;
        }

        return FrameDispatchChanges{
            .hadPrevious = hadPrevious,
            .lifecycleChanged = lifecycleChanged,
            .equippedTerminalChanged = equippedTerminalChanged,
            .handChanged = handChanged,
        };
    }

    void publishFrameChangeEvents(
        const RockProviderFrameSnapshot& snapshot,
        const std::array<RockProviderHandInteractionStateV1, 2>& handInteractionStates,
        const RockProviderEquippedWeaponStateV1& equippedWeaponState,
        const FrameDispatchChanges& changes)
    {
        const bool hadPrevious = changes.hadPrevious;
        const bool lifecycleChanged = changes.lifecycleChanged;
        const bool equippedTerminalChanged =
            changes.equippedTerminalChanged;
        const auto& handChanged = changes.handChanged;

        if (hadPrevious && lifecycleChanged) {
            RockProviderEventV1 event{};
            event.kind = RockProviderEventKindV1::LifecycleChanged;
            event.result = static_cast<std::uint32_t>(
                snapshot.lastLifecycleReason);
            event.subjectSequence = snapshot.stateSequence;
            event.data[0] = snapshot.lifecycleFlags;
            event.data[1] = snapshot.providerReady;
            event.data[2] = snapshot.stateChangeMask;
            publishProviderEvent(event);
        }
        if (hadPrevious) {
            for (std::size_t index = 0;
                 index < handInteractionStates.size();
                 ++index) {
                if (!handChanged[index]) {
                    continue;
                }
                const auto& state = handInteractionStates[index];
                RockProviderEventV1 event{};
                event.kind = RockProviderEventKindV1::GrabStateChanged;
                event.hand = state.hand;
                event.formId = state.targetFormId;
                event.result = static_cast<std::uint32_t>(state.phase);
                event.subjectSequence = state.stateSequence;
                event.data[0] = static_cast<std::uint32_t>(
                    state.targetKind);
                event.data[1] = state.primaryBodyId;
                event.data[2] = state.flags;
                publishProviderEvent(event);
            }
        }
        if (equippedTerminalChanged) {
            RockProviderEventV1 event{};
            event.kind =
                RockProviderEventKindV1::EquippedWeaponTransitionTerminal;
            event.weaponGenerationKey =
                equippedWeaponState.weaponGenerationKey;
            event.formId = equippedWeaponState.weaponFormId;
            event.result = static_cast<std::uint32_t>(
                equippedWeaponState.terminalResult);
            event.subjectSequence =
                equippedWeaponState.terminalSequence;
            event.data[0] = static_cast<std::uint32_t>(
                equippedWeaponState.transitionSource);
            event.data[1] = equippedWeaponState.flags;
            publishProviderEvent(event);
        }
    }

    void dispatchRegisteredFrameCallbacks(
        const RockProviderFrameSnapshot& snapshot)
    {
        for (std::size_t index = 0; index < s_callbacks.size(); ++index) {
            CallbackSlot slot{};
            {
                std::scoped_lock lock(s_callbackMutex);
                slot = s_callbacks[index];
            }
            if (slot.callback) {
                FrameCallbackInvocationResult callbackResult{};
                try {
                    callbackResult = invokeFrameCallbackSafely(
                        slot.callback,
                        &snapshot,
                        slot.userData);
                } catch (...) {
                    callbackResult.healthy = false;
                }

                if (!callbackResult.healthy) {
                    HMODULE faultModule = nullptr;
                    constexpr auto moduleFlags =
                        GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS |
                        GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT;
                    const auto faultAddress =
                        callbackResult.exceptionAddress;
                    const bool hasFaultModule =
                        faultAddress != 0 &&
                        GetModuleHandleExA(
                            moduleFlags,
                            reinterpret_cast<LPCSTR>(faultAddress),
                            &faultModule) != FALSE;
                    if (hasFaultModule) {
                        std::array<char, MAX_PATH> modulePath{};
                        const auto pathLength = GetModuleFileNameA(
                            faultModule,
                            modulePath.data(),
                            static_cast<DWORD>(modulePath.size()));
                        const char* moduleName = modulePath.data();
                        if (pathLength != 0) {
                            if (const auto* slash =
                                    std::strrchr(moduleName, '\\')) {
                                moduleName = slash + 1;
                            }
                        } else {
                            moduleName = "<unknown-module>";
                        }
                        const auto moduleBase =
                            reinterpret_cast<std::uintptr_t>(
                                faultModule);
                        logger::error(
                            "ROCK provider frame callback token {} faulted: "
                            "exception=0x{:08X} instruction={}+0x{:X} "
                            "(0x{:016X}); unregistering the callback.",
                            slot.token,
                            callbackResult.exceptionCode,
                            moduleName,
                            faultAddress - moduleBase,
                            faultAddress);
                    } else {
                        logger::error(
                            "ROCK provider frame callback token {} faulted: "
                            "exception=0x{:08X} instruction=0x{:016X}; "
                            "unregistering the callback.",
                            slot.token,
                            callbackResult.exceptionCode,
                            faultAddress);
                    }
                    if (slot.ownerToken != 0) {
                        clearOwnerStateAfterCallbackFault(slot.ownerToken);
                    } else {
                        clearCallbackSlot(slot.token);
                    }
                }
            }
        }
    }

    void handOffFrameToMonitor(
        const RockProviderFrameSnapshot& snapshot)
    {
        // Embedded grab-clock monitor panel consumes the same fully built
        // snapshot as external consumers, but in-process with direct access
        // to ROCK internals (game thread, every producer frame including
        // menu/config-blocked ones so the panel can hide itself).
        rock::monitor::onProviderFrame(snapshot);
    }
}
namespace rock::provider
{
    using namespace detail;

    void setPhysicsInteractionInstance(rock::PhysicsInteraction* pi)
    {
        s_physicsInteraction.store(pi, std::memory_order_release);
    }

    void dispatchFrameCallbacks(rock::PhysicsInteraction& pi)
    {
        auto snapshot = fillFrameSnapshot(pi);
        publishFrameGenerations(snapshot);
        pruneFrameLeases(pi, snapshot);

        std::array<RockProviderWeaponPartGripStateV1, 2> partGripStates{};
        std::array<RockProviderHandInteractionStateV1, 2>
            handInteractionStates{};
        RockProviderEquippedWeaponStateV1 equippedWeaponState{};
        fillFrameObservability(
            pi,
            snapshot,
            partGripStates,
            handInteractionStates,
            equippedWeaponState);

        const auto changes = updatePublishedFrameState(
            snapshot,
            partGripStates,
            handInteractionStates,
            equippedWeaponState);
        publishFrameChangeEvents(
            snapshot,
            handInteractionStates,
            equippedWeaponState,
            changes);
        dispatchRegisteredFrameCallbacks(snapshot);
        handOffFrameToMonitor(snapshot);
    }
}
