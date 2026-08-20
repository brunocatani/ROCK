#define ROCK_API_EXPORTS
// Build the DLL side of the public provider ABI.
#include "api/detail/ProviderApiState.h"

namespace rock::provider::detail
{
    // The callback lock guards registration and game-thread dispatch copies.
    std::atomic<rock::PhysicsInteraction*> s_physicsInteraction{ nullptr };
    std::atomic<std::uint64_t> s_nextFrameIndex{ 1 };
    std::atomic<std::uint64_t> s_nextCallbackToken{ 1 };
    std::mutex s_callbackMutex;
    std::array<CallbackSlot, 16> s_callbacks{};

    // The snapshot lock guards game-thread publication and consumer reads.
    std::mutex s_snapshotMutex;
    RockProviderFrameSnapshot s_lastSnapshot{};
    bool s_hasSnapshot{ false };
    std::atomic<bool> s_generationStateAvailable{ false };
    std::atomic<std::uint32_t> s_currentWorldGeneration{ 0 };
    std::atomic<std::uint32_t> s_currentSkeletonGeneration{ 0 };
    std::atomic<std::uint32_t> s_currentProviderGeneration{ 0 };
    std::array<RockProviderWeaponPartGripStateV1, 2>
        s_lastPartGripStates{};
    std::array<RockProviderHandInteractionStateV1, 2>
        s_lastHandInteractionStates{};
    RockProviderEquippedWeaponStateV1 s_lastEquippedWeaponState{};

    // The consumer lock guards owner registration and capability checks.
    std::mutex s_consumerMutex;
    std::array<ConsumerSlot, ROCK_PROVIDER_MAX_CONSUMERS_V1> s_consumers{};
    std::atomic<std::uint64_t> s_nextConsumerTokenSequence{ 1 };

    // The command lock joins game-thread requests to physics-thread results.
    std::mutex s_interactionCommandMutex;
    std::array<InteractionCommandSlot,
        ROCK_PROVIDER_MAX_INTERACTION_COMMANDS_V1>
        s_interactionCommands{};
    std::array<InteractionCommandResultSlot,
        ROCK_PROVIDER_MAX_COMPLETED_INTERACTION_COMMANDS_V1>
        s_interactionResults{};
    static_assert(
        ROCK_PROVIDER_MAX_COMPLETED_INTERACTION_COMMANDS_V1 >=
            ROCK_PROVIDER_MAX_INTERACTION_COMMANDS_V1 + 3,
        "Result history must retain live and deferred commands.");
    std::size_t s_nextInteractionResultSlot{ 0 };
    std::atomic<std::uint64_t> s_nextInteractionCommandId{ 1 };
    constinit interaction_command_policy::ForceGrabReservations
        s_forceGrabReservations{};

    // The event lock guards the bounded provider event ring.
    std::mutex s_providerEventMutex;
    std::array<RockProviderEventV1, ROCK_PROVIDER_MAX_PROVIDER_EVENTS_V1>
        s_providerEvents{};
    std::uint32_t s_providerEventCount{ 0 };
    std::uint32_t s_providerEventHead{ 0 };
    std::uint64_t s_nextProviderEventSequence{ 1 };
    std::uint64_t s_overwrittenProviderEventCount{ 0 };

    // The suppression lock guards owner leases read by the input path.
    std::mutex s_handInputSuppressionMutex;
    std::array<HandInputSuppressionSlot,
        ROCK_PROVIDER_MAX_HAND_INPUT_SUPPRESSIONS_V1>
        s_handInputSuppressions{};

    // The native authority lock guards selective animation leases.
    std::mutex s_nativeAnimationAuthorityMutex;
    std::array<NativeAnimationAuthoritySlot,
        ROCK_PROVIDER_MAX_CONSUMERS_V1>
        s_nativeAnimationAuthoritySlots{};
    std::atomic<std::uint32_t> s_nativeAnimationAuthorityFlags{ 0 };
    std::atomic<std::uint32_t> s_nativeAnimationAuthorityOwnerCount{ 0 };

    // The phase lock guards callbacks invoked on the animation owner thread.
    std::mutex s_animationPhaseCallbackMutex;
    std::array<AnimationPhaseCallbackSlot,
        ROCK_PROVIDER_MAX_ANIMATION_PHASE_CALLBACKS_V1>
        s_animationPhaseCallbacks{};
    std::atomic<std::uint64_t> s_nextAnimationPhaseCallbackToken{ 1 };
    std::atomic<std::uint64_t> s_nextAnimationPhaseFrameIndex{ 1 };
    std::atomic<std::uint64_t> s_activeAnimationPhaseFrameIndex{ 0 };
    std::atomic<std::uint32_t> s_animationOwnerThreadId{ 0 };
    std::atomic<bool> s_animationThreadMismatchLogged{ false };

    // The visual lock guards hFRIK hand publications on the animation thread.
    std::mutex s_handVisualAuthorityMutex;
    std::array<HandVisualAuthoritySlot, ROCK_PROVIDER_MAX_CONSUMERS_V1 * 2>
        s_handVisualAuthoritySlots{};

    // The runtime lock guards the single native animation provider lease.
    std::mutex s_nativeAnimationRuntimePublicationMutex;
    std::uint64_t s_nativeAnimationRuntimeProviderOwner{ 0 };
    RockProviderNativeAnimationRuntimePublicationV1
        s_nativeAnimationRuntimePublication{};
    bool s_hasNativeAnimationRuntimePublication{ false };
    std::uint64_t s_nativeAnimationRuntimeExpiresAfterFrame{ 0 };

    // The handling lock guards the single equipped-weapon policy owner.
    std::mutex s_equippedWeaponHandlingAuthorityMutex;
    EquippedWeaponHandlingAuthoritySlot s_equippedWeaponHandlingAuthority{};

    // The part lock guards game-thread target and drive publications.
    std::mutex s_weaponPartMutex;
    std::array<WeaponPartTargetSlot,
        ROCK_PROVIDER_MAX_WEAPON_PART_TARGETS_V1>
        s_weaponPartTargets{};
    std::array<WeaponPartDriveSlot,
        ROCK_PROVIDER_MAX_WEAPON_PART_DRIVES_V1>
        s_weaponPartDrives{};

    // The body lock joins game and physics-thread body evidence access.
    std::mutex s_externalBodyMutex;
    constinit ExternalBodyRegistry s_externalBodies{};

    // The touch lock joins game and physics-thread target access.
    std::mutex s_touchGrabMutex;
    constinit TouchGrabRegistry s_touchGrabTargets{};

    // The offhand lock guards the lease and its lock-free runtime mirror.
    std::mutex s_offhandReservationMutex;
    OffhandReservationSlot s_offhandReservationSlot{};
    std::atomic<std::uint64_t> s_offhandReservationOwner{ 0 };
    std::atomic<std::uint64_t> s_offhandReservationExpiry{ 0 };
    std::atomic<std::uint32_t> s_offhandReservation{
        static_cast<std::uint32_t>(RockProviderOffhandReservation::Normal)
    };
    std::atomic<bool> s_equippedWeaponFiringHandIsLeft{ false };
}
