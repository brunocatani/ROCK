param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$failures = [System.Collections.Generic.List[string]]::new()

function Read-Source {
    param([string]$RelativePath)

    $path = Join-Path $Root $RelativePath
    if (-not (Test-Path -LiteralPath $path)) {
        $failures.Add("$RelativePath`: missing source file")
        return ''
    }
    return Get-Content -Raw -LiteralPath $path
}

function Require-Text {
    param(
        [string]$Text,
        [string]$Pattern,
        [string]$Message
    )

    if ($Text -notmatch $Pattern) {
        $failures.Add($Message)
    }
}

function Reject-Text {
    param(
        [string]$Text,
        [string]$Pattern,
        [string]$Message
    )

    if ($Text -match $Pattern) {
        $failures.Add($Message)
    }
}

function Get-BoundedText {
    param(
        [string]$Text,
        [string]$StartToken,
        [string]$EndToken,
        [string]$BoundaryName
    )

    $startIndex = $Text.IndexOf($StartToken, [System.StringComparison]::Ordinal)
    if ($startIndex -lt 0) {
        $failures.Add("$BoundaryName`: start boundary '$StartToken' was not found")
        return ''
    }

    $endIndex = $Text.IndexOf($EndToken, $startIndex + $StartToken.Length, [System.StringComparison]::Ordinal)
    if ($endIndex -lt 0) {
        $failures.Add("$BoundaryName`: end boundary '$EndToken' was not found")
        return ''
    }

    return $Text.Substring($startIndex, $endIndex - $startIndex)
}

function Require-OrderedTokens {
    param(
        [string]$Text,
        [string[]]$Tokens,
        [string]$Message
    )

    $searchFrom = 0
    foreach ($token in $Tokens) {
        $index = $Text.IndexOf($token, $searchFrom, [System.StringComparison]::Ordinal)
        if ($index -lt 0) {
            $failures.Add("$Message (missing or out-of-order token '$token')")
            return
        }
        $searchFrom = $index + $token.Length
    }
}

$grenadeSource = Read-Source 'src/physics-interaction/grenade/LooseGrenadeRuntime.cpp'
$physicsSource = Read-Source 'src/physics-interaction/core/PhysicsInteraction.cpp'
$physicsHeader = Read-Source 'src/physics-interaction/core/PhysicsInteraction.h'
$contactSource = Read-Source 'src/physics-interaction/core/PhysicsInteractionContacts.inl'
$pendingCommitHeader = Read-Source 'src/physics-interaction/core/PendingForceGrabCommit.h'
$grabPhasePolicy = Read-Source 'src/physics-interaction/grab/GrabThreePhase.h'
$forceGrabPolicy = Read-Source 'src/physics-interaction/core/ForceGrabPolicy.h'
$bareFistPolicy = Read-Source 'src/physics-interaction/weapon/BareFistGuardPolicy.h'
$commandPolicy = Read-Source 'src/physics-interaction/api/InteractionCommandPolicy.h'
$commandQueueHeader = Read-Source 'src/physics-interaction/api/InteractionCommandQueue.h'
$providerSource = Read-Source 'src/api/ROCKProviderApi.cpp'
$providerHeader = Read-Source 'src/api/ROCKProviderApi.h'
$fo4vrRuntime = Read-Source 'src/rock_support/Fo4VrRuntime.cpp'
$actorStatePolicy = Read-Source 'src/rock_support/Fo4VrActorStatePolicy.h'
$nativeWeaponDraw = Read-Source 'src/physics-interaction/weapon/NativeEquippedWeaponDraw.cpp'
$allRuntimeCpp = (
    Get-ChildItem -LiteralPath (Join-Path $Root 'src') -Recurse -File -Filter '*.cpp' |
        Sort-Object FullName |
        ForEach-Object { Get-Content -Raw -LiteralPath $_.FullName }
) -join "`n"

# The menu hook owns one transaction, not a replayable FIFO. A repeated Pip-Boy
# equip press is consumed while that transaction remains active.
Require-Text $grenadeSource `
    'PendingEquipRequest\s+s_pendingEquipRequest\s*\{\s*\}\s*;' `
    'Loose grenade equip interception must keep exactly one named pending request.'
Reject-Text $grenadeSource `
    'std::array\s*<\s*PendingEquipRequest' `
    'Loose grenade equip interception must not retain the old pending-request FIFO.'
Require-Text $grenadeSource `
    'enqueuePendingEquipRequest[\s\S]*?if\s*\(s_pendingEquipRequest\.active\)\s*\{\s*return false;[\s\S]*?s_pendingEquipRequest\s*=\s*PendingEquipRequest' `
    'The single pending grenade transaction must reject replacement while active.'

$equipHook = Get-BoundedText $grenadeSource 'bool hookedEquipObject(' 'bool installEquipHook()' 'loose grenade equip hook'
Require-Text $equipHook `
    'if\s*\(enqueuePendingEquipRequest[\s\S]*?return true;[\s\S]*?Ignored duplicate loose grenade equip[\s\S]*?return true;' `
    'Duplicate intercepted grenade equip presses must be reported handled without native equip or replay.'

$dropRequest = Get-BoundedText $grenadeSource 'DropResult dropPendingEquipRequestToWorld(' 'bool createExplosionAtReference(' 'loose grenade inventory drop'
Require-OrderedTokens $dropRequest @(
    'result.handle = player->RemoveItem(removeData);',
    'if (!result.handle)',
    'const auto droppedRef = result.handle.get();',
    'if (!result.droppedRef)',
    'result.success = true;',
    'result.reason = "dropped-reference-pending";'
) 'A valid RemoveItem handle with a not-yet-resolved reference must remain an asynchronous pending transaction.'

# Hand selection is decided for both hands before inventory is removed. Right is
# preferred, but the role-driven occupancy policy must permit left-hand fallback.
$grenadeService = Get-BoundedText $physicsSource 'void PhysicsInteraction::servicePendingLooseGrenadeEquip(' 'void PhysicsInteraction::servicePendingForceGrabCommits(' 'loose grenade service'
Require-OrderedTokens $grenadeService @(
    'rightBlockers = forceGrabHandBlockerMask',
    'leftBlockers = forceGrabHandBlockerMask',
    'force_grab_policy::selectGrenadeHand',
    'GrenadeSelectionFailure::HandsBlocked',
    'Cannot draw grenade - both hands are blocked.',
    'dropPendingEquipRequestToWorld'
) 'Loose grenade service must choose an available hand and reject blocked hands before inventory removal.'
Require-Text $grenadeService `
    'const bool isLeft\s*=\s*selection\.hand\s*==\s*force_grab_policy::HandChoice::Left;[\s\S]*?_pendingForceGrabCommits\[isLeft\s*\?\s*1u\s*:\s*0u\][\s\S]*?handInput\s*=\s*isLeft\s*\?\s*frame\.left\s*:\s*frame\.right' `
    'The selected grenade hand must drive both the commit slot and spawn anchor.'
Require-Text $forceGrabPolicy `
    'if\s*\(rightAvailable\)[\s\S]*?HandChoice::Right[\s\S]*?if\s*\(leftAvailable\)[\s\S]*?HandChoice::Left[\s\S]*?GrenadeSelectionFailure::HandsBlocked' `
    'Grenade hand policy must prefer right, fall back to left, then report both hands blocked.'
Require-Text $forceGrabPolicy `
    'partGripActiveForHand\s*\|\|[\s\S]*?equippedWeaponPresent\s*&&\s*!partCarryActive\s*&&\s*handIsLeft\s*==\s*firingHandIsLeft' `
    'Equipped-weapon hand occupancy must be role-driven for future left-hand weapon support.'

# A pending force-grab owns its hand and target through settle/commit. Organic
# selection, input, and the peer hand must not steal that authority.
$selectionUpdate = Get-BoundedText $physicsSource 'void PhysicsInteraction::updateSelection(' 'GrabReleaseContext PhysicsInteraction::makeGrabReleaseContext(' 'selection update'
Require-Text $selectionUpdate `
    'if\s*\(pendingTarget\)[\s\S]*?context\.exclusiveRef\s*=\s*pendingTarget;[\s\S]*?return context;' `
    'A pending force-grab target must be exclusive to the peer selection context.'
Require-Text $selectionUpdate `
    'if\s*\(_pendingForceGrabCommits\[0\]\.active\)[\s\S]*?_rightHand\.clearSelectionState\(false\);[\s\S]*?_rightHand\.stopSelectionBeam\(\);' `
    'A pending right-hand force-grab must suppress organic right-hand selection and beam state.'
Require-Text $selectionUpdate `
    'if\s*\(_pendingForceGrabCommits\[1\]\.active\)[\s\S]*?_leftHand\.clearSelectionState\(false\);[\s\S]*?_leftHand\.stopSelectionBeam\(\);' `
    'A pending left-hand force-grab must suppress organic left-hand selection and beam state.'

$grabInput = Get-BoundedText $physicsSource 'void PhysicsInteraction::updateGrabInput(' 'bool PhysicsInteraction::physicsModOwnsObject(' 'grab input update'
Require-Text $grabInput `
    'if\s*\(_pendingForceGrabCommits\[handIndex\]\.active\)\s*\{[\s\S]*?grab_input_intent_policy::reset\(inputIntentState\);[\s\S]*?cancelPeerHeldJoinRetry[\s\S]*?clearGameplayCandidatesForHand[\s\S]*?clearSelectionState\(false\);[\s\S]*?return;' `
    'A pending force-grab must reserve normal input, retry, gameplay-candidate, and selection ownership for its hand.'
Require-Text $grabInput `
    'if\s*\(_forceGrabCommittedThisFrame\[handIndex\]\)[\s\S]*?readGrabButtonState\(isLeft,\s*grabButton\)[\s\S]*?grab_input_intent_policy::reset\(inputIntentState\)[\s\S]*?return;' `
    'A successful force-grab must consume stale pre-attachment button edges and skip normal release processing for the rest of its commit frame.'

# Every retry reacquires the exact handle target and commits it in the same
# update. Success is published only after the held-ref/body postcondition.
$commitService = Get-BoundedText $physicsSource 'void PhysicsInteraction::servicePendingForceGrabCommits(' 'void PhysicsInteraction::saveGrabOffsetForHand(' 'pending force-grab commit service'
Require-OrderedTokens $commitService @(
    'if (hand.hasSelection())',
    'hand.clearSelectionState(false);',
    'hand.acquireForceGrabLooseSelection',
    'targetRef,',
    'hand.getSelection().refr != targetRef',
    'PendingForceGrabCommitPhase::AcquireAndCommitExactTarget',
    'hand.grabSelectedObject',
    'auto* heldRef = hand.getHeldRef();',
    'if (heldRef != targetRef || !exactBody)',
    'hand.releaseGrabbedObject',
    'abandon("grab postcondition did not match exact target"'
) 'Force-grab retries must acquire, commit, and verify the exact requested reference/body before reporting success.'
Require-Text $commitService `
    'if\s*\(!grabbed\)\s*\{[\s\S]*?clearSelectionState\(false\);[\s\S]*?phase\s*=\s*PendingForceGrabCommitPhase::WaitingForSettle' `
    'A refused exact-target commit must reset to settle/reacquire instead of preserving stale ready selection.'
Reject-Text $pendingCommitHeader `
    'ReadyToCommit' `
    'Pending force-grab state must not reintroduce a stale ReadyToCommit phase.'
Require-Text $grabPhasePolicy `
    'if\s*\(!result\.frontHemisphere\)[\s\S]*?if\s*\(input\.programmaticArrival\)[\s\S]*?AcquisitionPhase::NearConverging[\s\S]*?"programmaticArrivalBehindPalm"' `
    'Programmatic exact-target arrival must bypass only the organic behind-palm gate while retaining convergence.'

# Contact push assistance must not eject a freshly spawned/targeted object
# before the deferred grab transaction attaches it.
$dynamicPush = Get-BoundedText $contactSource 'void PhysicsInteraction::applyDynamicPushAssist(' 'void PhysicsInteraction::resolveAndLogContact(' 'dynamic push assist'
Require-OrderedTokens $dynamicPush @(
    'auto* targetRef = resolveBodyToRef',
    'if (!targetRef || targetRef->IsDeleted() || targetRef->IsDisabled())',
    'isPendingForceGrabTarget(targetRef)',
    'scanObjectPhysicsBodySet'
) 'Dynamic push must exclude pending force-grab targets before body scanning or impulse application.'

# API force-grab single-flight is per hand and survives dequeue until a terminal
# result. Provider loss/unregistration must cancel liveness and release leases.
Require-Text $commandPolicy `
    'class ForceGrabReservations[\s\S]*?std::array<ForceGrabReservation,\s*2>\s+_slots' `
    'Provider force-grab reservations must have independent right/left slots.'
Require-Text $commandPolicy `
    'hand\s*==\s*RockProviderHand::Right\s*\?\s*0u\s*:\s*hand\s*==\s*RockProviderHand::Left\s*\?\s*1u' `
    'Provider force-grab reservation indexing must map right and left independently.'
Require-Text $providerSource `
    'command\.kind\s*==\s*RockProviderInteractionCommandKindV1::ForceGrab\s*&&\s*s_forceGrabReservations\.isReserved\(command\.forceGrab\.hand\)[\s\S]*?RockProviderResultV1::HandBusy' `
    'A second API force-grab for the same hand must be rejected before queue insertion.'
Require-Text $providerSource `
    's_forceGrabReservations\.reserve\(command\.forceGrab\.hand,\s*command\.ownerToken,\s*command\.commandId\)' `
    'The accepted API force-grab must reserve its requested hand with command identity.'
Require-Text $providerSource `
    'interaction_command_policy::isTerminal\(result\.state\)[\s\S]*?s_forceGrabReservations\.release\(result\.ownerToken,\s*result\.commandId\)' `
    'API hand reservations must be released only by terminal command completion.'
Require-Text $providerSource `
    'isInteractionCommandActiveV1[\s\S]*?return\s+s_forceGrabReservations\.matches\(ownerToken,\s*commandId\);' `
    'Deferred force-grab liveness must use the durable reservation instead of bounded polling history.'
Require-Text $providerSource `
    'storeInteractionResultLocked[\s\S]*?slot\.active\s*&&\s*!interaction_command_policy::isTerminal\(slot\.result\.state\)[\s\S]*?continue;' `
    'The result ring must never evict a live queued command while rotating terminal polling history.'
Require-Text $providerSource `
    'clearInteractionCommandsForProviderLossV1[\s\S]*?state\s*==\s*RockProviderInteractionCommandStateV1::Queued[\s\S]*?RockProviderInteractionCommandStateV1::Cancelled[\s\S]*?s_forceGrabReservations\.clear\(\)' `
    'Provider loss must cancel dequeued pending commands and clear force-grab reservations.'
Require-Text $commandQueueHeader `
    'isInteractionCommandActiveV1\(std::uint64_t ownerToken,\s*std::uint64_t commandId\)' `
    'Runtime force-grab service must have an explicit provider-command liveness query.'
Require-Text $physicsSource `
    'dequeueInteractionCommandV1\(command\)[\s\S]*?command\.kind\s*==\s*RockProviderInteractionCommandKindV1::ForceGrab\s*&&[\s\S]*?!provider::isInteractionCommandActiveV1\(command\.ownerToken,\s*command\.commandId\)' `
    'Dequeued provider force-grabs must be discarded if owner/provider loss already cancelled them.'
Require-Text $commitService `
    'PendingForceGrabCommitOrigin::ProviderForceGrabCommand\s*&&[\s\S]*?!provider::isInteractionCommandActiveV1' `
    'Deferred provider force-grab commits must revalidate liveness before mutating hand/physics state.'
Require-Text $physicsSource `
    'void PhysicsInteraction::init\(\)[\s\S]*?clearLooseGrenadeRuntimeState\(false\);' `
    'Initialization must preserve a legitimate menu request queued while physics creation was deferred.'
Require-Text $physicsSource `
    'void PhysicsInteraction::shutdown[\s\S]*?clearLooseGrenadeRuntimeState\(true\);' `
    'Shutdown must clear the global grenade request so it cannot replay into a later runtime instance.'

# Grenades are globally single-flight even though ordinary API force-grabs can
# proceed independently per hand.
$providerCommands = Get-BoundedText $physicsSource 'void PhysicsInteraction::processProviderInteractionCommands(' 'std::size_t PhysicsInteraction::applyProviderWeaponPartDrives(' 'provider interaction command processing'
Require-Text $providerCommands `
    'targetIsLooseGrenade\s*=\s*loose_grenade_runtime::isGrenadeRef\(targetRef\)[\s\S]*?handHoldsLooseGrenade\(_rightHand\)[\s\S]*?handHoldsLooseGrenade\(_leftHand\)[\s\S]*?hasActiveLooseGrenadeCommit\(\)[\s\S]*?loose_grenade_runtime::hasPendingEquipRequest\(\)[\s\S]*?RockProviderInteractionFailureV1::HandBusy' `
    'API grenade grabs must reject while any hand holds a grenade or any menu/API grenade transaction is pending.'

# Bare fists are identified from runtime evidence, not broad hand-to-hand
# weapon type, so real unarmed weapons remain supported.
Require-Text $actorStatePolicy `
    '0x140F78D10[\s\S]*?0x140E77090[\s\S]*?0x140E77100[\s\S]*?0x140FF2B90[\s\S]*?kWeaponStateStorageOffset\s*=\s*0x0C[\s\S]*?kWeaponStateShift\s*=\s*2[\s\S]*?kWeaponStateValueMask\s*=\s*0x7[\s\S]*?kGunStateShift\s*=\s*15[\s\S]*?kGunStateValueMask\s*=\s*0xF[\s\S]*?decodeWeaponState[\s\S]*?decodeGunState' `
    'The FO4VR 1.2.72 actor-state policy must retain independent binary witnesses and both verified state layouts.'
Require-Text $fo4vrRuntime `
    'getNativeWeaponState\(const RE::Actor\* actor\)[\s\S]*?static_cast<const RE::ActorState\*>\(actor\)[\s\S]*?std::memcpy\([\s\S]*?kWeaponStateStorageOffset[\s\S]*?decodeWeaponState[\s\S]*?getNativeGunState\(const RE::Actor\* actor\)[\s\S]*?decodeGunState' `
    'ROCK must decode native weapon and gun state from verified ActorState storage rather than shifted CommonLib bitfields.'
Reject-Text $allRuntimeCpp `
    '(?:->|\.)(?:weaponState|gunState)\b|GetWeaponMagicDrawn\s*\(' `
    'ROCK runtime code must not read CommonLibF4VR weaponState, gunState, or GetWeaponMagicDrawn because that bitfield block is shifted on FO4VR 1.2.72.'
Require-Text $physicsSource `
    'updateEquippedWeaponTransition\(\)[\s\S]{0,500}getNativeGunState\(player\)[\s\S]{0,600}getNativeWeaponState\(player\)[\s\S]*?nativeStateBeforeEquip\s*=\s*[\s\S]{0,120}getNativeWeaponState\(player\)[\s\S]*?nativeStateAfterEquip\s*=\s*[\s\S]{0,120}getNativeWeaponState\(player\)' `
    'Animation ownership, transition observation, admission, and post-equip diagnostics must all use verified native state accessors.'
Require-Text $nativeWeaponDraw `
    'stateBefore\s*=\s*f4vr::getNativeWeaponState\(player\)[\s\S]*?DrawWeaponMagicHands\(true\)[\s\S]*?stateAfter\s*=\s*f4vr::getNativeWeaponState\(player\)' `
    'Bounded draw recovery must observe both sides of the native call through the verified weapon-state accessor.'
Require-Text $bareFistPolicy `
    'shouldHolster[\s\S]*?rockEnabled\s*&&\s*witness\.weaponDrawn\s*&&\s*witness\.actorUsingMelee\s*&&\s*!witness\.realMeleeWeaponEquipped' `
    'Bare-fist policy must require drawn melee state with no real melee weapon equipped.'
Require-Text $physicsHeader `
    'void enforceNoBareFistState\(bool forceRecheck\)' `
    'PhysicsInteraction must own a central bare-fist state guard.'
$fistGuard = Get-BoundedText $physicsSource 'void PhysicsInteraction::enforceNoBareFistState(' 'void PhysicsInteraction::clearLooseGrenadeImpactWatches(' 'bare-fist guard'
Require-Text $fistGuard `
    'IsWeaponDrawn\(\)[\s\S]*?CombatUtilities_IsActorUsingMelee\(legacyPlayer\)[\s\S]*?isMeleeWeaponEquipped\(\)[\s\S]*?bare_fist_guard_policy::shouldHolster[\s\S]*?DrawWeaponMagicHands\(false\)' `
    'Central bare-fist guard must holster only the verified drawn-fist fallback.'
$mainUpdate = Get-BoundedText $physicsSource 'void PhysicsInteraction::update()' 'void PhysicsInteraction::clearLeftWeaponContact()' 'main interaction update'
Require-OrderedTokens $mainUpdate @(
    'const bool forceBareFistRecheck = _equippedWeaponMenuReconcilePending;',
    'if (_equippedWeaponMenuReconcilePending)',
    '_equippedWeaponMenuReconcilePending = false;',
    'enforceNoBareFistState(forceBareFistRecheck);'
) 'Bare-fist guard must run after menu equipment reconciliation and retain an explicit forced recheck witness.'

# VirtualHolsters compatibility was removed completely. Keep the retired ABI
# probe, config surface, and input-ownership branches from returning.
foreach ($relativePath in @(
    'src/RockConfig.cpp',
    'src/RockConfig.h',
    'src/physics-interaction/core/PhysicsInteraction.cpp',
    'src/physics-interaction/input/InputRemapPolicy.h',
    'src/physics-interaction/input/InputRemapRuntime.cpp',
    'src/physics-interaction/input/InputRemapRuntime.h',
    'src/physics-interaction/weapon/WeaponSupport.h',
    'tests/InputRemapPolicyTests.cpp',
    'tests/WeaponInteractionPolicyTests.cpp',
    'data/config/ROCK.ini',
    'data/mod/ROCK_Config/ROCK.ini'
)) {
    Reject-Text (Read-Source $relativePath) `
        'VirtualHolsters|virtualHolsters|VHAPI_GetApi' `
        "VirtualHolsters compatibility must remain fully removed: $relativePath"
}

# This is an internal behavioral correction: the public ABI remains v1.
Require-Text $providerHeader `
    'ROCK_PROVIDER_API_VERSION\s*=\s*1' `
    'Interaction safety fixes must keep the public provider API at v1.'
Reject-Text $providerHeader `
    'ROCK_PROVIDER_API_VERSION\s*=\s*[2-9]' `
    'Interaction safety fixes must not bump the public provider API version.'

if ($failures.Count -gt 0) {
    Write-Host 'Interaction safety source guard failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Interaction safety source guard passed.' -ForegroundColor Green
