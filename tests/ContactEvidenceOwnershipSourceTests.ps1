param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$failures = [System.Collections.Generic.List[string]]::new()

function Require-Text {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )

    $text = Get-Content -Raw -LiteralPath (Join-Path $Root $Path)
    if ($text -notmatch $Pattern) {
        $failures.Add($Message)
    }
}

function Reject-Text {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )

    $text = Get-Content -Raw -LiteralPath (Join-Path $Root $Path)
    if ($text -match $Pattern) {
        $failures.Add($Message)
    }
}

Require-Text 'src/physics-interaction/hand/HandInteractionStateMachine.h' `
    'suppressesGeneratedHandContactEvidence[\s\S]*SelectionLocked[\s\S]*Pulled[\s\S]*HeldInit[\s\S]*HeldBody[\s\S]*HeldTwoHanded' `
    'Hand state policy must expose ROCK generated-contact ownership boundaries.'

Require-Text 'src/physics-interaction/hand/Hand.h' `
    'std::atomic<HandState>\s+_stateAtomic' `
    'Physics-thread contact routing must use an atomic hand-state view, not direct Hand::_state reads.'

Require-Text 'src/physics-interaction/hand/Hand.cpp' `
    '_stateAtomic\.store\(result\.next[\s\S]*clearSemanticContactEvidence\(\)' `
    'Hand transitions into/out of ownership states must invalidate semantic contact evidence immediately.'

Require-Text 'src/physics-interaction/hand/Hand.h' `
    'std::mutex\s+_semanticContactWriteMutex' `
    'Semantic contact publish and ownership clear must have one writer boundary so stale callbacks cannot republish cleared evidence.'

Require-Text 'src/physics-interaction/hand/Hand.cpp' `
    'void Hand::recordSemanticContact[\s\S]*std::scoped_lock writeLock\(_semanticContactWriteMutex\)[\s\S]*void Hand::clearSemanticContactEvidence[\s\S]*std::scoped_lock writeLock\(_semanticContactWriteMutex\)' `
    'Semantic contact record and clear paths must serialize their sequence-slot writes.'

Require-Text 'src/physics-interaction/contact/NativeContactEvidence.h' `
    'std::uint32_t invalidateHand\(bool isLeft\)' `
    'Native contact evidence cache must support per-hand invalidation at ownership boundaries.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'synchronizeContactEvidenceOwnership\(rightHandWeaponEquipped,\s*leftSupportGripActive\);[\s\S]*_nativeContactEvidence\.snapshot' `
    'PhysicsInteraction must invalidate producer caches after grab input and before soft-contact snapshots native evidence.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'bool PhysicsInteraction::isHandContactEvidenceSuppressed[\s\S]*hasContactEvidenceSuppressedAtomic[\s\S]*_rightDominantWeaponCollisionSuppressed[\s\S]*_leftWeaponSupportCollisionSuppressed' `
    'Contact evidence suppression must combine hand ownership with weapon/support ownership.'

Require-Text 'src/physics-interaction/core/PhysicsInteractionContacts.inl' `
    'routeSourceHandContactEvidenceSuppressed[\s\S]*return;' `
    'Contact callback must stop producer-side semantic/native/dynamic-push routing when a stronger hand owner is active.'

Require-Text 'src/physics-interaction/core/PhysicsInteractionContacts.inl' `
    'rightBodyPairSuppressed[\s\S]*bodyAIsRight \|\| bodyBIsRight[\s\S]*_rightDominantWeaponCollisionSuppressed\.load[\s\S]*leftBodyPairSuppressed[\s\S]*bodyAIsLeft \|\| bodyBIsLeft[\s\S]*_leftWeaponSupportCollisionSuppressed\.load' `
    'Contact callback must also suppress stale body-pair events from weapon-owned hand colliders before side effects.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.h' `
    'std::atomic<std::uint64_t>\s+_lastHeldImpactPairRight[\s\S]*std::atomic<std::uint64_t>\s+_lastHeldImpactPairLeft' `
    'Held-impact evidence must publish held/other body IDs as one atomic pair.'

Require-Text 'src/physics-interaction/core/PhysicsInteractionContacts.inl' `
    'packHeldImpactPair\(heldId,\s*other\)' `
    'Held-impact contact producer must publish exact held/other body pairs.'

Require-Text 'src/physics-interaction/core/PhysicsInteractionContacts.inl' `
    '!_bodyBoneColliders\.isColliderBodyIdAtomic\(other\)' `
    'Held-impact contact producer must ignore generated body colliders so held items touching torso/back/legs do not emit false impacts.'

Require-Text 'src/physics-interaction/core/PhysicsInteractionContacts.inl' `
    'unpackHeldImpactPair\(packedPair,\s*heldBody,\s*otherBody\)' `
    'Held-impact contact consumer must unpack exact held/other body pairs.'

Require-Text 'src/physics-interaction/body/BodyContactRuntime.h' `
    'class BodyContactRuntime' `
    'Generated body contacts must have an internal runtime cache before public provider ABI is expanded.'

Require-Text 'src/physics-interaction/body/BodyContactRuntime.h' `
    'BodyZoneKind zone' `
    'Generated body contacts must carry stable body-zone identity instead of only descriptor indices.'

Require-Text 'src/api/ROCKProviderApi.h' `
    'RockProviderBodyContactV6' `
    'Generated body contacts must be exposed through a provider ABI row for future holsters/stash/backpack consumers.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.h' `
    'copyProviderBodyContactsV6' `
    'PhysicsInteraction must expose body contact snapshots to provider glue.'

Require-Text 'src/physics-interaction/core/PhysicsInteractionContacts.inl' `
    'recordBodyContactEvidence[\s\S]*_bodyContactRuntime\.record\(record\)' `
    'Contact callback must route generated body contacts into the internal body-contact runtime.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'pruneHeldImpactHapticCooldowns[\s\S]*_heldImpactHapticCooldownUntil\.erase' `
    'Held-impact haptic cooldowns must prune expired hand/body-pair entries during play.'

Reject-Text 'src/physics-interaction/core/PhysicsInteraction.h' `
    '_lastHeldImpactBody|_lastHeldImpactOther' `
    'Held-impact evidence must not split held and other body IDs across separate atomics.'

$contactsText = Get-Content -Raw -LiteralPath (Join-Path $Root 'src/physics-interaction/core/PhysicsInteractionContacts.inl')
$heldNotifyIndex = $contactsText.IndexOf('_rightHand.notifyHeldBodyContact')
$suppressedIndex = $contactsText.IndexOf('rightBodyPairSuppressed')
$publishIndex = $contactsText.IndexOf('publishNativeContactEvidence(routeHandMetadata)')
$semanticIndex = $contactsText.IndexOf('_rightHand.recordSemanticContact')
if ($heldNotifyIndex -lt 0 -or $suppressedIndex -lt 0 -or $publishIndex -lt 0 -or $semanticIndex -lt 0) {
    $failures.Add('Contact callback ownership boundary markers could not be located.')
} else {
    if ($heldNotifyIndex -gt $suppressedIndex) {
        $failures.Add('Held-body contact notification must stay before stronger-owner producer suppression.')
    }
    if ($suppressedIndex -gt $publishIndex -or $suppressedIndex -gt $semanticIndex) {
        $failures.Add('Stronger-owner producer suppression must run before native evidence and semantic contact recording.')
    }
}

if ($failures.Count -gt 0) {
    Write-Host 'Contact evidence ownership source boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Contact evidence ownership source boundary passed.'
