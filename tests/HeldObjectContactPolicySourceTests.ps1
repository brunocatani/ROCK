param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
)

$ErrorActionPreference = "Stop"

$policyPath = Join-Path $Root "src/physics-interaction/grab/GrabHeldObject.h"
$contactsPath = Join-Path $Root "src/physics-interaction/core/PhysicsInteractionContacts.inl"

$policy = Get-Content -LiteralPath $policyPath -Raw
$contacts = Get-Content -LiteralPath $contactsPath -Raw

$requiredPolicyPatterns = @(
    "namespace rock::held_object_contact_policy",
    "struct HeldExternalContactInput",
    "struct HeldExternalContactDecision",
    "bodyAIsHeld && input.bodyBIsHeld",
    'reason = "same-held-object"',
    'reason = "external-held-impact"'
)

foreach ($pattern in $requiredPolicyPatterns) {
    if ($policy -notlike "*$pattern*") {
        throw "Held object contact policy is missing required pattern: $pattern"
    }
}

$requiredContactPatterns = @(
    "notifyHeldExternalContact",
    "held_object_contact_policy::evaluateHeldExternalContact",
    "decision.sameHeldObject",
    "held self-contact suppressed",
    "hand.notifyHeldBodyContact(heldId, other",
    "packHeldImpactPair(heldId, other)"
)

foreach ($pattern in $requiredContactPatterns) {
    if ($contacts -notlike "*$pattern*") {
        throw "PhysicsInteractionContacts held contact path is missing required pattern: $pattern"
    }
}

$selfSuppressIndex = $contacts.IndexOf("decision.sameHeldObject")
$notifyIndex = $contacts.IndexOf("hand.notifyHeldBodyContact(heldId, other")
if ($selfSuppressIndex -lt 0 -or $notifyIndex -lt 0 -or $selfSuppressIndex -gt $notifyIndex) {
    throw "Same-held-object suppression must be evaluated before notifyHeldBodyContact."
}

Write-Host "Held object contact policy source guard passed."
