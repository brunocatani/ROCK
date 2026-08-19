param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
)

$ErrorActionPreference = "Stop"

$policyPath = Join-Path $Root "src/physics-interaction/grab/GrabHeldObject.h"
$policy = Get-Content -LiteralPath $policyPath -Raw

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

Write-Host "Held object contact policy source guard passed."
