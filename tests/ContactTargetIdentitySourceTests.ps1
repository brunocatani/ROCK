param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$failures = [System.Collections.Generic.List[string]]::new()

function Read-Text {
    param([string]$Path)
    return Get-Content -Raw -LiteralPath (Join-Path $Root $Path)
}

function Require-Text {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )

    if ((Read-Text $Path) -notmatch $Pattern) {
        $failures.Add($Message)
    }
}

function Reject-Text {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )

    if ((Read-Text $Path) -match $Pattern) {
        $failures.Add($Message)
    }
}

Require-Text 'src/physics-interaction/contact/ContactTargetIdentity.h' `
    'Contact target identity is intentionally separated from soft-contact math' `
    'ContactTargetIdentity must document ROCK evidence-first contact identity boundaries.'

Require-Text 'src/physics-interaction/contact/ContactTargetIdentity.cpp' `
    'resolveBodyToRef\(bhkWorld,\s*hknpWorld' `
    'Contact target identity must reuse the existing body-to-ref resolver instead of inventing a parallel ref path.'

Require-Text 'src/physics-interaction/contact/SoftContactRuntime.cpp' `
    'ContactTargetIdentity\.h[\s\S]*resolveContactTarget[\s\S]*Contact target identity:' `
    'SoftContactRuntime must carry and log resolved target identity for world contact candidates.'

Require-Text 'src/physics-interaction/contact/SoftContactRuntime.cpp' `
    'resolveReference\s*=\s*richDiagnostics[\s\S]*includeRichText\s*=\s*richDiagnostics' `
    'Rich contact target reference/name resolution must be gated behind the identity logging toggle.'

Require-Text 'src/physics-interaction/contact/NativeContactEvidence.h' `
    'sourceFilterInfo[\s\S]*targetFilterInfo' `
    'Native contact evidence must preserve source and target filter info for later identity diagnostics.'

Require-Text 'src/physics-interaction/core/PhysicsInteractionContacts.inl' `
    'readBodyFilterInfo[\s\S]*sourceFilterInfo[\s\S]*targetFilterInfo' `
    'Native contact callback must publish filter info without doing game-ref resolution inside the callback.'

Reject-Text 'src/physics-interaction/contact/ContactTargetIdentity.cpp' `
    'setFilterInfo|SetBodyCollisionFilterInfo|applyRock.*LayerPolicy|collisionMatrix|kFilter_CollisionMatrix' `
    'Contact target identity must remain observational and must not mutate collision layers.'

Reject-Text 'src/physics-interaction/contact/ContactTargetIdentity.h' `
    'TESObjectREFR\s*\*|TESObjectREFR\*|refr\s*=' `
    'Contact target identity must not cache raw TESObjectREFR pointers across contact candidates.'

Reject-Text 'src/physics-interaction/contact/SoftContactRuntime.cpp' `
    'setFilterInfo|SetBodyCollisionFilterInfo|applyRock.*LayerPolicy|collisionMatrix|kFilter_CollisionMatrix' `
    'SoftContactRuntime identity diagnostics must not mutate collision layers.'

if ($failures.Count -gt 0) {
    Write-Host 'Contact target identity source boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Contact target identity source boundary passed.'
