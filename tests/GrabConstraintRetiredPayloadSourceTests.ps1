param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$failures = [System.Collections.Generic.List[string]]::new()

function Require-Text {
    param(
        [string]$RelativePath,
        [string]$Pattern,
        [string]$Message
    )

    $path = Join-Path $Root $RelativePath
    $text = Get-Content -Raw -LiteralPath $path
    if ($text -notmatch $Pattern) {
        $failures.Add("$RelativePath`: $Message")
    }
}

Require-Text 'src/physics-interaction/grab/GrabConstraint.cpp' 'struct RetiredGrabConstraintPayload' 'Grab constraint payload retirement must be explicit and named.'
Require-Text 'src/physics-interaction/grab/GrabConstraint.cpp' 'kRetiredGrabConstraintPayloadGraceSteps\s*=\s*8' 'Grab constraint payloads must survive multiple completed physics steps after native destroy.'
Require-Text 'src/physics-interaction/grab/GrabConstraint.cpp' 'setGrabMotorAtomsActive\(static_cast<char\*>\(constraint\.constraintData\), false, false\);[\s\S]*world->DestroyConstraints[\s\S]*retireGrabConstraintPayload\(constraint\);[\s\S]*constraint\.clear\(\);' 'Grab constraint destroy must deactivate atoms, destroy the native constraint, retire payload memory, then clear the handle.'
Require-Text 'src/physics-interaction/grab/GrabConstraint.cpp' 'void serviceRetiredGrabConstraintPayloads\([\s\S]*freeRetiredGrabConstraintPayload' 'Retired grab constraint payloads must be reclaimed from an explicit service point.'
Require-Text 'src/physics-interaction/grab/GrabConstraint.h' 'serviceRetiredGrabConstraintPayloads' 'Grab constraint payload retirement service must be exposed to the physics step owner.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'observeCustomGrabAuthorityAfterSolve[\s\S]*serviceRetiredGrabConstraintPayloads\(\);' 'The physics after-solve callback must service retired grab constraint payloads after native readers have advanced.'

if ($failures.Count -gt 0) {
    Write-Host 'GrabConstraintRetiredPayloadSourceTests failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'GrabConstraintRetiredPayloadSourceTests passed.' -ForegroundColor Green
