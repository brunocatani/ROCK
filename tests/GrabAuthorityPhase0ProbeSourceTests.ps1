param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

$failures = New-Object System.Collections.Generic.List[string]

function Read-Text($relativePath) {
    $path = Join-Path $Root $relativePath
    if (-not (Test-Path -LiteralPath $path)) {
        $failures.Add("Missing required file: $relativePath")
        return ''
    }
    return Get-Content -Raw -LiteralPath $path
}

function Require-Text($relativePath, $pattern, $message) {
    $text = Read-Text $relativePath
    if ($text -notmatch $pattern) {
        $failures.Add($message)
    }
}

function Reject-Text($relativePath, $pattern, $message) {
    $text = Read-Text $relativePath
    if ($text -match $pattern) {
        $failures.Add($message)
    }
}

Require-Text 'src/physics-interaction/native/PhysicsStepDriveCoordinator.h' 'DriveCallback\s+_betweenCollideAndSolveCallback' `
    'PhysicsStepDriveCoordinator must expose a real between-collide-and-solve callback slot.'
Require-Text 'src/physics-interaction/native/PhysicsStepDriveCoordinator.cpp' 'onBetweenCollideAndSolve\(substepProgress,\s*substepDeltaSeconds\)' `
    'Native vtable slot +0x28 must route into the coordinator between-collide-and-solve method.'
Require-Text 'src/physics-interaction/native/HavokPhysicsTiming.h' 'BetweenCollideAndSolve' `
    'Physics timing must distinguish the pre-solve between-collide-and-solve phase.'
Require-Text 'src/physics-interaction/native/HavokPhysicsTiming.h' 'SubstepPostSolve' `
    'Physics timing must expose an after-solve phase for solver response diagnostics.'

Require-Text 'src/physics-interaction/native/GrabAuthorityPhase0Probe.cpp' 'BethesdaMotionType::Keyframed' `
    'Phase 0 proxy body must be a keyframed hidden authority probe, not a dynamic collision participant.'
Require-Text 'src/physics-interaction/native/GrabAuthorityPhase0Probe.cpp' 'BethesdaMotionType::Dynamic' `
    'Phase 0 solver probe must include an isolated dynamic receiver body.'
Require-Text 'src/physics-interaction/native/GrabAuthorityPhase0Probe.cpp' 'createGrabConstraint\(' `
    'Phase 0 solver probe must exercise the custom grab constraint instead of only logging body setters.'
Require-Text 'src/physics-interaction/native/GrabAuthorityPhase0Probe.cpp' 'setTransform\(targetHavok\)' `
    'Phase 0 proxy must write transform from the between-collide-and-solve callback.'
Require-Text 'src/physics-interaction/native/GrabAuthorityPhase0Probe.cpp' 'setVelocity\(linearVelocityHavok,\s*angularVelocityHavok\)' `
    'Phase 0 proxy must write velocity from the between-collide-and-solve callback.'
Require-Text 'src/physics-interaction/native/GrabAuthorityPhase0Probe.cpp' 'kSuppressionNoCollideBit' `
    'Phase 0 proxy filter must include the confirmed hknp no-contact bit.'
Require-Text 'src/physics-interaction/native/GrabAuthorityPhase0Probe.cpp' 'FO4_LAYER_NONCOLLIDABLE' `
    'Phase 0 default proxy filter policy must use the native noncollidable layer.'
Require-Text 'src/physics-interaction/native/GrabAuthorityPhase0Probe.cpp' 'noteSemanticContactBodyIds' `
    'Phase 0 probe must check semantic contact body ids for hidden proxy leakage.'
Reject-Text 'src/physics-interaction/native/GrabAuthorityPhase0Probe.cpp' 'NativeMouseSpringGrab|flushPendingHeldNativeGrab' `
    'Phase 0 diagnostics must not call native mouse-spring grab authority.'

Require-Text 'src/physics-interaction/native/HavokTlsDiagnostics.cpp' 'kData_HavokTlsAllocKey' `
    'Phase 0 diagnostics must read the mapped Havok TLS allocation key for command-state telemetry.'
Require-Text 'src/physics-interaction/native/HavokTlsDiagnostics.cpp' 'kCommandModeByteOffset' `
    'Phase 0 diagnostics must report the mapped TLS +0x1528 command-mode byte.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'onGrabAuthorityPhase0BetweenStep' `
    'PhysicsInteraction must wire the Phase 0 probe into the between-collide-and-solve callback.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'onGrabAuthorityPhase0AfterSolve' `
    'PhysicsInteraction must wire the Phase 0 probe into the after-solve callback.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'flushPendingHeldNativeGrab' `
    'Current native mouse-spring held-object flush must remain intact during Phase 0.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'GrabAuthorityPhase0|Phase0' `
    'Phase 0 diagnostics must not change active HandGrab behavior.'

Require-Text 'src/RockConfig.h' 'rockDebugGrabAuthorityPhase0ProbeEnabled\s*=\s*false' `
    'Phase 0 probe config must be disabled by default.'
Require-Text 'data/config/ROCK.ini' 'bDebugGrabAuthorityPhase0ProbeEnabled\s*=\s*false' `
    'Packaged ROCK.ini must keep the Phase 0 probe disabled by default.'

if ($failures.Count -gt 0) {
    Write-Host "GrabAuthorityPhase0ProbeSourceTests failed:" -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure" -ForegroundColor Red
    }
    exit 1
}

Write-Host "GrabAuthorityPhase0ProbeSourceTests passed." -ForegroundColor Green
