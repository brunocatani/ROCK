param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$failures = [System.Collections.Generic.List[string]]::new()

function Read-Source {
    param([string]$Path)
    Get-Content -Raw -LiteralPath (Join-Path $Root $Path)
}

function Require-Pattern {
    param([string]$Path, [string]$Pattern, [string]$Message)
    if ((Read-Source $Path) -notmatch $Pattern) {
        $failures.Add($Message)
    }
}

function Reject-Pattern {
    param([string]$Path, [string]$Pattern, [string]$Message)
    if ((Read-Source $Path) -match $Pattern) {
        $failures.Add($Message)
    }
}

$policy = 'src/physics-interaction/debug/DebugVisualizationPolicy.h'
$overlay = 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl'
$bodyOverlay = 'src/physics-interaction/debug/DebugBodyOverlay.cpp'

Require-Pattern $policy 'targetColliders\s*=\s*colliders\s*&&\s*input\.targetColliders[\s\S]*colliderPhaseDiagnostics\s*=\s*[\s]*colliders\s*&&\s*input\.colliderPhaseDiagnostics[\s\S]*handColliders\s*=\s*colliders\s*&&\s*input\.handColliders[\s\S]*handBoneColliders\s*=\s*colliders\s*&&\s*input\.handBoneColliders[\s\S]*bodyBoneColliders\s*=\s*colliders\s*&&\s*input\.bodyBoneColliders[\s\S]*dynamicHandColliders\s*=\s*colliders\s*&&\s*input\.dynamicHandColliders[\s\S]*weaponColliders\s*=\s*colliders\s*&&\s*input\.weaponColliders[\s\S]*grabbedWeaponPartCollider\s*=\s*[\s]*colliders\s*&&\s*input\.grabbedWeaponPartCollider[\s\S]*dynamicWeaponColliders\s*=\s*[\s]*colliders\s*&&\s*input\.dynamicWeaponColliders' 'Every ROCK-owned collider class must resolve through the collider master.'

Require-Pattern $policy 'grabForceTorqueText\s*=\s*[\s]*forceTorque\s*&&\s*input\.grabForceTorqueText[\s\S]*grabPivotSourceEvidence\s*=\s*[\s]*forceTorque\s*&&\s*input\.grabPivotSourceEvidence[\s\S]*grabTransformTelemetryAxes\s*=\s*[\s]*telemetry\s*&&\s*input\.grabTransformTelemetryAxes[\s\S]*grabTransformTelemetryText\s*=\s*[\s]*telemetry\s*&&\s*input\.grabTransformTelemetryText' 'Subordinate evidence and text streams must resolve through their suite masters.'

Require-Pattern $overlay 'debug_visualization_policy::resolve\(\{[\s\S]*const bool drawAnyRockColliderBodies[\s\S]*frame\.drawRockBodies\s*=\s*drawAnyRockColliderBodies[\s\S]*frame\.drawColliderPhaseDiagnostics\s*=\s*drawColliderPhaseDiagnostics' 'The overlay publisher must derive renderer channels from one resolved ownership state.'

Require-Pattern $overlay 'if \(drawGrabPivots\) \{[\s\S]*getGrabPivotDebugSnapshot' 'Grab pivots must draw only behind the grab-pivot owner.'
Require-Pattern $overlay 'if \(drawGrabPocketNormal\) \{[\s\S]*getGrabPocketNormalDebugSnapshot' 'Grab surface normals must draw only behind the surface-normal owner.'
Reject-Pattern $overlay 'drawGrabPivots\s*\|\|\s*drawGrabContactPatch|drawGrabPocketNormal\s*\|\|\s*drawGrabContactPatch' 'Contact-patch debug must not activate pivot or surface-normal suites.'

Require-Pattern $overlay 'drawWeaponAuthorityDebug\s*=\s*[\s]*_twoHandedGrip\.isGripping\(\)\s*&&\s*visualization\.weaponAuthority[\s\S]*drawAuthoredSupportGripDebug\s*=\s*[\s]*drawAuthoredGripActivationZones' 'Weapon authority and authored support activation must have explicit, independent owners.'
Reject-Pattern $overlay 'drawWeaponAuthorityDebug\s*=\s*[\s\S]{0,160}rockDebugShowHandAxes|drawAuthoredSupportGripDebug\s*=\s*[\s\S]{0,100}drawWeaponAuthorityDebug\s*\|\|' 'Generic hand axes and grab pivots must not activate weapon or authored-grip diagnostics.'
Require-Pattern $overlay 'if \(drawLooseWeaponGripZones\) \{[\s\S]{0,500}Loose-weapon grip zone' 'Loose-weapon grip zones must have an explicit owner separate from grab pivots.'

Require-Pattern $overlay 'auto addTriadLabel[\s\S]{0,220}if \(!drawGrabForceTorqueText\) \{[\s\S]{0,80}return;' 'Force/torque labels must fail closed behind the text child.'
Require-Pattern $overlay 'if \(drawGrabAuthorityProxyCollider\) \{[\s\S]{0,300}BodyOverlayRole::(?:Left|Right)GrabAuthorityProxy' 'The grab-authority proxy collider must require the collider master.'
Reject-Pattern $overlay 'const RE::hknpBodyId rightPalm|const RE::hknpBodyId leftPalm' 'Grab-authority proxy debug must not inject ordinary palm colliders.'

Require-Pattern $overlay 'drawGrabbedWeaponPartCollider[\s\S]*getHandGripReport\(isLeft, report\)[\s\S]{0,500}isProviderWeaponBodyCurrentV1\([\s\S]{0,180}report\.weaponGenerationKey[\s\S]{0,180}report\.bodyId[\s\S]*BodyOverlayRole::FocusedWeaponPart' 'Grabbed weapon-part focus must use the active per-hand grip identity and fail closed against the current weapon body catalog.'
Require-Pattern $overlay 'if \(drawWeaponColliders\) \{[\s\S]{0,500}isGrabbedWeaponPartColliderBody\([\s\S]{0,120}weaponSnapshot\.bodyIds\[i\][\s\S]{0,120}continue;' 'The all-weapon collider view must not redraw a focused grabbed collider with the generic role.'

Require-Pattern $overlay 'if \(drawHandBoneColliders\) \{[\s\S]*addHandBoneBodies[\s\S]*if \(drawBodyBoneColliders\) \{[\s\S]*_bodyBoneColliders\.getBodyCount' 'Hand-bone and body-bone collider publication must have separate child switches.'
Require-Pattern $bodyOverlay 'phaseDiagnosticsEnabled\s*=\s*[\s]*source\.drawColliderPhaseDiagnostics[\s\S]*if \(enabled\s*&&\s*next->phaseDiagnosticsEnabled\)[\s\S]*s_physicsPhaseCaptureEnabled\.load' 'Collider phase capture, shells, text, and physics callback work must require the explicit phase child.'

Require-Pattern 'src/physics-interaction/weapon/DynamicWeaponCollision.cpp' 'dynamicWeaponDebugEnabled\(\)[\s\S]{0,180}rockDebugShowColliders\s*&&[\s]*g_rockConfig\.rockDebugDrawDynamicWeaponColliders' 'Dynamic weapon diagnostic capture and logs must require the collider master.'
Require-Pattern 'src/physics-interaction/input/DebugControllerRuntime.cpp' 'void toggleHandColliders\(\)[\s\S]{0,500}rockDebugDrawHandColliders\s*=\s*enabled[\s\S]{0,300}persistColliderState\("bDebugDrawHandColliders"' 'The controller hand-collider command must own only the palm collider child.'
Reject-Pattern 'src/physics-interaction/input/DebugControllerRuntime.cpp' 'void toggleHandColliders\(\)[\s\S]{0,700}rockDebugDrawHandBoneColliders\s*=\s*enabled' 'The controller hand-collider command must not enable hand/body bone visualization.'

foreach ($path in @(
    'src/RockConfig.h',
    'src/RockConfig.cpp',
    'data/config/ROCK_example.ini'
)) {
    Require-Pattern $path 'DebugDrawColliderPhaseDiagnostics[\s\S]*DebugDrawBodyBoneColliders[\s\S]*DebugDrawGrabbedWeaponPartCollider[\s\S]*DebugDrawWeaponAuthority[\s\S]*DebugDrawLooseWeaponGripZones' "$path must contain the explicit visualization owners."
    Reject-Pattern $path 'DebugGrabFingerPoseLogging|DebugWorkbenchWeaponReattach' "$path must not retain dead debug controls."
}

if ($failures.Count -gt 0) {
    Write-Host 'DebugVisualizationOwnershipSourceTests failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'DebugVisualizationOwnershipSourceTests passed.' -ForegroundColor Green
