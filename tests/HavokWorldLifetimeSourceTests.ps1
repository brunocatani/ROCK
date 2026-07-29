param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
$failures = [System.Collections.Generic.List[string]]::new()

function Read-Source {
    param([string]$Path)
    $fullPath = Join-Path $Root $Path
    if (-not (Test-Path -LiteralPath $fullPath)) {
        $failures.Add("Missing source file: $Path")
        return ''
    }
    return Get-Content -Raw -LiteralPath $fullPath
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

function Require-Order {
    param([string]$Path, [string[]]$Patterns, [string]$Message)
    $text = Read-Source $Path
    $offset = 0
    foreach ($pattern in $Patterns) {
        $match = [regex]::Match(
            $text.Substring($offset),
            $pattern,
            [System.Text.RegularExpressions.RegexOptions]::Singleline)
        if (-not $match.Success) {
            $failures.Add($Message)
            return
        }
        $offset += $match.Index + $match.Length
    }
}

$coordinator = 'src/physics-interaction/native/PhysicsStepDriveCoordinator.cpp'
Require-Pattern $coordinator 'std::atomic<PhysicsStepDriveCoordinator\*>\s+owner' 'The process-lifetime native listener owner must be atomic.'
Require-Order $coordinator @(
    'void beforeWhole\(',
    'wholeUpdateLease = callbackState\.gate\.tryEnterCallback\(\)',
    'onBeforeWholePhysicsUpdate\(\)',
    'void afterWhole\(',
    'wholeUpdateLease = \{\}'
) 'One callback lease must span the complete native world update, from beforeWhole through afterWhole.'
Require-Order $coordinator @(
    'void PhysicsStepDriveCoordinator::reset\(\)',
    'gate\.pauseAndWait\(\)',
    'owner\.store\(nullptr'
) 'Coordinator reset must drain the whole-world callback lease before clearing its owner.'

$interaction = 'src/physics-interaction/core/PhysicsInteraction.cpp'
Require-Order $interaction @(
    'void PhysicsInteraction::markGeneratedBodiesInvalidated\(\)',
    '_generatedBodyStepDrive\.reset\(\);',
    'clearGeneratedBodyContactRegistry\(\);',
    '_dynamicHandCollision\.(?:retireAll|reset)\('
) 'Generated-body invalidation must quiesce callbacks before registry or body teardown.'
Require-Pattern $interaction 'dispatchFrameCallbacks\(\*this\);\s*// Publish callback ownership[\s\S]{0,220}_generatedBodyStepDrive\.registerForNextStep\(bhk, hknp\);' 'The next-step listener must publish only after all frame mutations and provider callbacks.'
Require-Pattern $interaction '_rightHand\.setPhysicsCallbackGate[\s\S]{0,600}_weaponCollision\.setPhysicsCallbackGate' 'Every generated collider owner must share the same callback-quiescence gate.'
Require-Pattern $interaction 'generatedWorldStillLive[\s\S]{0,300}_dynamicHandCollision\.retireAll\(_generatedBodiesBhkWorld\)[\s\S]{0,180}_dynamicHandCollision\.reset\(\)' 'World invalidation must retire only through the still-current exact world and otherwise abandon wrappers.'

foreach ($path in @(
        'src/physics-interaction/hand/HandBoneColliderSet.cpp',
        'src/physics-interaction/body/BodyBoneColliderSet.cpp',
        'src/physics-interaction/hand/DynamicHandCollision.cpp',
        'src/physics-interaction/weapon/WeaponCollision.cpp')) {
    Require-Pattern $path '_physicsCallbackGate->pauseForMutation\(\)' "$path must quiesce native callbacks around structural body-bank mutation."
}

Require-Order 'src/physics-interaction/weapon/WeaponCollision.cpp' @(
    'void WeaponCollision::flushPendingPhysicsDrive\(',
    'const auto publishedGeneration = getCurrentWeaponGenerationKey\(\);',
    'publishedGeneration == 0',
    'auto& bank = activeWeaponBodies\(\);'
) 'Weapon physics traversal must acquire the generation publication before selecting the active bank.'
Require-Pattern 'src/physics-interaction/weapon/WeaponCollision.cpp' 'abandonHavokStateAfterWorldLoss[\s\S]{0,700}clearWeaponBodyInstance\(instance, true\)' 'Weapon world-loss cleanup must abandon wrappers without native removal through a stale world.'

$bodyHeader = 'src/physics-interaction/native/BethesdaPhysicsBody.h'
$bodySource = 'src/physics-interaction/native/BethesdaPhysicsBody.cpp'
Require-Pattern $bodyHeader '_createdHknpWorld[\s\S]{0,120}_createdBhkWorld' 'Each generated body must retain both creation-world identities.'
Require-Pattern $bodySource 'bool BethesdaPhysicsBody::matchesCreationWorld[\s\S]*nativeWorldFromPhysicsSystem[\s\S]*getHknpWorldFromBhk' 'Native removal must prove stored, instance, and wrapper world identity.'
Require-Pattern $bodySource 'Rejected body retirement through mismatched world' 'Mismatched native retirement must fail closed with diagnostic context.'

Reject-Pattern 'src/physics-interaction/hand/DynamicHandCollision.cpp' 'dimensionsDrifted|kTwinDimensionRebuildToleranceGameUnits|kTwinConvexRadiusRebuildToleranceGameUnits' 'Pose-time dimension drift must not remain a dynamic-body reconstruction path.'
Require-Pattern 'src/physics-interaction/body/BodyBoneColliderSet.cpp' '_canonicalForearmTwinDimensions = forearmTwinTargets[\s\S]*applyCanonicalForearmDimensions' 'Body collider generations must capture and reapply canonical forearm dimensions.'
Require-Pattern 'src/physics-interaction/core/PhysicsInteractionFrame.inl' 'reloadBoundaryActive[\s\S]{0,240}kArms[\s\S]{0,120}kHands' 'Reload transition gating must derive from the existing arms/hands animation authority.'
Require-Pattern $interaction '!frame\.reloadBoundaryActive\s*&&\s*rebuildGeneratedBodiesForLifecycle' 'Lifecycle body creation must wait until the animation authority boundary closes.'
Require-Pattern $interaction 'if \(!_bodyBoneColliders\.hasBodies\(\)\) \{\s*if \(frame\.reloadBoundaryActive\)' 'Optional body collider retries must not create bodies during the animation boundary.'
Require-Pattern 'src/physics-interaction/hand/DynamicHandCollision.cpp' 'kSuppressionNoCollideBit[\s\S]*applyTransitionCollisionSuppression' 'Animation transitions must retain bodies and suppress their collision filter.'
Require-Pattern 'src/physics-interaction/hand/DynamicHandCollision.cpp' 'slot\.created && !_transitionCollisionSuppressed' 'Missing transition-time forearm targets must retain the existing body instead of retiring it.'
Require-Pattern 'src/physics-interaction/hand/DynamicHandCollision.cpp' '_transitionCollisionSuppressed[\s\S]{0,220}createdGeometryGeneration == geometryGeneration' 'Real geometry rebuilds must be deferred, not discarded, while animation collision is suspended.'

if ($failures.Count -gt 0) {
    Write-Host 'Havok world-lifetime source boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Havok world-lifetime source boundary passed.'
