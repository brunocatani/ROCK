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


foreach ($path in @(
        'src/physics-interaction/body/BodyBoneColliderSet.cpp',
        'src/physics-interaction/weapon/collision/WeaponCollision.cpp',
        'src/physics-interaction/weapon/collision/WeaponCollisionBodies.cpp')) {
    Require-Pattern $path '_physicsCallbackGate->pauseForMutation\(\)' "$path must quiesce native callbacks around structural body-bank mutation."
}

# The generation publication is the physics thread's read barrier over the body
# bank. Reading the bank before it would race the seqlock write side in the same
# file. Literal patterns, one file, no span.
Require-Order 'src/physics-interaction/weapon/collision/WeaponCollisionBodies.cpp' @(
    'void WeaponCollision::flushPendingPhysicsDrive\(',
    'const auto publishedGeneration = getCurrentWeaponGenerationKey\(\);',
    'publishedGeneration == 0',
    'auto& bank = activeWeaponBodies\(\);'
) 'Weapon physics traversal must acquire the generation publication before selecting the active bank.'

$bodyHeader = 'src/physics-interaction/native/BethesdaPhysicsBody.h'
$bodySource = 'src/physics-interaction/native/BethesdaPhysicsBody.cpp'
Require-Pattern $bodyHeader '_createdHknpWorld[\s\S]{0,120}_createdBhkWorld' 'Each generated body must retain both creation-world identities.'
Require-Pattern $bodySource 'bool BethesdaPhysicsBody::matchesCreationWorld[\s\S]*nativeWorldFromPhysicsSystem[\s\S]*getHknpWorldFromBhk' 'Native removal must prove stored, instance, and wrapper world identity.'
Require-Pattern $bodySource 'Rejected body retirement through mismatched world' 'Mismatched native retirement must fail closed with diagnostic context.'

Require-Pattern 'src/physics-interaction/body/BodyBoneColliderSet.cpp' '_canonicalForearmTwinDimensions = forearmTwinTargets[\s\S]*applyCanonicalForearmDimensions' 'Body collider generations must capture and reapply canonical forearm dimensions.'

if ($failures.Count -gt 0) {
    Write-Host 'Havok world-lifetime source boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Havok world-lifetime source boundary passed.'
