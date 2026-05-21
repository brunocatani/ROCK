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

    $fullPath = Join-Path $Root $Path
    if (-not (Test-Path -LiteralPath $fullPath)) {
        $failures.Add($Message)
        return
    }

    $text = Get-Content -Raw -LiteralPath $fullPath
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

    $fullPath = Join-Path $Root $Path
    if (-not (Test-Path -LiteralPath $fullPath)) {
        return
    }

    $text = Get-Content -Raw -LiteralPath $fullPath
    if ($text -match $Pattern) {
        $failures.Add($Message)
    }
}

function Reject-Path {
    param(
        [string]$Path,
        [string]$Message
    )

    $fullPath = Join-Path $Root $Path
    if (Test-Path -LiteralPath $fullPath) {
        $failures.Add($Message)
    }
}

function Reject-TextInTree {
    param(
        [string]$Path,
        [string[]]$Include,
        [string]$Pattern,
        [string]$Message
    )

    $fullPath = Join-Path $Root $Path
    if (-not (Test-Path -LiteralPath $fullPath)) {
        return
    }

    $extensions = $Include | ForEach-Object { $_.Replace('*', '') }
    $rootFullPath = (Resolve-Path -LiteralPath $Root).Path
    foreach ($file in Get-ChildItem -LiteralPath $fullPath -Recurse -File) {
        if ($extensions -notcontains $file.Extension) {
            continue
        }

        $relative = $file.FullName
        if ($relative.StartsWith($rootFullPath, [System.StringComparison]::OrdinalIgnoreCase)) {
            $relative = $relative.Substring($rootFullPath.Length).TrimStart(
                [System.IO.Path]::DirectorySeparatorChar,
                [System.IO.Path]::AltDirectorySeparatorChar)
        }

        $text = Get-Content -Raw -LiteralPath $file.FullName
        if ($text -match $Pattern) {
            $failures.Add("$Message Found in $relative.")
        }
    }
}

Reject-Path 'src/physics-interaction/weapon/WeaponVisualRemapRuntime.cpp' 'Native weapon visual remap runtime must be removed from the collider lifecycle.'
Reject-Path 'src/physics-interaction/weapon/WeaponVisualRemapRuntime.h' 'Native weapon visual remap runtime header must be removed from the collider lifecycle.'
Reject-Path 'src/physics-interaction/weapon/WeaponInstanceWitnessRuntime.cpp' 'Weapon instance witness runtime must be removed from the collider lifecycle.'
Reject-Path 'src/physics-interaction/weapon/WeaponInstanceWitnessRuntime.h' 'Weapon instance witness runtime header must be removed from the collider lifecycle.'

Require-Text 'src/physics-interaction/weapon/WeaponCollision.h' 'struct GeneratedWeaponBodyCreateOptions' 'Weapon body creation must keep explicit creation options.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.h' 'collisionEnabledOnCreate' 'Weapon body creation options must name the initial filter state explicitly.'
Reject-Text 'src/physics-interaction/weapon/WeaponCollision.h' 'publishAfterCreate' 'Weapon body creation must not expose publication as a creation-time option.'
Reject-Text 'src/physics-interaction/weapon/WeaponCollision.h' 'createGeneratedWeaponBodiesInBank\([^\)]*bool publishAfterCreate' 'Weapon body creation must not use a boolean that couples publishing with collision filter activation.'

Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'findGeneratedWeaponShapeSources\(weaponNode,\s*generatedSources\)' 'Weapon collision update must scan the current equipped visible geometry directly.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'GeneratedWeaponBodyCreateOptions\{\s*\.collisionEnabledOnCreate = false' 'Generated weapon bodies must be created collision-disabled until metadata is published.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'publishAtomicBodyIds\(activeWeaponBodies\(\)\);\s*setWeaponBodyBankCollisionEnabled\(world,\s*activeWeaponBodies\(\),\s*true\);' 'Generated weapon bodies must publish metadata before enabling collision.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'if \(!weaponNode\)[\s\S]{0,220}destroyWeaponBody\(world\)' 'Missing weapon visuals must destroy stale generated bodies instead of retaining disabled shadow bodies.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'Generated weapon collision is now geometry-first' 'Generated weapon source extraction must document the visible-geometry-first source scan.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'candidate roots are[\s\S]{0,120}discovery witnesses' 'Generated weapon source extraction must treat candidate roots as merged witnesses, not competing winners.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'duplicate TriShape already claimed by earlier candidate' 'Merged candidate roots must dedupe overlapping source TriShapes.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'TriShape is hidden or locally zero-scale' 'Hidden TriShapes must be skipped as sources without pruning helper-node children.'

Reject-TextInTree 'src/physics-interaction/weapon' @('*.cpp', '*.h') '\b(?:QueueAttachWeapon|AttachModToReference|AttachWeapon|HandleItemEquip|SetEquippedItem|RequestLoadAnimationsForWeaponChange|QueueUpdate3D|QueueShow1stPerson|Update1StPersonArm)\b' 'Weapon collision module must not call native visual refresh/equip/remap paths.'
Reject-TextInTree 'src/physics-interaction/weapon' @('*.cpp', '*.h') '\b(?:WeaponVisualRemapRuntime|WeaponInstanceWitnessRuntime|requestCurrentFirstPersonWeaponVisualRemap|tryGetAuthoritativeEquippedWeaponIdentity|nativeVisualRemapAllowedForWitness|clearAuthoritativeEquippedWeaponWitness)\b' 'Weapon collision module must not retain native remap or instance-witness plumbing.'
Reject-TextInTree 'src/physics-interaction/weapon' @('*.cpp', '*.h') '\b(?:_pendingWeaponKey|_pendingEquippedWeaponOwnerKey|_cachedEquippedWeaponOwnerKey|_cachedEquippedWeaponInstanceWitness|_pendingEquippedWeaponInstanceWitness|_cachedGeneratedWeaponVisualWitness|_pendingGeneratedWeaponVisualWitness|_visualSettleState|_generatedSourceSettleState|_weaponBodyPending|_weaponBodiesDisabledForMissingVisual|_retryCounter|_pendingGeneratedRebuildAttemptRequested)\b' 'Weapon collision must not retain pending/settle/witness shadow state.'
Reject-TextInTree 'src/physics-interaction/weapon' @('*.cpp', '*.h') '\b(?:disableWeaponBodiesForMissingVisual|shouldReenableCachedBodiesForReturnedVisual|requestGeneratedSourceCompletenessProbeNextFrame|requestPendingGeneratedRebuildAttemptNextFrame|armGeneratedSourceCompletenessProbe|tryEnrichGeneratedWeaponBodiesFromLateSources|makeGeneratedWeaponVisualWitness)\b' 'Weapon collision must not retain refresh/probe/late-source lifecycle helpers.'
Reject-TextInTree 'src/physics-interaction/weapon' @('*.cpp', '*.h') '\b(?:evaluateEquippedVisualMissing|evaluateGeneratedCollisionPendingTransition|evaluateReturnedCachedVisualPending|sameFormEquippedInstanceRemapWitnessChanged|evaluateNativeVisualRemap|NativeVisualRemapAttemptState|InstanceVisualSyncDecision|GeneratedSourceSettleState|VisualSettleState|makeGeneratedSourcePendingSettleKey|makeGeneratedSourceReplacementSettleKey|evaluateGeneratedSourceReplacement|sourceSetImproved)\b' 'Weapon authority policy must not retain removed refresh, pending, settle, remap, or late-enrichment helpers.'
Reject-Text 'src/ROCKMain.cpp' 'weapon_instance_witness_runtime|WeaponInstanceWitnessRuntime' 'ROCK startup must not install removed weapon instance witness hooks.'
Reject-Text 'src/RockConfig.h' 'rockWeaponCollisionNativeVisualRemapEnabled' 'ROCK config must not expose removed native visual remap option.'
Reject-Text 'src/RockConfig.cpp' 'bWeaponCollisionNativeVisualRemapEnabled|rockWeaponCollisionNativeVisualRemapEnabled' 'ROCK config loader must not read removed native visual remap option.'
Reject-Text 'data/config/ROCK.ini' 'bWeaponCollisionNativeVisualRemapEnabled' 'Default ROCK.ini must not document removed native visual remap option.'

Reject-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'deferGenerationForWeaponVisualRefresh' 'Initial weapon collision generation must not defer through a visual-refresh state machine.'
Reject-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'advanceWeaponVisualRefreshFrame\(\);' 'Generated weapon collision must not retain visual-refresh cooldown advancement.'
Reject-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'generatedWeaponSourceMissingRequiredPackageCoverage' 'Firearm source scans must not use required front/rear package coverage as a generation gate.'
Reject-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'sourcePackageNeedsMoreCoverage|missingCoverageMask' 'Runtime generated collision must not block or retry visible weapon sources for semantic package coverage.'

if ($failures.Count -gt 0) {
    Write-Host 'Weapon collision lifecycle source boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Weapon collision lifecycle source boundary passed.'
