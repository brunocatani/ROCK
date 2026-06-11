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

Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'getEquippedWeaponIdentityKey\(&observedIdentityKey\)' 'Weapon collision update must read equipped identity before any visual tree witness.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'getWeaponVisualCompositionKey\(weaponNode,\s*visualKeyStats\)' 'Weapon visual witness collection must be an explicit rebuild-time step.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'findGeneratedWeaponShapeSources\(weaponNode,\s*generatedSources\)' 'Weapon collision update must scan the current equipped visible geometry directly only after rebuild gates open.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'getEquippedWeaponIdentityKey\(&observedIdentityKey\)[\s\S]{0,7000}if \(rebuildRequired\)[\s\S]{0,420}getWeaponVisualCompositionKey\(weaponNode,\s*visualKeyStats\)' 'Weapon visual traversal must stay behind the identity/settings/drive rebuild gate.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'stable visual witness[\s\S]{0,1800}return;[\s\S]{0,1800}findGeneratedWeaponShapeSources\(weaponNode,\s*generatedSources\)' 'Weapon visual stabilization must wait on the cheap visual witness before running the full generated source scan.'
Reject-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'waiting for stable visual sources' 'Weapon visual stabilization must not use generated source extraction as the per-frame wait witness.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'GeneratedWeaponBodyCreateOptions\{\s*\.collisionEnabledOnCreate = false' 'Generated weapon bodies must be created collision-disabled until metadata is published.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'publishAtomicBodyIds\(activeWeaponBodies\(\)\);\s*setWeaponBodyBankCollisionEnabled\(world,\s*activeWeaponBodies\(\),\s*true\);' 'Generated weapon bodies must publish metadata before enabling collision.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.h' 'struct PendingGeneratedWeaponBuild' 'Weapon collision must track staged generated body creation explicitly.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'beginPendingGeneratedWeaponBuild\(' 'Generated weapon collision must queue full source sets before frame-sliced body creation.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'createGeneratedWeaponBodiesInBankSlice\(' 'Generated weapon collision must create native bodies through a bounded per-frame slice.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.h' 'tryBuildSupportGripEvidenceTriangles\(\s*std::uint32_t bodyId,\s*const RE::NiAVObject\* currentWeaponRoot,\s*std::vector<TriangleData>& outTriangles\) const' 'Two-hand support grip evidence must accept the current weapon root so cached triangles do not require stale drive-node dereferences.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'tryBuildSupportGripEvidenceTriangles\(decision\.bodyId,\s*weaponNode,\s*triangles\)' 'Two-hand support grip must reuse cached weapon collision geometry evidence through the current weapon root.'
Reject-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'extractAllTriangles\(sourceRoot,\s*triangles\)' 'Two-hand support grip startup must not rescan the live weapon mesh.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'Weapon visual node absent for unchanged equipped identity - retaining generated weapon bodies' 'Reload-time missing weapon visuals must retain same-identity generated bodies.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'Weapon visual node absent while rebuild required[\s\S]{0,700}destroyWeaponBody\(world\)' 'Missing weapon visuals must still destroy stale generated bodies when identity or safety rebuild gates require it.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'pendingGeneratedWeaponBuildMatches\(observedKey\)' 'Pending generated weapon creation must match equipped identity, not reload visual witnesses.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.h' 'requestWorkbenchExitRebuild\(\)' 'Weapon collision must expose an explicit one-shot workbench-exit rebuild request.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' '_workbenchExitRebuildRequested\.store\(true' 'Workbench-exit rebuild requests must be queued as explicit state, not inferred from visual churn.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'weaponNode\s*!=\s*nullptr\s*&&\s*_workbenchExitRebuildRequested\.exchange\(false' 'Workbench-exit rebuild requests must only be consumed when a drawn weapon visual is available.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'rebuildRequired\s*=\s*driveRequestedRebuild\s*\|\|\s*workbenchExitRequested\s*\|\|\s*settingsChanged\s*\|\|\s*keyChanged\s*\|\|\s*missingBodies' 'Workbench exit must be an explicit rebuild gate alongside equip/settings/drive/missing-body gates.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'pendingInvalidated\s*=\s*driveRequestedRebuild\s*\|\|\s*workbenchExitRequested\s*\|\|\s*!pendingGeneratedWeaponBuildMatches\(observedKey\)' 'A consumed workbench-exit request must restart a matching staged create rather than silently keeping old sources.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'WeaponCollisionWorkbenchExitMenuSink' 'PhysicsInteraction must own the UI menu close sink that arms the workbench-exit rebuild request.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'requestWeaponCollisionRebuildAfterWorkbenchExit\(event\.menuName\.c_str\(\)\)' 'Workbench-family menu close must arm the weapon collision rebuild gate through PhysicsInteraction.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'ensureWeaponCollisionWorkbenchExitMenuSinkRegistered\(\);[\s\S]{0,180}const auto& runtime = runtime_state::currentFrame\(\)' 'Workbench-exit menu sink registration must retry from update before normal runtime early-outs.'
Reject-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'WeaponsWorkbenchExited|WorkbenchMenuBase::vfunction4' 'ROCK must not add a raw workbench ProcessMessage hook for this rebuild gate.'
Reject-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'CollisionSuppressionOwner::WeaponDominantHand' 'WeaponCollision must not acquire/release dominant-hand suppression; PhysicsInteraction owns the complete generated hand-collider set.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'retainedWeaponCollisionActive[\s\S]{0,140}_weaponCollision\.hasWeaponBody\(\)[\s\S]{0,140}_weaponCollision\.getCurrentWeaponGenerationKey\(\)\s*!=\s*0' 'Dominant weapon authority must include retained generated weapon bodies, not only a live weapon visual node.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'rightHandWeaponAuthorityActive\s*=\s*rightHandWeaponEquipped\s*\|\|\s*retainedWeaponCollisionActive' 'Right-hand weapon suppression must stay active across reload-null visual frames with retained weapon bodies.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'if \(rightHandWeaponAuthorityActive\)[\s\S]{0,120}suppressRightHandCollisionForDominantWeapon\(hknp\)' 'PhysicsInteraction must suppress dominant-hand collision while retained weapon bodies are active.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'synchronizeContactEvidenceOwnership\(rightHandWeaponAuthorityActive,\s*leftSupportGripActive\)' 'Contact-evidence ownership must follow retained weapon authority.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' '_softContactRuntime\.update\([\s\S]{0,220}rightHandWeaponAuthorityActive' 'Soft-contact ownership must follow retained weapon authority.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'retainedPackageRootStillCurrent[\s\S]{0,220}visualSourceMissRetainFrameLimit[\s\S]{0,220}canRetainCurrentWeaponBodiesForVisualSourceMiss' 'Same-identity visual-only source misses must retain live generated weapon bodies only when the retained root is current and the retain window is bounded.'
Require-Text 'src/physics-interaction/performance/PerformanceProfiler.cpp' 'weaponRebuildVisualSourceUnavailableRetained' 'Profiler counters must expose retained same-identity visual source misses for runtime sampling.'
Require-Text 'src/physics-interaction/performance/PerformanceProfiler.cpp' 'weaponRebuildVisualSourceUnavailableRetainExpired' 'Profiler counters must expose same-identity visual source retain-window expiration for runtime sampling.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'if \(fallbackWeaponNode\)[\s\S]{0,80}return fallbackWeaponNode;' 'Live weapon collision motion/probe paths must prefer the current weapon root over cached body drive roots.'
Reject-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'safeNodeName\(instance\.driveNode\)' 'Weapon collision must not dereference cached body drive roots for mismatch logging.'
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
