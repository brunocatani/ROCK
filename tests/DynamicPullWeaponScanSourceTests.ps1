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

    $text = Get-Content -Raw -LiteralPath (Join-Path $Root $Path)
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

    $text = Get-Content -Raw -LiteralPath (Join-Path $Root $Path)
    if ($text -match $Pattern) {
        $failures.Add($Message)
    }
}

Require-Text 'src/physics-interaction/native/HavokRuntime.h' 'enum class PhysicsSystemBodyScanStatus' 'Physics-system body enumeration must expose a structured scan status.'
Require-Text 'src/physics-interaction/native/HavokRuntime.h' 'forEachPhysicsSystemBodyIdDetailed' 'Object scanning must be able to inspect why native body-id enumeration failed.'
Require-Text 'src/physics-interaction/native/HavokRuntime.cpp' 'bool tryReadValue' 'Native body scanning must use guarded reads, not raw page-probed dereferences.'
Require-Text 'src/physics-interaction/native/HavokRuntime.cpp' 'tryReadValue\(&instance->bodyIds,\s*bodyIds\)' 'Native body-id arrays must be loaded through guarded reads.'
Require-Text 'src/physics-interaction/native/HavokRuntime.cpp' 'tryReadValue\(bodyIds \+ i,\s*bodyId\)' 'Native body-id entries must be read individually through the guarded reader.'
Require-Text 'src/physics-interaction/native/HavokRuntime.cpp' 'PhysicsSystemBodyScanStatus::UnreadableBodyIds' 'Unreadable physics-system body-id arrays must be a non-crashing skip reason.'
Require-Text 'src/physics-interaction/native/HavokRuntime.cpp' 'forEachPhysicsSystemBodyIdDetailed\([^\)]*\)\.enumerated\(\)' 'Legacy boolean enumeration must route through the detailed guarded scanner.'

Require-Text 'src/physics-interaction/object/ObjectPhysicsBodySet.h' 'seedBodyId' 'Object-body scans must carry the selected hit body as an explicit seed.'
Require-Text 'src/physics-interaction/object/ObjectPhysicsBodySet.h' 'requireSameResolvedRef' 'Active object scans must be able to reject bodies that resolve to a different ref.'
Require-Text 'src/physics-interaction/object/ObjectPhysicsBodySet.h' 'refResolutionKnown' 'Body records must distinguish verified native ownership from unresolved ownership fallback.'
Require-Text 'src/physics-interaction/object/ObjectPhysicsBodySet.h' 'invalidPhysicsSystems' 'Object-body diagnostics must count invalid native physics systems.'
Require-Text 'src/physics-interaction/object/ObjectPhysicsBodySet.h' 'benignScanSkips' 'Benign native scan skips must not be mixed with severe scan failures.'
Require-Text 'src/physics-interaction/object/ObjectPhysicsBodySet.h' 'unresolvedRefBodiesAccepted' 'Object-body diagnostics must expose unresolved ownership fallback bodies.'
Require-Text 'src/physics-interaction/object/ObjectPhysicsBodySet.h' 'depthLimitSkips' 'Object-body diagnostics must expose depth-truncated scans.'
Require-Text 'src/physics-interaction/object/ObjectPhysicsBodySet.cpp' 'appendBodyRecord\(seedContext,\s*options\.seedBodyId,\s*true\)' 'The selected hit body must be recorded before subtree expansion.'
Require-Text 'src/physics-interaction/object/ObjectPhysicsBodySet.cpp' 'forEachPhysicsSystemBodyIdDetailed' 'Subtree expansion must use the guarded detailed native scanner.'
Require-Text 'src/physics-interaction/object/ObjectPhysicsBodySet.cpp' 'foreignRefBodySkips' 'Seeded scans must preserve the selected-ref ownership boundary.'
Require-Text 'src/physics-interaction/object/ObjectPhysicsBodySet.cpp' 'record\.refResolutionKnown\s*=\s*true' 'Resolved bodies must be marked as verified ownership, not implicit root ownership.'
Require-Text 'src/physics-interaction/object/ObjectPhysicsBodySet.cpp' 'unresolvedRefBodiesAccepted' 'Unresolved selected-tree bodies must be accepted only through an explicit diagnostic path.'
Require-Text 'src/physics-interaction/object/ObjectPhysicsBodySet.cpp' 'Object body scan skipped invalid physics system' 'Invalid weapon/tree physics systems must be logged as skipped nodes instead of crashing.'
Reject-Text 'src/physics-interaction/object/ObjectPhysicsBodySet.cpp' 'havok_runtime::forEachPhysicsSystemBodyId\(collisionObject' 'Object-body expansion must not use the old boolean-only scanner directly.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'makeActiveGrabBodyScanOptions' 'Active grab and pull scans must share one seeded scan-option builder.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'scanOptions\.seedBodyId\s*=\s*selection\.bodyId\.value' 'The shared scan-option builder must seed object scanning from the selected body.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'scanOptions\.requireSameResolvedRef\s*=\s*true' 'The shared scan-option builder must keep expanded bodies owned by the selected ref.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'tryUseGrabAcquisitionBeforePrepCache' 'Active grab startup must have a pre-prep acquisition cache path.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'tryBuildGrabAcquisitionPreparedBodySetFromCache' 'Active grab startup may rebuild prepared records from cached body-id evidence only with explicit completeness reporting.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'outPostPrepComplete' 'Cached prepared record rebuilds must report whether post-prep discovery was actually complete.'

Require-Text 'src/physics-interaction/grab/GrabCore.h' 'capturedAfterPrep' 'Lifecycle records must identify bodies discovered only after recursive active prep.'
Require-Text 'src/physics-interaction/grab/GrabCore.h' 'originalStateKnown' 'Lifecycle restore plans must not restore manufactured original state for late bodies.'
Require-Text 'src/physics-interaction/grab/GrabCore.h' 'latePreparedBodyCount' 'Lifecycle diagnostics must count bodies that appeared only after active prep.'
Require-Text 'src/physics-interaction/grab/GrabCore.h' 'hasIncompleteNativeScan' 'Lifecycle snapshots must expose incomplete native scans to runtime cleanup.'
Require-Text 'src/physics-interaction/grab/GrabCore.h' 'depthLimitSkips\s*>\s*0' 'Lifecycle snapshots must mark depth-truncated scans incomplete.'
Require-Text 'src/physics-interaction/grab/GrabCore.h' 'BodyReleaseIntent' 'Lifecycle release planning must carry explicit release intent.'
Require-Text 'src/physics-interaction/grab/GrabCore.h' 'shouldPreserveConvertedMotionOnRelease' 'Lifecycle release planning must preserve converted keyframed loose-object physical drops.'
Require-Text 'src/physics-interaction/grab/GrabCore.h' 'shouldSkipIncompleteScanRootRestore' 'Lifecycle release planning must expose when incomplete-scan root restore would undo an intentional physical drop.'
Require-Text 'src/physics-interaction/grab/GrabCore.h' 'plan\.motionRestoreCount\s*==\s*0\s*&&\s*plan\.filterRestoreCount\s*==\s*0[\s\S]*return\s+true' 'Incomplete-scan root restore must not run after loose dynamic physical drops that intentionally preserve active motion and filters.'
Require-Text 'src/physics-interaction/grab/GrabCore.h' 'reason == BodyRestoreReason::FailedGrabSetup \|\| policy == BodyRestorePolicy::RestoreAllChanged[\s\S]*entry\.restoreFilter\s*=\s*record\.originalStateKnown' 'Failure and explicit restore-all paths must restore captured filters.'
Require-Text 'src/physics-interaction/grab/GrabCore.h' 'entry\.restoreFilter\s*=\s*record\.originalStateKnown && record\.motionRole == MotionRole::SystemOwnedNonDynamic' 'Physical release must not restore inactive filters onto loose dynamic objects that ROCK leaves dynamic.'
Require-Text 'tests/ActiveGrabBodyLifecyclePolicyTests.cpp' 'loose dynamic release should keep active collision filter' 'Compiled lifecycle policy tests must cover loose dynamic release collision ownership.'
Require-Text 'tests/ActiveGrabBodyLifecyclePolicyTests.cpp' 'loose keyframed physical drop should keep converted dynamic motion' 'Compiled lifecycle policy tests must cover keyframed loose-object physical drop preservation.'
Require-Text 'tests/ActiveGrabBodyLifecyclePolicyTests.cpp' 'loose keyframed non-physical transfer should restore motion' 'Compiled lifecycle policy tests must cover non-physical keyframed restoration.'
Require-Text 'tests/ActiveGrabBodyLifecyclePolicyTests.cpp' 'pull-consumed loose keyframed physical drop should keep converted dynamic motion' 'Compiled lifecycle policy tests must cover pull-to-grab lifecycle release preservation.'
Require-Text 'tests/ActiveGrabBodyLifecyclePolicyTests.cpp' 'depth-truncated scans should force incomplete restore fallback' 'Compiled lifecycle policy tests must cover depth-truncated scan fallback.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'finishPullPrepAsPhysicalDropIfActive\("pull-release"\)' 'User dynamic-pull release must finish pull prep as a physical drop, not a failed setup restore.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'finishPullPrepAsPhysicalDropIfActive\("pull-catch-normal-grab-suppressed"\)' 'Suppressed normal grab input must preserve an active converted pull-catch object as a physical drop.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'finishPullPrepAsPhysicalDropIfActive\("pull-catch-stale-reacquire-failed"\)' 'Stale pull-catch reacquire failure must preserve the converted pulled object as a physical drop.'

Require-Text 'src/physics-interaction/hand/Hand.cpp' 'restorePullPrepIfActive\(context\)' 'Clearing pull runtime state must restore abandoned pull prep when the world is still valid.'

if ($failures.Count -gt 0) {
    Write-Host 'Dynamic pull weapon scan boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Dynamic pull weapon scan boundary passed.'
