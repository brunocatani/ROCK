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
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'tryBuildGrabAcquisitionPreparedBodySetFromCache' 'Active grab startup must rebuild prepared records from cached body-id evidence.'

Require-Text 'src/physics-interaction/grab/GrabCore.h' 'capturedAfterPrep' 'Lifecycle records must identify bodies discovered only after recursive active prep.'
Require-Text 'src/physics-interaction/grab/GrabCore.h' 'originalStateKnown' 'Lifecycle restore plans must not restore manufactured original state for late bodies.'
Require-Text 'src/physics-interaction/grab/GrabCore.h' 'latePreparedBodyCount' 'Lifecycle diagnostics must count bodies that appeared only after active prep.'
Require-Text 'src/physics-interaction/grab/GrabCore.h' 'hasIncompleteNativeScan' 'Lifecycle snapshots must expose incomplete native scans to runtime cleanup.'
Require-Text 'src/physics-interaction/grab/GrabCore.h' 'depthLimitSkips\s*>\s*0' 'Lifecycle snapshots must mark depth-truncated scans incomplete.'
Require-Text 'src/physics-interaction/grab/GrabCore.h' 'reason == BodyRestoreReason::FailedGrabSetup \|\| policy == BodyRestorePolicy::RestoreAllChanged[\s\S]*entry\.restoreFilter\s*=\s*record\.originalStateKnown' 'Failure and explicit restore-all paths must restore captured filters.'
Require-Text 'src/physics-interaction/grab/GrabCore.h' 'entry\.restoreFilter\s*=\s*record\.originalStateKnown && record\.motionRole == MotionRole::SystemOwnedNonDynamic' 'Physical release must not restore inactive filters onto loose dynamic objects that ROCK leaves dynamic.'
Require-Text 'tests/ActiveGrabBodyLifecyclePolicyTests.cpp' 'loose dynamic release should keep active collision filter' 'Compiled lifecycle policy tests must cover loose dynamic release collision ownership.'
Require-Text 'tests/ActiveGrabBodyLifecyclePolicyTests.cpp' 'system-owned release should restore filter' 'Compiled lifecycle policy tests must cover system-owned release restoration.'
Require-Text 'tests/ActiveGrabBodyLifecyclePolicyTests.cpp' 'depth-truncated scans should force incomplete restore fallback' 'Compiled lifecycle policy tests must cover depth-truncated scan fallback.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'restoreIncompleteActivePrepRoot' 'Incomplete object scans must have an explicit recursive root restore fallback.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'release-incomplete-scan' 'Release cleanup must invoke the incomplete-scan root restore path.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'releaseRestorePolicyForTargetKind\(_savedObjectState\.targetKind\)' 'Release cleanup must choose target-aware lifecycle restore policy from the saved target kind.'

$handGrabText = Get-Content -Raw -LiteralPath (Join-Path $Root 'src/physics-interaction/hand/HandGrab.cpp')
$pullStart = $handGrabText.IndexOf('bool Hand::startDynamicPull')
$pullEnd = if ($pullStart -ge 0) { $handGrabText.IndexOf('bool Hand::updateDynamicPull', $pullStart) } else { -1 }
if ($pullStart -lt 0 -or $pullEnd -lt 0) {
    $failures.Add('Hand::startDynamicPull boundary could not be located.')
} else {
    $pullText = $handGrabText.Substring($pullStart, $pullEnd - $pullStart)
    if ($pullText -notmatch 'makeActiveGrabBodyScanOptions\(_currentSelection\)') {
        $failures.Add('Dynamic pull must build seeded object scan options from the current far-hit selection.')
    }
    if ($pullText -notmatch 'tryUseGrabAcquisitionBeforePrepCache') {
        $failures.Add('Dynamic pull must try the prewarmed acquisition cache before a direct tree scan.')
    }
    if ($pullText -notmatch 'tryBuildGrabAcquisitionPreparedBodySetFromCache') {
        $failures.Add('Dynamic pull must rebuild prepared body records from cached acquisition evidence before falling back to a direct scan.')
    }
    if ($pullText -notmatch 'PULL scan:') {
        $failures.Add('Dynamic pull must log seeded scan diagnostics.')
    }
    if ($pullText -notmatch 'unresolvedAccepted') {
        $failures.Add('Dynamic pull logs must expose unresolved ownership fallback counts.')
    }
    if ($pullText -notmatch 'scanSource=\{\}/\{\}') {
        $failures.Add('Dynamic pull logs must expose cache/direct scan source for both before and prepared body sets.')
    }
    if ($pullText -notmatch '_pulledBodyIds\s*=\s*preparedBodySet\.acceptedBodyIds\(\)') {
        $failures.Add('Dynamic pull must retain all accepted object bodies for activation/ownership, not collapse multipart weapons to one motion body.')
    }
    if ($pullText -match '_pulledBodyIds\s*=\s*preparedBodySet\.uniqueAcceptedMotionBodyIds\(\)') {
        $failures.Add('Dynamic pull must not use the unique-motion body list as the ownership body set.')
    }
    if ($pullText -notmatch 'pullLifecycle\.captureBeforeActivePrep\(beforePrepBodySet\)' -or
        $pullText -notmatch 'pullLifecycle\.markPreparedBodies\(preparedBodySet\)' -or
        $pullText -notmatch '_pullActiveLifecycle\s*=\s*pullLifecycle' -or
        $pullText -notmatch '_pullPrepRestoreArmed\s*=\s*true') {
        $failures.Add('Dynamic pull must arm a lifecycle snapshot so abandoned pull prep can restore motion/filter state.')
    }
    if ($pullText -match 'markPullCatchIntentArrived\(\);\s*clearPullRuntimeState\(') {
        $failures.Add('Pull arrival must keep the pull prep lifecycle armed until pull-catch grab consumes it or selection cleanup restores it.')
    }
}

$grabStart = $handGrabText.IndexOf('bool Hand::grabSelectedObject')
$grabEnd = if ($grabStart -ge 0) { $handGrabText.IndexOf('void Hand::updateHeldObject', $grabStart) } else { -1 }
if ($grabStart -lt 0 -or $grabEnd -lt 0) {
    $failures.Add('Hand::grabSelectedObject boundary could not be located.')
} else {
    $grabText = $handGrabText.Substring($grabStart, $grabEnd - $grabStart)
    if ($grabText -notmatch 'makeActiveGrabBodyScanOptions\(sel\)') {
        $failures.Add('Close/pull-catch grab must build seeded object scan options from the selected body.')
    }
    if ($grabText -notmatch 'tryUseGrabAcquisitionBeforePrepCache') {
        $failures.Add('Close/pull-catch grab must try the prewarmed acquisition cache before a direct tree scan.')
    }
    if ($grabText -notmatch 'tryBuildGrabAcquisitionPreparedBodySetFromCache') {
        $failures.Add('Close/pull-catch grab must rebuild prepared body records from cached acquisition evidence before falling back to a direct scan.')
    }
    if ($grabText -notmatch 'invalidSystems') {
        $failures.Add('Grab object-tree prep logs must include invalid native physics-system diagnostics.')
    }
    if ($grabText -notmatch 'latePrepared') {
        $failures.Add('Grab object-tree prep logs must include late body lifecycle diagnostics.')
    }
    if ($grabText -notmatch 'scanSource=\{\}/\{\}') {
        $failures.Add('Grab object-tree prep logs must expose cache/direct scan source for both before and prepared body sets.')
    }
    if ($grabText -notmatch 'restoreIncompleteActivePrepRoot\(rootNode,\s*selectedOriginalMotionPropsId') {
        $failures.Add('Failed grab setup must restore the root when body scanning was incomplete.')
    }
    if ($grabText -notmatch 'consumePullPrepLifecycleForActiveGrab\(sel\.refr,\s*activeLifecycle\)') {
        $failures.Add('Pull-catch grab must consume the pull lifecycle snapshot instead of recapturing post-pull state.')
    }
}

Require-Text 'src/physics-interaction/hand/Hand.cpp' 'restorePullPrepIfActive\(context\)' 'Clearing pull runtime state must restore abandoned pull prep when the world is still valid.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'restorePullPrepIfActive' 'Pull prep lifecycle restore helper must be implemented in the grab runtime.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'clearPullPrepTracking' 'Pull prep tracking must have an explicit abandon/consume cleanup path.'

if ($failures.Count -gt 0) {
    Write-Host 'Dynamic pull weapon scan boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Dynamic pull weapon scan boundary passed.'
