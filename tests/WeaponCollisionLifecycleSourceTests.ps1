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

function Require-OrderedText {
    param(
        [string]$Path,
        [string[]]$Patterns,
        [string]$Message
    )

    $fullPath = Join-Path $Root $Path
    if (-not (Test-Path -LiteralPath $fullPath)) {
        $failures.Add($Message)
        return
    }

    $text = Get-Content -Raw -LiteralPath $fullPath
    $offset = 0
    foreach ($pattern in $Patterns) {
        $remaining = $text.Substring($offset)
        $match = [regex]::Match($remaining, $pattern, [System.Text.RegularExpressions.RegexOptions]::Singleline)
        if (-not $match.Success) {
            $failures.Add($Message)
            return
        }
        $offset += $match.Index + $match.Length
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
Reject-Path 'src/physics-interaction/weapon/HeldWeaponEquipVisualHandoff.cpp' 'Held weapon equip phantom visual handoff must stay removed.'
Reject-Path 'src/physics-interaction/weapon/HeldWeaponEquipVisualHandoff.h' 'Held weapon equip phantom visual handoff header must stay removed.'
Reject-Path 'src/physics-interaction/weapon/HeldWeaponVisualSnapshot.h' 'Held weapon equip phantom visual snapshot must stay removed.'
Reject-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'HeldWeaponEquipVisualHandoff|captureHeldWeaponEquipVisualSnapshot|visualHandoffStarted' 'Held weapon equip must not run phantom visual handoff code.'
Reject-Text 'src/physics-interaction/core/PhysicsInteraction.h' 'HeldWeaponEquipVisualHandoff' 'PhysicsInteraction must not own phantom visual handoff state.'
Reject-Text 'src/physics-interaction/hand/Hand.h' 'HeldWeaponVisualSnapshot|captureHeldWeaponEquipVisualSnapshot' 'Hand must not expose phantom held-weapon visual snapshot capture.'

Require-Text 'src/physics-interaction/weapon/WeaponGeometry.h' 'findDetachedSourceComponentIndices[\s\S]*FailOpenNoAssembledAnchor[\s\S]*nearestAnchorGapSquared[\s\S]*minimumDetachedGapSquared' 'Detached collider filtering must use AABB-component separation and fail open without assembled weapon evidence.'
Reject-Text 'src/RockConfig.h' 'rockWeaponCollisionMaxSourceDistance' 'The superseded origin-distance collider settings must stay removed.'
Reject-Text 'src/RockConfig.cpp' 'WeaponCollisionMaxSourceDistance' 'The superseded origin-distance collider parser must stay removed.'
Reject-Text 'data/config/ROCK.ini' 'WeaponCollisionMaxSourceDistance' 'The superseded origin-distance collider template settings must stay removed.'
Reject-Text 'data/mod/ROCK_Config/ROCK.ini' 'WeaponCollisionMaxSourceDistance' 'The shipped origin-distance collider template settings must stay removed.'

# --- Publication before collision -------------------------------------------
# A generated weapon body that collides before its metadata is published makes
# the physics thread misread the contact owner. Both halves stay one literal
# pattern in one file.
$bodiesSource = 'src/physics-interaction/weapon/collision/WeaponCollisionBodies.cpp'
Require-Text $bodiesSource 'GeneratedWeaponBodyCreateOptions\{\s*\.collisionEnabledOnCreate = false' 'Generated weapon bodies must be created collision-disabled until metadata is published.'
Require-Text $bodiesSource 'publishAtomicBodyIds\(activeWeaponBodies\(\)\);\s*setWeaponBodyBankCollisionEnabled\(world,\s*activeWeaponBodies\(\),\s*true\);' 'Generated weapon bodies must publish metadata before enabling collision.'

Require-Text 'src/physics-interaction/native/BethesdaPhysicsBody.h' 'RetiredBethesdaPhysicsBodyPayload' 'Generated Bethesda body teardown must expose an explicit retired native payload.'
Require-Text 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' 'retireFromWorld[\s\S]*RemovePhysicsSystem[\s\S]*outPayload\.collisionObject' 'Retired generated Bethesda bodies must be removed from the world before native wrapper memory is delayed.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'observeCustomGrabAuthorityAfterSolve[\s\S]*_weaponCollision\.serviceRetiredWeaponBodies\(\);' 'The physics after-solve callback must service retired generated weapon bodies after native readers advance.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'getCurrentWeaponReleaseGeometry\(releaseGripWorld,\s*releaseWeaponWorld\)[\s\S]{0,2600}_weaponCollision\.destroyWeaponBody\(hknp\);[\s\S]{0,500}dropCommitted\s*&&\s*dropResult\.handle[\s\S]{0,300}armEquippedWeaponDropMomentumHandoff' 'A committed equipped-weapon drop must capture its frozen release pose and lever, retire coincident generated colliders, then arm the native handoff even when the reference resolves asynchronously.'
Require-OrderedText 'src/physics-interaction/core/PhysicsInteraction.cpp' @('enableCollisionRecursive\(droppedRoot', 'scanObjectPhysicsBodySet\(', 'completedSettleStep\(', 'currentBodySetMatches\(', 'setBodyVelocityDeferred') 'Equipped drop momentum must enable collision, rescan native bodies, cross a completed solve barrier, revalidate full body identity, and only then write velocity.'
Reject-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'serviceEquippedWeaponDropMomentumTransaction[\s\S]*?uniqueAcceptedMotionRecords\([\s\S]*?bool PhysicsInteraction::armHeldLooseGrenade' 'Equipped drop handoff must collect unique motions into fixed-capacity transaction state without per-frame unique-set allocation.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'hasAvailableEquippedWeaponDropHandoff[\s\S]*physicalDropRequested\s*&&\s*!dropHandoffAvailable[\s\S]*Cannot drop weapon - drop handoff queue is full' 'Drop handoff capacity must be reserved before inventory removal instead of silently creating an unmanaged weapon.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'releaseGeometry\.hasCapturedWeaponWorld[\s\S]{0,500}Cannot drop weapon - release pose is not ready[\s\S]{0,900}dropEquippedWeaponFromPlayer' 'Inventory removal must be blocked until a finite frozen release pose is available.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'currentRootInverse[\s\S]*rootToBody[\s\S]*handoff\.releaseWeaponWorld[\s\S]*setBodyTransformDeferred[\s\S]*zeroVelocity[\s\S]*WaitingForSettleStep' 'The exact native body set must be placed at the frozen release pose with zero velocity before crossing the solve barrier.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'completedSettleStep\([\s\S]{0,300}currentBodySetMatches\(\)[\s\S]{0,1800}handoff\.hasReleaseVelocity[\s\S]*setBodyVelocityDeferred[\s\S]*handoff\s*=\s*\{\};' 'Release momentum must be written exactly once only after one solve and full native-generation revalidation.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'originalMotionTransforms[\s\S]*queued writes were rolled back[\s\S]*partial writes were zeroed' 'Multipart drop writes must compensate partial placement and momentum commands instead of exposing a half-applied transaction.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'kPublicationStallSolveSteps\s*=\s*180[\s\S]*publicationProgressStalled\([\s\S]{0,240}handoff\.progressSolveSequence[\s\S]{0,240}completedSolveSequence' 'Async publication cleanup must use completed physics progress rather than a wall-clock timeout.'
Reject-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'GuardingWorldCrossing|WaitingForGuard|PreparingGuardRebind|detectAndCorrectWorldCrossings|makeCoveringSphereGridLayout|runWitnessSweep' 'The rejected full-flight sphere-grid guard must not remain as a dormant production path.'
Reject-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'serviceEquippedWeaponDropMomentumHandoff[\s\S]*?setMotionRecursive[\s\S]*?bool PhysicsInteraction::armHeldLooseGrenade' 'Equipped drop handoff must preserve native weapon motion properties instead of coercing them to a generic preset.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'sourceHandKnown\s*&&\s*dropCommitted' 'Dropped-reference-unavailable commits must share the post-drop hand-collision path with immediately resolved drops.'

# Hand/body bone colliders and the grab-authority proxy share the same
# broadphase-lifetime hazard as weapon bodies: freeing a keyframed collision
# object in the same call that removes it from the world lets a native reader
# (foot-IK raycast, navmesh obstacle manager) dereference freed memory. They
# must use the deferred grace-window retirement, never immediate destroy().
Require-Text 'src/physics-interaction/native/BethesdaPhysicsBody.h' 'void retireDeferred\(void\* bhkWorld\)' 'Generated Bethesda bodies must expose deferred, grace-windowed retirement.'
Require-Text 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' 'retireDeferred\(void\* bhkWorld\)[\s\S]{0,600}retireFromWorld\(bhkWorld, payload\)[\s\S]{0,600}remainingPhysicsSteps = kRetiredDeferredBodyGraceSteps' 'Deferred body retirement must remove from world then hold a bounded physics-step grace before native release.'
Require-Text 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' 'serviceRetiredDeferredPayloads\(std::uint32_t completedPhysicsSteps\)[\s\S]*releaseRetiredPayload\(retired\.payload\)' 'Deferred body retirement must be reclaimed from an explicit physics-step service point.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'observeCustomGrabAuthorityAfterSolve[\s\S]*BethesdaPhysicsBody::serviceRetiredDeferredPayloads\(\);' 'The physics after-solve callback must service deferred collider retirements after native readers advance.'
Require-Text 'src/physics-interaction/body/BodyBoneColliderSet.cpp' 'instance\.body\.retireDeferred\(' 'Body bone collider teardown must defer native release through retireDeferred.'
Reject-Text 'src/physics-interaction/body/BodyBoneColliderSet.cpp' 'instance\.body\.destroy\(' 'Body bone colliders must not immediately destroy native collision objects in the live-world teardown path.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'WeaponCollisionWorkbenchExitMenuSink' 'PhysicsInteraction must own the UI menu close sink that arms the workbench-exit rebuild request.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'requestWeaponCollisionRebuildAfterWorkbenchExit\(event\.menuName\.c_str\(\)\)' 'Workbench-family menu close must arm the weapon collision rebuild gate through PhysicsInteraction.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'ensureWeaponCollisionWorkbenchExitMenuSinkRegistered\(\);[\s\S]{0,180}const auto& runtime = runtime_state::currentFrame\(\)' 'Workbench-exit menu sink registration must retry from update before normal runtime early-outs.'
Reject-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'WeaponsWorkbenchExited|WorkbenchMenuBase::vfunction4' 'ROCK must not add a raw workbench ProcessMessage hook for this rebuild gate.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'retainedWeaponCollisionActive[\s\S]{0,140}_weaponCollision\.hasWeaponBody\(\)[\s\S]{0,140}_weaponCollision\.getCurrentWeaponGenerationKey\(\)\s*!=\s*0' 'Dominant weapon authority must include retained generated weapon bodies, not only a live weapon visual node.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'rightHandWeaponAuthorityActive\s*=\s*rightHandWeaponEquipped\s*\|\|\s*retainedWeaponCollisionActive' 'Right-hand weapon suppression must stay active across reload-null visual frames with retained weapon bodies.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'if \(rightHandWeaponAuthorityActive\)[\s\S]{0,120}suppressRightHandCollisionForDominantWeapon\(hknp\)' 'PhysicsInteraction must suppress dominant-hand collision while retained weapon bodies are active.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'synchronizeContactEvidenceOwnership\(rightHandWeaponAuthorityActive,\s*leftSupportGripActive,\s*rightPartGripActive\)' 'Contact-evidence ownership must follow retained weapon authority and part-grip drivers.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' '_dynamicHandCollision\.updateFrame\([\s\S]{0,220}rightHandWeaponAuthorityActive' 'Dynamic hand visual ownership must follow retained weapon authority.'
Require-Text 'src/physics-interaction/performance/PerformanceProfiler.cpp' 'weaponRebuildVisualSourceUnavailableRetained' 'Profiler counters must expose retained same-identity visual source misses for runtime sampling.'
Require-Text 'src/physics-interaction/performance/PerformanceProfiler.cpp' 'weaponRebuildVisualSourceUnavailableRetainExpired' 'Profiler counters must expose same-identity visual source retain-window expiration for runtime sampling.'
Require-Text 'src/physics-interaction/grab/MeshGrab.h' 'outLocalTriangles[\s\S]{0,1600}TriangleData localTriangle[\s\S]{0,700}outLocalTriangles->push_back\(localTriangle\)' 'Mesh extraction must optionally preserve native source-local triangle vertices before applying the world transform.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'beginPrimaryOnlyGrip\(\s*weaponNode,\s*currentWeaponGenerationKey,\s*currentEquippedWeaponOwnershipKey\s*,' 'Primary-only ownership must preserve a real zero collision generation while colliders build.'
Reject-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'primaryOwnershipGenerationKey|currentWeaponGenerationKey\s*!=\s*0\s*\?\s*currentWeaponGenerationKey\s*:\s*currentEquippedWeapon' 'Equipped ownership identity must never masquerade as a collision generation.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' '_equippedWeaponMenuReconcilePending[\s\S]*?isRawButtonPhysicallyHeld[\s\S]*?_pendingEquippedWeaponPrimaryOnlyGripStart' 'Menu exit must re-arm equipped weapon ownership only when the physical primary grab remains held.'
Reject-TextInTree 'src/physics-interaction/weapon' @('*.cpp', '*.h') '\b(?:QueueAttachWeapon|AttachModToReference|AttachWeapon|HandleItemEquip|SetEquippedItem|RequestLoadAnimationsForWeaponChange|QueueUpdate3D|QueueShow1stPerson|Update1StPersonArm)\b' 'Weapon collision module must not call native visual refresh/equip/remap paths.'
Reject-TextInTree 'src/physics-interaction/weapon' @('*.cpp', '*.h') '\b(?:WeaponVisualRemapRuntime|WeaponInstanceWitnessRuntime|requestCurrentFirstPersonWeaponVisualRemap|tryGetAuthoritativeEquippedWeaponIdentity|nativeVisualRemapAllowedForWitness|clearAuthoritativeEquippedWeaponWitness)\b' 'Weapon collision module must not retain native remap or instance-witness plumbing.'
Reject-TextInTree 'src/physics-interaction/weapon' @('*.cpp', '*.h') '\b(?:_pendingWeaponKey|_pendingEquippedWeaponOwnerKey|_cachedEquippedWeaponOwnerKey|_cachedEquippedWeaponInstanceWitness|_pendingEquippedWeaponInstanceWitness|_cachedGeneratedWeaponVisualWitness|_pendingGeneratedWeaponVisualWitness|_visualSettleState|_generatedSourceSettleState|_weaponBodyPending|_weaponBodiesDisabledForMissingVisual|_retryCounter|_pendingGeneratedRebuildAttemptRequested)\b' 'Weapon collision must not retain pending/settle/witness shadow state.'
Reject-TextInTree 'src/physics-interaction/weapon' @('*.cpp', '*.h') '\b(?:disableWeaponBodiesForMissingVisual|shouldReenableCachedBodiesForReturnedVisual|requestGeneratedSourceCompletenessProbeNextFrame|requestPendingGeneratedRebuildAttemptNextFrame|armGeneratedSourceCompletenessProbe|tryEnrichGeneratedWeaponBodiesFromLateSources|makeGeneratedWeaponVisualWitness)\b' 'Weapon collision must not retain refresh/probe/late-source lifecycle helpers.'
Reject-TextInTree 'src/physics-interaction/weapon' @('*.cpp', '*.h') '\b(?:evaluateEquippedVisualMissing|evaluateGeneratedCollisionPendingTransition|evaluateReturnedCachedVisualPending|sameFormEquippedInstanceRemapWitnessChanged|evaluateNativeVisualRemap|NativeVisualRemapAttemptState|InstanceVisualSyncDecision|GeneratedSourceSettleState|VisualSettleState|makeGeneratedSourcePendingSettleKey|makeGeneratedSourceReplacementSettleKey|evaluateGeneratedSourceReplacement|sourceSetImproved)\b' 'Weapon authority policy must not retain removed refresh, pending, settle, remap, or late-enrichment helpers.'
Reject-Text 'src/ROCKMain.cpp' 'weapon_instance_witness_runtime|WeaponInstanceWitnessRuntime' 'ROCK startup must not install removed weapon instance witness hooks.'
Reject-Text 'src/RockConfig.h' 'rockWeaponCollisionNativeVisualRemapEnabled' 'ROCK config must not expose removed native visual remap option.'
Reject-Text 'src/RockConfig.cpp' 'bWeaponCollisionNativeVisualRemapEnabled|rockWeaponCollisionNativeVisualRemapEnabled' 'ROCK config loader must not read removed native visual remap option.'

# --- Native VR-offset machinery: fail-closed guards ---------------------------
# The regex assertions that used to describe the OMOD self-heal were retired with
# the WeaponCollision split. What must not be lost is the fail-closed guard set in
# front of the one native call ROCK makes here. The layout itself is pinned by
# static_asserts in the source, which are the real compile-time test; these checks
# only prove the runtime guards are still present, each as one symbol in one file.
$omodAuditSource = 'src/physics-interaction/weapon/collision/WeaponCollisionOmodAudit.cpp'
Require-Text $omodAuditSource 'native_memory::guardedCopyFromMemory' 'Raw native reads in the OMOD audit must go through the guarded copy helper.'
Require-Text $omodAuditSource 'native_memory::pointerRangeLooksReadable' 'Native pointer walks in the OMOD audit must plausibility-check each hop before dereferencing it.'
Require-Text $omodAuditSource 'kExpectedPrefix' 'The native model-customization entry must keep its verified byte-prefix gate.'
Require-Text $omodAuditSource 'F4SE::RUNTIME_VR_1_2_72' 'The native model-customization entry must keep its VR executable identity gate.'
Reject-Text $omodAuditSource 'recoveryConnectPoint|recoveryNode' 'A plain NiNode must never masquerade as authored BSConnectPoint::Parents metadata.'

if ($failures.Count -gt 0) {
    Write-Host 'Weapon collision lifecycle source boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Weapon collision lifecycle source boundary passed.'
