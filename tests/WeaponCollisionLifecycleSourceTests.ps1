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
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'captureHeldWeaponEquipVisualSnapshot' 'Hand grab runtime must not retain phantom held-weapon visual snapshot capture.'

Require-Text 'src/physics-interaction/weapon/WeaponCollision.h' 'struct GeneratedWeaponBodyCreateOptions' 'Weapon body creation must keep explicit creation options.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.h' 'collisionEnabledOnCreate' 'Weapon body creation options must name the initial filter state explicitly.'
Reject-Text 'src/physics-interaction/weapon/WeaponCollision.h' 'publishAfterCreate' 'Weapon body creation must not expose publication as a creation-time option.'
Reject-Text 'src/physics-interaction/weapon/WeaponCollision.h' 'createGeneratedWeaponBodiesInBank\([^\)]*bool publishAfterCreate' 'Weapon body creation must not use a boolean that couples publishing with collision filter activation.'

Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'if \(!weaponDrawn\)[\s\S]{0,700}clearCurrentWeaponState\(\);[\s\S]{0,140}return;[\s\S]{0,500}getEquippedWeaponIdentityKey\(&observedIdentityKey,' 'Weapon collision update must clear not-drawn weapons before reading equipped instance identity data.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'getEquippedWeaponIdentityKey\(&observedIdentityKey,' 'Weapon collision update must read equipped identity before any visual tree witness.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'getWeaponVisualCompositionKey\(weaponNode,\s*visualKeyStats\)' 'Weapon visual witness collection must be an explicit rebuild-time step.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'findGeneratedWeaponShapeSources\(weaponNode,\s*observedKey,\s*generatedSources,' 'Weapon collision update must scan the current equipped visible geometry directly only after rebuild gates open.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'getEquippedWeaponIdentityKey\(&observedIdentityKey,[\s\S]{0,7000}if \(rebuildRequired\)[\s\S]{0,420}getWeaponVisualCompositionKey\(weaponNode,\s*visualKeyStats\)' 'Weapon visual traversal must stay behind the identity/settings/drive rebuild gate.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'stable visual witness[\s\S]{0,1800}return;[\s\S]{0,1800}findGeneratedWeaponShapeSources\(weaponNode,\s*observedKey,\s*generatedSources,' 'Weapon visual stabilization must wait on the cheap visual witness before running the full generated source scan.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'generationDrivenRebuild[\s\S]{0,1200}maybeRunWeaponOmodCoverageAudit\(weaponNode,\s*observedKey,\s*true\)[\s\S]{0,900}auditResult\.sceneEnriched[\s\S]{0,900}clearPendingWeaponVisualRebuild\(\);[\s\S]{0,300}return;' 'Initial weapon generation must complete OMOD scene enrichment for the observed equipped identity before beginning collider source stabilization.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'auditResult\.sceneEnriched[\s\S]{0,900}return;[\s\S]{0,300}auditResult\.ran[\s\S]{0,300}_omodPrebuildAuditEquippedKey\s*=\s*observedKey' 'A mutating OMOD pre-build pass must repeat until a later non-mutating pass confirms convergence.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'publishedBodyEvidenceMatchesAudit\(\s*auditedEquippedKey,\s*_cachedWeaponKey,[\s\S]{0,500}if \(publishedBodyEvidenceCurrent\)[\s\S]{0,300}activeWeaponBodies\(\)' 'OMOD audit must read published collider evidence only when the body bank belongs to the audited equipped generation.'
Require-Text 'src/physics-interaction/weapon/WeaponOmodAuditPolicy.h' 'if \(input\.disabled\)\s*\{\s*return \{ \.verdict = CoverageVerdict::Disabled \};\s*\}' 'Disabled OMOD records must receive a diagnostic verdict without entering the missing-part self-heal path.'
Require-Text 'src/physics-interaction/weapon/WeaponOmodAuditPolicy.h' 'if \(input\.hasNodeMatch\)[\s\S]{0,1100}NodePresentNoCollider[\s\S]{0,350}\.selfHealCandidate\s*=\s*true' 'Token-only node matches must still pass through equipped-instance template verification before being trusted as OMOD coverage.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'loadCompleteOmodModelTemplate[\s\S]{0,1800}0xED[\s\S]*collectOmodPhysicalTemplateSignature[\s\S]*durableAnchorName[\s\S]*physicalTemplateSignatureIsPresent' 'OMOD completeness must inspect the complete native model hierarchy and require its largest non-effect physical housing.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'loadGeometryInspectionOmodModelTemplate[\s\S]{0,500}0x20[\s\S]*shouldPreferRawReceiverGeometryTemplate[\s\S]*usesRawReceiverGeometry[\s\S]*nativeWholeModelEligible[\s\S]*!recoveryTemplate->usesRawReceiverGeometry' 'Physics-bearing receiver OMODs may use only a strict-superset raw geometry view, and that raw-only anchor must bypass the native postprocessed attach that consumed it.'
Require-Text 'src/physics-interaction/weapon/WeaponOmodAuditPolicy.h' 'shouldAttemptWholeModelAttach[\s\S]*!templateSignatureIsPresent\(matchedDistinctMeshCount,\s*distinctTemplateMeshCount\)' 'Incidental sub-majority name matches must retain native whole-model recovery; only a coherent partial branch may bypass it.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'kRockOmodEnrichmentPrefix[\s\S]*enrichMissingOmodPhysicalAnchor[\s\S]*containerName[\s\S]*cloneNode[\s\S]*applyEquippedOmodModelCustomization[\s\S]*clonedParent[\s\S]*collectWeaponAnimNodeMatches\(coverageRoot,\s*parentName\)[\s\S]*createEngineNiNode\(1\)[\s\S]*AttachChild\(enrichmentContainer' 'Missing physical housings must clone the authored asset, preserve equipped material customization, prefer the existing animated runtime parent, and remain inside a ROCK-owned container.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'enum class OmodPhysicalEnrichmentStage[\s\S]*AnchorLookupFailed[\s\S]*AnchorDetachFailed[\s\S]*AttachVerificationFailed[\s\S]*enrichmentStage=' 'Failed physical-housing enrichment must publish one bounded stage code that discriminates the next runtime diagnosis.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'NativeConnectPointParentLayout[\s\S]*GetExtraData\(cpaKey\)[\s\S]*BSConnectPoint::Parents[\s\S]*recoveryProviderAttachPointForAttachPoint' 'Whole-part recovery must read the installed provider NIF CPA metadata instead of guessing a synthetic P-* scene node.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'prepareAuthoredOmodParentPath[\s\S]*sourceNode->local[\s\S]*liveAncestor->AttachChild\(container\.get\(\),\s*true\)[\s\S]*tryAttach3DRecurse[\s\S]*authoredPathCapturedAnchor[\s\S]*rollbackAuthoredOmodParentPath' 'Missing attachment parents must be reconstructed from authored static transforms before native whole-model attachment and retained only after geometry lands below the owned path.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'removeStaleRockOmodEnrichmentContainers[\s\S]*activeOmodFormIds[\s\S]*DetachChild[\s\S]*requestWorkbenchExitRebuild' 'ROCK-owned physical enrichment must be removed deterministically when its OMOD leaves the equipped instance.'
Reject-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'recoveryConnectPoint|recoveryNode' 'A plain NiNode must never masquerade as authored BSConnectPoint::Parents metadata.'
Reject-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'waiting for stable visual sources' 'Weapon visual stabilization must not use generated source extraction as the per-frame wait witness.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'GeneratedWeaponBodyCreateOptions\{\s*\.collisionEnabledOnCreate = false' 'Generated weapon bodies must be created collision-disabled until metadata is published.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'publishAtomicBodyIds\(activeWeaponBodies\(\)\);\s*setWeaponBodyBankCollisionEnabled\(world,\s*activeWeaponBodies\(\),\s*true\);' 'Generated weapon bodies must publish metadata before enabling collision.'
Require-Text 'src/physics-interaction/native/BethesdaPhysicsBody.h' 'RetiredBethesdaPhysicsBodyPayload' 'Generated Bethesda body teardown must expose an explicit retired native payload.'
Require-Text 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' 'retireFromWorld[\s\S]*RemovePhysicsSystem[\s\S]*outPayload\.collisionObject' 'Retired generated Bethesda bodies must be removed from the world before native wrapper memory is delayed.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.h' 'RetiredWeaponBodyPayload' 'Weapon collision must own retired generated body payload state.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'destroyWeaponBodyBank[\s\S]*retireWeaponBodyInstance\(instance,\s*releaseShapeRef\)' 'Generated weapon body banks must retire native payloads instead of immediately releasing collision objects during rebuild handoff.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'retireWeaponBodyPayload[\s\S]*RETIRED_GENERATED_WEAPON_BODY_GRACE_STEPS' 'Retired generated weapon bodies must wait a bounded number of physics steps before native release.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'serviceRetiredWeaponBodies[\s\S]*BethesdaPhysicsBody::releaseRetiredPayload' 'Retired generated weapon bodies must be reclaimed from an explicit service point.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'observeCustomGrabAuthorityAfterSolve[\s\S]*_weaponCollision\.serviceRetiredWeaponBodies\(\);' 'The physics after-solve callback must service retired generated weapon bodies after native readers advance.'
Reject-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'destroyWeaponBodyBank[\s\S]{0,260}instance\.body\.destroy\(_cachedBhkWorld\)' 'Generated weapon body bank teardown must not immediately destroy native wrapper bodies in the rebuild path.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'getCurrentWeaponReleaseGeometry\(releaseGripWorld,\s*releaseWeaponWorld\)[\s\S]{0,2600}_weaponCollision\.destroyWeaponBody\(hknp\);[\s\S]{0,500}dropCommitted\s*&&\s*dropResult\.handle[\s\S]{0,300}armEquippedWeaponDropMomentumHandoff' 'A committed equipped-weapon drop must capture its frozen release pose and lever, retire coincident generated colliders, then arm the native handoff even when the reference resolves asynchronously.'
Require-OrderedText 'src/physics-interaction/core/PhysicsInteraction.cpp' @('enableCollisionRecursive\(droppedRoot', 'scanObjectPhysicsBodySet\(', 'completedSettleStep\(', 'currentBodySetMatches\(', 'setBodyVelocityDeferred') 'Equipped drop momentum must enable collision, rescan native bodies, cross a completed solve barrier, revalidate full body identity, and only then write velocity.'
Reject-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'serviceEquippedWeaponDropMomentumTransaction[\s\S]*?uniqueAcceptedMotionRecords\([\s\S]*?bool PhysicsInteraction::armHeldLooseGrenade' 'Equipped drop handoff must collect unique motions into fixed-capacity transaction state without per-frame unique-set allocation.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'hasCapturedWeaponWorld\s*=\s*true[\s\S]{0,120}capturedWeaponWorld\s*=\s*capturedWeaponWorld[\s\S]{0,220}gripWorldPoint' 'Release-pose capture must remain available even when grip evidence cannot provide a lever.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'hasAvailableEquippedWeaponDropHandoff[\s\S]*physicalDropRequested\s*&&\s*!dropHandoffAvailable[\s\S]*Cannot drop weapon - drop handoff queue is full' 'Drop handoff capacity must be reserved before inventory removal instead of silently creating an unmanaged weapon.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'releaseGeometry\.hasCapturedWeaponWorld[\s\S]{0,500}Cannot drop weapon - release pose is not ready[\s\S]{0,900}dropEquippedWeaponFromPlayer' 'Inventory removal must be blocked until a finite frozen release pose is available.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'currentRootInverse[\s\S]*rootToBody[\s\S]*handoff\.releaseWeaponWorld[\s\S]*setBodyTransformDeferred[\s\S]*zeroVelocity[\s\S]*WaitingForSettleStep' 'The exact native body set must be placed at the frozen release pose with zero velocity before crossing the solve barrier.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'completedSettleStep\([\s\S]{0,300}currentBodySetMatches\(\)[\s\S]{0,1800}handoff\.hasReleaseVelocity[\s\S]*setBodyVelocityDeferred[\s\S]*handoff\s*=\s*\{\};' 'Release momentum must be written exactly once only after one solve and full native-generation revalidation.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'originalMotionTransforms[\s\S]*queued writes were rolled back[\s\S]*partial writes were zeroed' 'Multipart drop writes must compensate partial placement and momentum commands instead of exposing a half-applied transaction.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'kPublicationStallSolveSteps\s*=\s*180[\s\S]*publicationProgressStalled\([\s\S]{0,240}handoff\.progressSolveSequence[\s\S]{0,240}completedSolveSequence' 'Async publication cleanup must use completed physics progress rather than a wall-clock timeout.'
Reject-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'GuardingWorldCrossing|WaitingForGuard|PreparingGuardRebind|detectAndCorrectWorldCrossings|makeCoveringSphereGridLayout|runWitnessSweep' 'The rejected full-flight sphere-grid guard must not remain as a dormant production path.'
Reject-Text 'src/physics-interaction/native/PhysicsShapeCast.cpp' 'FixedFilteredClosestCollector|prewarmConservativeSphereCastShapes|g_conservativeSphereShapes' 'The rejected drop-only sphere cache and collector must be removed.'
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
Require-Text 'src/physics-interaction/hand/HandBoneColliderSet.cpp' 'instance\.body\.retireDeferred\(' 'Hand bone collider teardown must defer native release through retireDeferred.'
Require-Text 'src/physics-interaction/hand/HandBoneColliderSet.cpp' 'palmAnchorBody\.retireDeferred\(' 'Hand palm-anchor teardown must defer native release through retireDeferred.'
Require-Text 'src/physics-interaction/body/BodyBoneColliderSet.cpp' 'instance\.body\.retireDeferred\(' 'Body bone collider teardown must defer native release through retireDeferred.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' '_grabAuthorityProxy\.retireDeferred\(' 'Grab-authority proxy teardown must defer native release through retireDeferred.'
Reject-Text 'src/physics-interaction/hand/HandBoneColliderSet.cpp' 'instance\.body\.destroy\(|palmAnchorBody\.destroy\(' 'Hand bone colliders must not immediately destroy native collision objects in the live-world teardown path.'
Reject-Text 'src/physics-interaction/body/BodyBoneColliderSet.cpp' 'instance\.body\.destroy\(' 'Body bone colliders must not immediately destroy native collision objects in the live-world teardown path.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' '_grabAuthorityProxy\.destroy\(' 'Grab-authority proxy must not immediately destroy its native collision object in the live-world teardown path.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.h' 'struct PendingGeneratedWeaponBuild' 'Weapon collision must track staged generated body creation explicitly.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'beginPendingGeneratedWeaponBuild\(' 'Generated weapon collision must queue full source sets before frame-sliced body creation.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'createGeneratedWeaponBodiesInBankSlice\(' 'Generated weapon collision must create native bodies through a bounded per-frame slice.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.h' 'struct SupportGripEvidenceView[\s\S]*std::span<const TriangleData> localTriangles[\s\S]*RE::NiTransform localToWorld[\s\S]*weaponGenerationKey' 'Two-hand support grip evidence must expose a frame-scoped local triangle view with its current transform and generation.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.h' 'tryGetSupportGripEvidenceView\(\s*std::uint32_t bodyId,\s*const RE::NiAVObject\* currentWeaponRoot,\s*SupportGripEvidenceView& outView\) const' 'Two-hand support grip evidence must accept the current weapon root without copying cached triangles into world space.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'tryGetSupportGripEvidenceView\(decision\.bodyId,\s*weaponNode,\s*evidenceView\)' 'Two-hand support grip must reuse the contacted collider part cache through the current weapon root.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.h' 'tryFindCurrentWeaponSurfaceNearPoint\(\s*const RE::NiAVObject\* currentWeaponRoot,\s*const RE::NiPoint3& pointWorld,\s*float maxDistanceGameUnits,\s*WeaponSurfaceProximityWitness& outWitness\) const' 'Weapon collision must expose a generation-tagged current-surface witness query for authored support seats.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'tryFindCurrentWeaponSurfaceNearPoint\([\s\S]*currentGeneration = getCurrentWeaponGenerationKey\(\)[\s\S]*activeWeaponBodies\(\)[\s\S]*generatedSourceLocalTrianglesGame[\s\S]*generatedLocalTrianglesGame[\s\S]*pointAabbDistanceSquared\([\s\S]*closestPointOnTriangleToPoint\([\s\S]*outWitness\.weaponGenerationKey = currentGeneration' 'Authored-seat validation must broadphase against the current body bank, confirm exact cached-triangle proximity, and publish the matching weapon generation.'
Reject-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'extractAllTriangles\(sourceRoot,\s*triangles\)' 'Two-hand support grip startup must not rescan the live weapon mesh.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'Weapon visual node absent for unchanged equipped identity - retaining generated weapon bodies' 'Reload-time missing weapon visuals must retain same-identity generated bodies.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'Weapon visual node absent while rebuild required[\s\S]{0,700}destroyWeaponBody\(world\)' 'Missing weapon visuals must still destroy stale generated bodies when identity or safety rebuild gates require it.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'pendingGeneratedWeaponBuildMatches\(observedKey,\s*observedOwnershipKey,\s*observedFormID\)' 'Pending generated weapon creation must match equipped identity, ownership, and form, not reload visual witnesses.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.h' 'requestWorkbenchExitRebuild\(\)' 'Weapon collision must expose an explicit one-shot workbench-exit rebuild request.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' '_workbenchExitRebuildRequested\.store\(true' 'Workbench-exit rebuild requests must be queued as explicit state, not inferred from visual churn.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'weaponNode\s*!=\s*nullptr\s*&&\s*_workbenchExitRebuildRequested\.exchange\(false' 'Workbench-exit rebuild requests must only be consumed when a drawn weapon visual is available.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'rebuildRequired\s*=\s*driveRequestedRebuild\s*\|\|\s*workbenchExitRequested\s*\|\|\s*settingsChanged\s*\|\|\s*keyChanged\s*\|\|\s*missingBodies' 'Workbench exit must be an explicit rebuild gate alongside equip/settings/drive/missing-body gates.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'pendingInvalidated\s*=\s*driveRequestedRebuild\s*\|\|\s*workbenchExitRequested\s*\|\|[\s\S]{0,120}!pendingGeneratedWeaponBuildMatches\(observedKey,\s*observedOwnershipKey,\s*observedFormID\)' 'A consumed workbench-exit request must restart a matching staged create rather than silently keeping old sources.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'WeaponCollisionWorkbenchExitMenuSink' 'PhysicsInteraction must own the UI menu close sink that arms the workbench-exit rebuild request.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'requestWeaponCollisionRebuildAfterWorkbenchExit\(event\.menuName\.c_str\(\)\)' 'Workbench-family menu close must arm the weapon collision rebuild gate through PhysicsInteraction.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'ensureWeaponCollisionWorkbenchExitMenuSinkRegistered\(\);[\s\S]{0,180}const auto& runtime = runtime_state::currentFrame\(\)' 'Workbench-exit menu sink registration must retry from update before normal runtime early-outs.'
Reject-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'WeaponsWorkbenchExited|WorkbenchMenuBase::vfunction4' 'ROCK must not add a raw workbench ProcessMessage hook for this rebuild gate.'
Reject-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'CollisionSuppressionOwner::WeaponDominantHand' 'WeaponCollision must not acquire/release dominant-hand suppression; PhysicsInteraction owns the complete generated hand-collider set.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'retainedWeaponCollisionActive[\s\S]{0,140}_weaponCollision\.hasWeaponBody\(\)[\s\S]{0,140}_weaponCollision\.getCurrentWeaponGenerationKey\(\)\s*!=\s*0' 'Dominant weapon authority must include retained generated weapon bodies, not only a live weapon visual node.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'rightHandWeaponAuthorityActive\s*=\s*rightHandWeaponEquipped\s*\|\|\s*retainedWeaponCollisionActive' 'Right-hand weapon suppression must stay active across reload-null visual frames with retained weapon bodies.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'if \(rightHandWeaponAuthorityActive\)[\s\S]{0,120}suppressRightHandCollisionForDominantWeapon\(hknp\)' 'PhysicsInteraction must suppress dominant-hand collision while retained weapon bodies are active.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'synchronizeContactEvidenceOwnership\(rightHandWeaponAuthorityActive,\s*leftSupportGripActive,\s*rightPartGripActive\)' 'Contact-evidence ownership must follow retained weapon authority and part-grip drivers.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' '_dynamicHandCollision\.updateFrame\([\s\S]{0,220}rightHandWeaponAuthorityActive' 'Dynamic hand visual ownership must follow retained weapon authority.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'retainedPackageRootStillCurrent[\s\S]{0,220}visualSourceMissRetainFrameLimit[\s\S]{0,220}canRetainCurrentWeaponBodiesForVisualSourceMiss' 'Same-identity visual-only source misses must retain live generated weapon bodies only when the retained root is current and the retain window is bounded.'
Require-Text 'src/physics-interaction/performance/PerformanceProfiler.cpp' 'weaponRebuildVisualSourceUnavailableRetained' 'Profiler counters must expose retained same-identity visual source misses for runtime sampling.'
Require-Text 'src/physics-interaction/performance/PerformanceProfiler.cpp' 'weaponRebuildVisualSourceUnavailableRetainExpired' 'Profiler counters must expose same-identity visual source retain-window expiration for runtime sampling.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'if \(fallbackWeaponNode\)[\s\S]{0,80}return fallbackWeaponNode;' 'Live weapon collision motion/probe paths must prefer the current weapon root over cached body drive roots.'
Reject-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'safeNodeName\(instance\.driveNode\)' 'Weapon collision must not dereference cached body drive roots for mismatch logging.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'Generated weapon collision is now geometry-first' 'Generated weapon source extraction must document the visible-geometry-first source scan.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'candidate roots are[\s\S]{0,120}discovery witnesses' 'Generated weapon source extraction must treat candidate roots as merged witnesses, not competing winners.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'duplicate TriShape already claimed by earlier candidate' 'Merged candidate roots must dedupe overlapping source TriShapes.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'TriShape is hidden or locally zero-scale' 'Hidden TriShapes must be skipped as sources without pruning helper-node children.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'if \(node->GetAppCulled\(\)\)[\s\S]{0,300}ancestor branch is app-culled[\s\S]{0,120}return;' 'App-culled attachment branches must be pruned before their locally visible descendant meshes consume collider capacity.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'selectBalancedHullIndices\(selectionInputs,\s*MAX_WEAPON_BODIES\)[\s\S]{0,700}balanced-semantic-coverage' 'Generated weapon overflow must preserve balanced semantic part coverage instead of truncating by traversal order.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'pointCloudCanBuildHull\(shapePoints,\s*shapePointScale\)' 'Source-local hull validation must include the authored source-node scale used by native shape construction.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'durableAttachmentEvidence[\s\S]{0,180}!weapon_generated_source_completeness_policy::isTransientReloadPart[\s\S]{0,300}evidenceSourceAddresses\.insert' 'OMOD coverage must not accept cartridge or cosmetic-ammo bodies as proof of an attachment collider.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.h' 'getCurrentEquippedWeaponOwnershipKey\(\)\s+const\s*\{\s*return\s+_observedEquippedWeaponOwnershipKey' 'Manual weapon ownership must use an instance-bound witness separately from collision content identity.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'preserving manual ownership across collision rebuild[\s\S]*?if\s*\(grip\.active\s*&&\s*!tryRebindPartGripToCurrentGeneration[\s\S]*?return false;' 'Two-handed ownership must fail closed when an active part cannot rebind to the published generation.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'beginPrimaryOnlyGrip\(\s*weaponNode,\s*currentWeaponGenerationKey,\s*currentEquippedWeaponOwnershipKey\s*,' 'Primary-only ownership must preserve a real zero collision generation while colliders build.'
Reject-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'primaryOwnershipGenerationKey|currentWeaponGenerationKey\s*!=\s*0\s*\?\s*currentWeaponGenerationKey\s*:\s*currentEquippedWeapon' 'Equipped ownership identity must never masquerade as a collision generation.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' '_state\s*!=\s*TwoHandedState::PrimaryOnly[\s\S]{0,320}if\s*\(currentWeaponGenerationKey\s*==\s*0\)[\s\S]{0,500}return\s+true;' 'Only PrimaryOnly ownership may survive the pre-publication collider window.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' '_equippedWeaponMenuReconcilePending[\s\S]*?isRawButtonPhysicallyHeld[\s\S]*?_pendingEquippedWeaponPrimaryOnlyGripStart' 'Menu exit must re-arm equipped weapon ownership only when the physical primary grab remains held.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'debouncePrimaryGripRelease' 'Primary firing-grip release must reject transient one-frame open samples.'

Reject-TextInTree 'src/physics-interaction/weapon' @('*.cpp', '*.h') '\b(?:QueueAttachWeapon|AttachModToReference|AttachWeapon|HandleItemEquip|SetEquippedItem|RequestLoadAnimationsForWeaponChange|QueueUpdate3D|QueueShow1stPerson|Update1StPersonArm)\b' 'Weapon collision module must not call native visual refresh/equip/remap paths.'
Reject-TextInTree 'src/physics-interaction/weapon' @('*.cpp', '*.h') '\b(?:WeaponVisualRemapRuntime|WeaponInstanceWitnessRuntime|requestCurrentFirstPersonWeaponVisualRemap|tryGetAuthoritativeEquippedWeaponIdentity|nativeVisualRemapAllowedForWitness|clearAuthoritativeEquippedWeaponWitness)\b' 'Weapon collision module must not retain native remap or instance-witness plumbing.'
Reject-TextInTree 'src/physics-interaction/weapon' @('*.cpp', '*.h') '\b(?:_pendingWeaponKey|_pendingEquippedWeaponOwnerKey|_cachedEquippedWeaponOwnerKey|_cachedEquippedWeaponInstanceWitness|_pendingEquippedWeaponInstanceWitness|_cachedGeneratedWeaponVisualWitness|_pendingGeneratedWeaponVisualWitness|_visualSettleState|_generatedSourceSettleState|_weaponBodyPending|_weaponBodiesDisabledForMissingVisual|_retryCounter|_pendingGeneratedRebuildAttemptRequested)\b' 'Weapon collision must not retain pending/settle/witness shadow state.'
Reject-TextInTree 'src/physics-interaction/weapon' @('*.cpp', '*.h') '\b(?:disableWeaponBodiesForMissingVisual|shouldReenableCachedBodiesForReturnedVisual|requestGeneratedSourceCompletenessProbeNextFrame|requestPendingGeneratedRebuildAttemptNextFrame|armGeneratedSourceCompletenessProbe|tryEnrichGeneratedWeaponBodiesFromLateSources|makeGeneratedWeaponVisualWitness)\b' 'Weapon collision must not retain refresh/probe/late-source lifecycle helpers.'
Reject-TextInTree 'src/physics-interaction/weapon' @('*.cpp', '*.h') '\b(?:evaluateEquippedVisualMissing|evaluateGeneratedCollisionPendingTransition|evaluateReturnedCachedVisualPending|sameFormEquippedInstanceRemapWitnessChanged|evaluateNativeVisualRemap|NativeVisualRemapAttemptState|InstanceVisualSyncDecision|GeneratedSourceSettleState|VisualSettleState|makeGeneratedSourcePendingSettleKey|makeGeneratedSourceReplacementSettleKey|evaluateGeneratedSourceReplacement|sourceSetImproved)\b' 'Weapon authority policy must not retain removed refresh, pending, settle, remap, or late-enrichment helpers.'
Reject-Text 'src/ROCKMain.cpp' 'weapon_instance_witness_runtime|WeaponInstanceWitnessRuntime' 'ROCK startup must not install removed weapon instance witness hooks.'
Reject-Text 'src/RockConfig.h' 'rockWeaponCollisionNativeVisualRemapEnabled' 'ROCK config must not expose removed native visual remap option.'
Reject-Text 'src/RockConfig.cpp' 'bWeaponCollisionNativeVisualRemapEnabled|rockWeaponCollisionNativeVisualRemapEnabled' 'ROCK config loader must not read removed native visual remap option.'

Reject-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'deferGenerationForWeaponVisualRefresh' 'Initial weapon collision generation must not defer through a visual-refresh state machine.'
Reject-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'advanceWeaponVisualRefreshFrame\(\);' 'Generated weapon collision must not retain visual-refresh cooldown advancement.'
Reject-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'generatedWeaponSourceMissingRequiredPackageCoverage' 'Firearm source scans must not use required front/rear package coverage as a generation gate.'
Reject-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'sourcePackageNeedsMoreCoverage' 'Runtime generated collision must not block or retry visible weapon sources for semantic package coverage.'

if ($failures.Count -gt 0) {
    Write-Host 'Weapon collision lifecycle source boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Weapon collision lifecycle source boundary passed.'
