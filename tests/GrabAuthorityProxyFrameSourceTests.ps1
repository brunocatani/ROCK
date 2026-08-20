param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$failures = [System.Collections.Generic.List[string]]::new()

function Require-Text {
    param(
        [string]$RelativePath,
        [string]$Pattern,
        [string]$Message
    )

    $path = Join-Path $Root $RelativePath
    $text = Get-Content -Raw -LiteralPath $path
    if ($text -notmatch $Pattern) {
        $failures.Add("$RelativePath`: $Message")
    }
}

function Reject-Text {
    param(
        [string]$RelativePath,
        [string]$Pattern,
        [string]$Message
    )

    $path = Join-Path $Root $RelativePath
    $text = Get-Content -Raw -LiteralPath $path
    if ($text -match $Pattern) {
        $failures.Add("$RelativePath`: $Message")
    }
}

Require-Text 'src/physics-interaction/hand/Hand.h' 'struct\s+LivePalmAnchorReference' 'Dynamic grab must have an explicit live palm-anchor reference type.'
Require-Text 'src/physics-interaction/hand/Hand.h' 'tryResolveLivePalmAnchorReference' 'Dynamic grab must resolve the actual palm anchor body as the hand-side reference.'
Require-Text 'src/physics-interaction/hand/Hand.h' 'enum class\s+GrabAuthorityProxyFramePolicy[\s\S]*LivePalmOnly[\s\S]*PreferQueuedPalmTarget' 'Grab proxy frame resolution must expose an explicit live-vs-queued authority policy.'
Require-Text 'src/physics-interaction/hand/Hand.h' 'struct\s+GrabAuthorityProxyDebugSnapshot' 'Debug overlay must expose a grab authority proxy target snapshot that is not dependent on an active grab.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'Hand::tryResolveLivePalmAnchorReference[\s\S]*tryResolveLiveBodyWorldTransform\(world,\s*_handBody\.getBodyId\(\)' 'Live palm authority must read the actual palm-anchor hknp body frame.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'Hand::getGrabAuthorityProxyDebugSnapshot\(RE::hknpWorld\* world,\s*const RE::NiTransform& rawHandWorld[\s\S]*tryResolveLivePalmAnchorReference\(world,\s*palmReference\)[\s\S]*generatedColliderFrameToGrabAuthorityFrame\(palmReference\.world\)[\s\S]*applyGrabAuthorityProxyLocalOffsetToFrame\(palmAuthorityBaseWorld,\s*_isLeft\)' 'Idle proxy debug target must show the generated/proxy-local seat point without changing the runtime proxy drive frame.'
Reject-Text 'src/physics-interaction/hand/Hand.cpp' 'applyGrabAuthorityProxyLocalOffsetToFrame\(rawHandWorldTransform,\s*_isLeft\)' 'Startup fallback must not apply the generated/proxy local offset converter to raw controller space.'
Require-Text 'src/physics-interaction/hand/Hand.h' 'tryComputeGrabProxyLocalPalmPocketPivotAWorld' 'Proxy-local palm-pocket pivot computation must be an explicit helper so held paths do not reuse generated proxy translation.'
Require-Text 'src/RockConfig.h' 'rockRightGrabAuthorityProxyOffsetGameUnits' 'RockConfig must expose the right-hand hidden proxy offset.'
Require-Text 'src/RockConfig.h' 'rockLeftGrabAuthorityProxyOffsetGameUnits' 'RockConfig must expose the left-hand hidden proxy offset.'
Require-Text 'src/RockConfig.cpp' 'fRightGrabAuthorityProxyOffsetXGameUnits' 'RockConfig must read the right-hand hidden proxy offset from INI.'
Require-Text 'src/RockConfig.cpp' 'fLeftGrabAuthorityProxyOffsetXGameUnits' 'RockConfig must read the left-hand hidden proxy offset from INI.'
Require-Text 'src/physics-interaction/weapon/LooseWeaponGripZone.cpp' 'weapon->weaponData\.type\s*==\s*RE::WEAPON_TYPE::kGrenade[\s\S]*weapon->weaponData\.type\s*==\s*RE::WEAPON_TYPE::kMine[\s\S]*throwableSkipsFrikOffset[\s\S]*frik_weapon_offset_cache::findPrimaryWeaponOffset' 'Throwable loose weapon grip-zone projection must bail before consulting FRIK primary offsets.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'Hand::computeGrabPivotAWorld[\s\S]*tryComputeGrabProxyLocalPalmPocketFrameWorld\(world,\s*proxyFrameWorld\)[\s\S]*return\s+proxyFrameWorld\.translate' 'Pivot-A fallback helper must use the shared generated/proxy palm-pocket frame before falling back to raw translation.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'Hand::tryComputeGrabProxyLocalPalmPocketFrameWorld[\s\S]*generatedColliderFrameToGrabAuthorityFrame\(palmReference\.world\)[\s\S]*applyGrabAuthorityProxyLocalOffsetToFrame\(palmAuthorityBaseWorld,\s*_isLeft\)' 'Proxy-local palm-pocket frame capture must apply the configured offset through generated/proxy local space.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'Hand::tryComputeGrabProxyLocalPalmPocketPivotAWorld[\s\S]*tryComputeGrabProxyLocalPalmPocketFrameWorld\(world,\s*proxyFrameWorld\)[\s\S]*outPivotWorld\s*=\s*proxyFrameWorld\.translate' 'Proxy-local palm-pocket pivot capture must delegate to the shared generated/proxy frame helper.'
Reject-Text 'src/physics-interaction/hand/Hand.cpp' 'computeGrabStartupCapturePivotAWorld' 'Committed grab capture must not retain a raw-fallback startup pivot helper.'
Require-Text 'src/physics-interaction/grab/GrabCore.h' 'desiredBodyWorld\s*=\s*input\.hasDesiredBodyWorld\s*\?\s*input\.desiredBodyWorld\s*:[\s\S]*shiftObjectToAlignGripWithPocket\(\s*input\.bodyWorld,\s*input\.pivotAWorld,\s*input\.gripPointWorld\)' 'Proxy authority must derive the solver BODY target from explicit BODY authority or BODY-frame pocket alignment, not from the visual object frame.'
Reject-Text 'src/physics-interaction/grab/GrabCore.h' 'desiredBodyWorld\s*=\s*transform_math::composeTransforms\(frozen\.desiredObjectWorld,\s*frozen\.bodyLocal\)' 'Proxy authority must not let the visual object relation define the solver BODY target.'
Require-Text 'src/physics-interaction/grab/GrabConstraint.cpp' 'setGrabMotorAtomsActive\(header,\s*true,\s*true\)' 'Proxy angular authority must always enable the ragdoll atom motor.'
Require-Text 'src/physics-interaction/grab/GrabCore.h' 'objectInGeneratedProxyLocalSpace[\s\S]*generatedColliderWorldPointToLocal\(proxyWorld,\s*objectWorld\.translate\)[\s\S]*generatedColliderWorldVectorToLocal\(proxyRotationWorld,\s*objectAxisXWorld\)' 'Proxy authority must convert BODY/object relations through the same stored-column generated collider local-space boundary used by pivot A.'
Require-Text 'src/physics-interaction/grab/GrabCore.h' 'objectFromGeneratedProxyLocalSpace[\s\S]*generatedColliderLocalVectorToWorld\(proxyWorld,\s*objectProxyLocal\.translate\)[\s\S]*generatedColliderLocalVectorToWorld\(proxyRotationWorld,\s*objectAxisXProxyLocal\)' 'Proxy authority must replay BODY/object relations through the same stored-column generated collider world-space boundary used by generated bodies.'
Require-Text 'src/physics-interaction/grab/GrabCore.h' 'proxyAuthorityBodyHandSpace\s*=[\s\S]*objectInGeneratedProxyLocalSpace\(input\.proxyWorld,\s*frozen\.desiredBodyWorld\)[\s\S]*frozen\.desiredBodyWorld\s*=[\s\S]*objectFromGeneratedProxyLocalSpace\(input\.proxyWorld,\s*frozen\.proxyAuthorityBodyHandSpace\)' 'Grab capture must freeze and replay BODY authority through generated/proxy collider-local space, not plain row-axis inverse math.'
Require-Text 'src/RockConfig.h' 'rockDebugGrabTimelineTrace[\s\S]*rockDebugGrabTimelineTraceIntervalFrames' 'RockConfig must expose structured grab timeline tracing and interval control.'
Require-Text 'src/RockConfig.cpp' 'bGrabTimeline[\s\S]*iGrabTimelineIntervalFrames' 'RockConfig must read structured grab timeline tracing from INI.'
Reject-Text 'src/RockConfig.h' 'rockGrabRagdollDecompositionMode' 'The retired ragdoll decomposition mode config must stay removed; the aligned write is the only convention (binary-verified + in-game validated 2026-07-13).'
Reject-Text 'src/RockConfig.cpp' 'iGrabRagdollDecompositionMode' 'The retired iGrabRagdollDecompositionMode INI key must stay removed.'
Reject-Text 'src/physics-interaction/grab/GrabConstraint.cpp' 'resolveGrabRagdollDecompositionMode|ragdollDecomposition' 'Grab constraint creation must not resurrect the retired decomposition mode selector.'
Reject-Text 'src/physics-interaction/grab/GrabConstraint.h' 'ragdollDecomposition' 'Active constraints must not carry retired decomposition mode state.'
Require-Text 'src/physics-interaction/grab/GrabCore.h' 'traceId[\s\S]*traceTargetWriteSequence' 'Canonical grab frame must carry a stable trace id and target-write sequence for timeline correlation.'
Reject-Text 'src/physics-interaction/grab/GrabConstraintMath.h' 'resolveGrabRagdollDecompositionMode|kGrabRagdollDecompositionMode|kGrabRagdollDecompositionAutoThresholdDegrees|computeGrabRagdollDecompositionColumnDeltaDegrees' 'The retired COL-90 decomposition mode selector must stay removed; the aligned write is the single convention.'
Require-Text 'src/physics-interaction/grab/GrabConstraintMath.h' 'writeGrabConstraintAngularDecomposition\(float\* transformBRotation,\s*float\* targetBRca,\s*const Transform& proxyInBody\)' 'FO4VR custom grab angular setup must centralize the single aligned transform-B/target write.'
Require-Text 'src/physics-interaction/grab/GrabConstraintMath.h' 'writeGrabConstraintHeldTargetAtoms\(\s*float\*\s*transformBRotation[\s\S]*writeGrabConstraintAngularDecomposition\(transformBRotation,\s*targetBRca,\s*proxyInBody\)[\s\S]*computeHiggsTransformBTranslationGameFromProxyInBody' 'FO4VR custom grab held updates must rewrite the aligned transform-B/target atoms plus the relation-derived transform-B translation.'
Require-Text 'src/physics-interaction/grab/GrabConstraintMath.h' 'writeGrabConstraintAngularDecomposition[\s\S]{0,900}writeHavokRotationRows\(transformBRotation,\s*proxyInBody\.rotate\);[\s\S]{0,200}writeHavokRotationRows\(targetBRca,\s*proxyInBody\.rotate\)' 'Transform-B and target_bRca must receive identical solver rows (2026-07-13 binary-verified zero-separation contract, in-game validated).'
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.cpp' 'drawGrabAuthorityProxy[\s\S]*getGrabAuthorityProxyDebugSnapshot\(hknp,\s*rawHandWorld,\s*snapshot\)[\s\S]*RightGrabAuthorityProxyTarget' 'Proxy overlay must draw the computed grab-start capture frame before a grab creates a readback proxy body.'
Require-Text 'src/physics-interaction/hand/Hand.h' 'tryGetPalmAnchorTarget\(RE::NiTransform& outTarget\)\s+const' 'Proxy debug overlay must be able to draw the live generated palm target before relying on body readback.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.cpp' 'addGrabAuthorityAxisReference[\s\S]*tryGetPalmAnchorTarget\(palmAnchorTarget\)[\s\S]*addStoredColumnAxisTransform\([\s\S]*RightGrabPalmGeneratedDirect' 'Proxy overlay must include the generated palm XYZ marker in the same view as the proxy target/readback markers.'
Reject-Text 'src/physics-interaction/grab/GrabConstraintMath.h' 'writeHavokRotationColumns' 'The column write helper is the disproven 2026-05-12 convention and must stay removed; both angular atoms take the solver-row write.'
Reject-Text 'src/physics-interaction/grab/GrabAuthorityProxyMotion.h' 'computeAngularVelocityRadiansPerSecond|matrixColumn|axisSum' 'The old manual matrix-column proxy angular helper must stay removed; proxy angular telemetry uses the FO4VR hard-keyframe helper only as fallback.'

Reject-Text 'src/physics-interaction/grab/GrabAuthoritySourceClockResampler.h' 'phaseSeconds|kMaxExtrapolationSourceIntervals|ResampleAction::Extrapolate' 'The physics-clock resampler machinery (phase accumulator, extrapolation window) must stay removed; the game-clock phase lock owns the target time base.'

if ($failures.Count -gt 0) {
    Write-Host 'Grab authority proxy frame source test failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Grab authority proxy frame source test passed.'
