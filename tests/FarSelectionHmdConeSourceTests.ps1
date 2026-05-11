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

Require-Text 'src/physics-interaction/hand/HandSelection.h' 'struct\s+FarSelectionHmdConeGate' 'Far HMD cone gate must be an explicit frame input, not an object-detection global read.'
Require-Text 'src/physics-interaction/hand/HandSelection.h' 'isPointInsideFarHmdCone' 'HMD cone math must stay in pure selection policy for unit coverage.'
Require-Text 'src/physics-interaction/core/PhysicsFrameContext.h' 'hasHmdFrame[\s\S]*hmdPositionWorld[\s\S]*hmdForwardWorld' 'Frame context must carry a coherent HMD snapshot.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionFrame.inl' 'HmdNode[\s\S]*rotate\.Transpose\(\)\s*\*\s*RE::NiPoint3\(0\.0f,\s*1\.0f,\s*0\.0f\)' 'HMD forward must use the FO4VR/CommonFramework Y-forward node convention.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'makeFarSelectionHmdConeGate\(frame\)' 'Selection and grab input must share the same frame HMD cone gate.'
Require-Text 'src/physics-interaction/object/ObjectDetection.cpp' 'promotesFarHitToCloseSelection[\s\S]*shouldGateFarCandidateWithHmdCone\(isFarSelection,\s*promotesToCloseSelection\)[\s\S]*acceptsHitPoint\(hitPoint,\s*&hmdConeDot\)[\s\S]*\+\+outRejectedHmdCone' 'Object detection must only reject true far candidates outside the HMD cone after near-promotion eligibility is known.'
Require-Text 'src/physics-interaction/object/ObjectDetection.cpp' 'chooseShapeCastSelection\(bhkWorld,\s*hknpWorld,\s*palmPos[\s\S]*false[\s\S]*nullptr' 'Close selection must not receive the HMD cone gate.'
Require-Text 'src/physics-interaction/object/ObjectDetection.h' 'resolveFarSelectionHmdConeAnchor[\s\S]*tryResolveLiveBodyWorldTransform[\s\S]*selection\.hasHitPoint' 'Far HMD cone revalidation must prefer live body/node anchors before falling back to the original hit point.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'farSelectionQueryReady[\s\S]*!farHmdConeGate\.enabled\s*\|\|\s*farHmdConeGate\.hasHmdFrame[\s\S]*_farDetectCounter\s*=\s*0' 'Missing HMD frame must skip far queries instead of running casts that can only fail closed.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' '_cachedFarCandidate\.clear\(\)[\s\S]*selectedObjectPassesFarHmdCone\(hknpWorld,\s*farCandidate,\s*farHmdConeGate\)' 'Cached far candidates must be invalidated against live HMD cone anchors when the current cone no longer accepts them.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'cleared far selection outside HMD cone' 'SelectedFar highlights must clear when head intent no longer matches the far target.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'far grab blocked outside HMD cone' 'Dynamic pull startup must revalidate the HMD cone before locking or actor-equipment drop handoff.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'far grab press ignored outside HMD cone[\s\S]*grab_input_intent_policy::reset\(inputIntentState\)' 'The backup grab-start gate must reject stale far selections before consuming a latched grab edge.'
Require-Text 'tests/TransformConventionTests.cpp' 'far hmd cone rejects side hit' 'Pure tests must cover side-hit rejection.'
Require-Text 'tests/TransformConventionTests.cpp' 'far hmd cone disabled passes without frame' 'Pure tests must cover the disabled-gate compatibility path.'
Require-Text 'tests/TransformConventionTests.cpp' 'far hmd cone skips near-promoted candidates' 'Pure tests must cover the near-promotion bypass before HMD gating.'

if ($failures.Count -gt 0) {
    Write-Host 'Far selection HMD cone source test failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Far selection HMD cone source test passed.'
