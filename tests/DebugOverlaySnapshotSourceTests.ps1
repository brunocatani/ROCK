param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$failures = [System.Collections.Generic.List[string]]::new()
$relativePath = 'src/physics-interaction/debug/DebugBodyOverlay.cpp'
$path = Join-Path $Root $relativePath
$text = Get-Content -Raw -LiteralPath $path
$interactionText = Get-Content -Raw -LiteralPath (
    Join-Path $Root 'src/physics-interaction/core/PhysicsInteraction.cpp')
$captureFunction = [regex]::Match(
    $text,
    'void CaptureSolvedBodyTransformsFromPhysicsStep[\s\S]*?(?=\r?\n\s*void ClearFrame\()').Value

function Require-Pattern {
    param([string]$Pattern, [string]$Message)
    if ($text -notmatch $Pattern) {
        $failures.Add($Message)
    }
}

function Reject-Pattern {
    param([string]$Pattern, [string]$Message)
    if ($text -match $Pattern) {
        $failures.Add($Message)
    }
}

Require-Pattern 'DebugOverlaySnapshotPool\.h' `
    'Overlay publication must use the bounded, tested immutable-snapshot pool.'
Require-Pattern 'std::atomic<std::shared_ptr<const PublishedOverlayFrame>>\s+s_publishedFrame' `
    'The compositor boundary must publish an immutable atomic shared_ptr.'
Require-Pattern 'std::atomic<std::shared_ptr<const PublishedOverlayFrame>>\s+s_pendingSolvedFrame' `
    'A body-backed logical frame must remain private until final-solve promotion.'
Require-Pattern 'DebugOverlayLatestSnapshot\.h' `
    'The post-physics handoff must use the fixed lock-free snapshot exchange.'
Require-Pattern 'LatestSnapshot<AppliedBodyTransformFrame,\s*kPublishedFramePoolCapacity>\s+s_appliedBodyTransforms' `
    'Post-physics body matrices must cross into Submit through bounded fixed slots.'
Require-Pattern 'LatestSnapshot<SolvedBodyCaptureRequestFrame,\s*kPublishedFramePoolCapacity>\s+s_solvedBodyCaptureRequests' `
    'The game-to-physics capture request must cross through bounded fixed slots.'
Require-Pattern 'CaptureSolvedBodyTransformsFromPhysicsStep[\s\S]*s_solvedBodyCaptureRequests\.tryAcquire\(\)' `
    'The native physics callback must acquire fixed capture metadata.'
Require-Pattern 'CaptureSolvedBodyTransformsFromPhysicsStep[\s\S]*s_appliedBodyTransforms\.tryBeginWrite\(\)' `
    'The native physics callback must acquire an allocation-free fixed write slot.'
Require-Pattern 'CaptureSolvedBodyTransformsFromPhysicsStep[\s\S]*extractBody\([\s\S]*request\.frameSource' `
    'The post-drive publication must preserve each body or axis BODY/MOTION frame convention.'
Require-Pattern 'for \(const auto& published : next->bodies\)[\s\S]*appendSolvedBodyCaptureRequest\([\s\S]*published\.frameSource[\s\S]*for \(const auto& published : next->axes\)[\s\S]*AxisOverlaySource::Body[\s\S]*targetAxisOverlayFrameSource\(published\.entry\.role\)' `
    'Solved capture must cover every drawn collider and every body-backed axis.'
Require-Pattern 's_pendingSolvedFrame\.store\([\s\S]*captureRequest\.publish\(\)[\s\S]*return;[\s\S]*s_publishedFrame\.store' `
    'A body-backed frame must remain pending while only body-free frames publish immediately.'
Require-Pattern 'next\.publish\(\)[\s\S]*s_pendingSolvedFrame\.compare_exchange_strong\([\s\S]*s_publishedFrame\.store\(pending[\s\S]*s_frameAdmission\.publish\(\)' `
    'Final solve must publish transforms, claim the exact pending frame, then admit the immutable pair.'
Require-Pattern 'appliedTransformOwner->publicationSequence == frame->publicationSequence[\s\S]*appliedTransformOwner->worldIdentity == frame->worldIdentity' `
    'Submit must reject applied matrices from a different logical frame or Havok world.'
Require-Pattern 'frame->requiresSolvedBodyTransforms\s*&&\s*!appliedTransforms[\s\S]*return;' `
    'Submit must fail closed rather than draw pre-solve fallbacks for body-backed frames.'
Require-Pattern 's_publishedFrame\.load\(std::memory_order_acquire\)' `
    'The compositor must acquire one immutable frame snapshot.'
Require-Pattern 's_publishedFrame\.store\([\s\S]*std::memory_order_release\)' `
    'Frame publication must use release ordering.'
Require-Pattern 'buildPublishedFrame[\s\S]*extractBody\(source\.world' `
    'Live Havok body state must be resolved while building the game-thread publication.'
Require-Pattern 'buildPublishedFrame[\s\S]*captureBodyWorldAabb\(source\.world' `
    'The publication must capture the real body AABB before crossing into the compositor thread.'
Require-Pattern 'world->GetBodyAabb\(bodyId,\s*&raw\)' `
    'Body bounds must use the current CommonLibF4VR world wrapper instead of imported raw standalone offsets.'
Require-Pattern 'captureOverlayRenderSettings[\s\S]*g_rockConfig' `
    'Mutable overlay settings must be captured into the publication.'
Require-Pattern 'std::shared_ptr<const GpuShape>\s+shapeOwner' `
    'A render lookup must retain immutable GPU buffers across concurrent cache invalidation.'

if ($interactionText -notmatch '_dynamicHandCollision\.samplePostSolveDeviations\([\s\S]{0,1800}timing\.substepIndex\s*\+\s*1\s*>=\s*timing\.substepCount[\s\S]{0,500}debug::CaptureSolvedBodyTransformsFromPhysicsStep\(world\);') {
    $failures.Add('Solved collider matrices must publish after the final body solve, never from an intermediate substep.')
}
if ($interactionText -match '_dynamicHandCollision\.flushPendingPhysicsDrive\(world, timing\);[\s\S]{0,900}debug::CaptureSolvedBodyTransformsFromPhysicsStep\(world\);') {
    $failures.Add('Pre-collide must not publish the previous solved body matrices as current.')
}
if ([string]::IsNullOrWhiteSpace($captureFunction) -or
    $captureFunction -match 's_publishedFrame\.(load|exchange)') {
    $failures.Add('The native physics callback must not acquire the shared_ptr render publication.')
}

Reject-Pattern 's_frameMutex' `
    'The compositor must not contend on the retired full-frame mutex.'
Reject-Pattern 'frame\s*=\s*s_frame' `
    'The compositor must not copy the full logical frame.'
Reject-Pattern 'kBodyAabb16|kAabbDecompressOffset|kAabbDecompressScale' `
    'ROCK must not import the standalone visualizer raw AABB offsets.'
Reject-Pattern 'reinterpret_cast<const\s+std::int16_t\s*\*>' `
    'Compressed hknp body AABB components are unsigned; the signed standalone decoder is forbidden.'

$drawMatch = [regex]::Match(
    $text,
    'void\s+drawOverlayToSubmittedTexture\([^\)]*\)\s*\{(?<body>[\s\S]*?)\n\s*void\s+reportOverlayExceptionOnce')
if (-not $drawMatch.Success) {
    $failures.Add('Could not isolate the compositor draw function for boundary checks.')
} else {
    $drawBody = $drawMatch.Groups['body'].Value
    foreach ($forbidden in @('g_rockConfig', 'extractBody(', 'makeShapeKey(', 'generateShape(', 'computeShapeGeometryFingerprint(')) {
        if ($drawBody.Contains($forbidden)) {
            $failures.Add("Compositor draw must not access '$forbidden'.")
        }
    }
}

if ($failures.Count -gt 0) {
    Write-Host 'DebugOverlaySnapshotSourceTests failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'DebugOverlaySnapshotSourceTests passed.' -ForegroundColor Green
