param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$failures = [System.Collections.Generic.List[string]]::new()
$overlay = Get-Content -Raw -LiteralPath (Join-Path $Root 'src/physics-interaction/debug/DebugBodyOverlay.cpp')
$pipeline = Get-Content -Raw -LiteralPath (Join-Path $Root 'src/physics-interaction/debug/DebugOverlayShapePipeline.cpp')
$geometry = Get-Content -Raw -LiteralPath (Join-Path $Root 'src/physics-interaction/debug/DebugOverlayShapeGeometry.cpp')
$physics = Get-Content -Raw -LiteralPath (Join-Path $Root 'src/physics-interaction/core/PhysicsInteraction.cpp')

function Require-In {
    param([string]$Text, [string]$Pattern, [string]$Message)
    if ($Text -notmatch $Pattern) {
        $failures.Add($Message)
    }
}

function Reject-In {
    param([string]$Text, [string]$Pattern, [string]$Message)
    if ($Text -match $Pattern) {
        $failures.Add($Message)
    }
}

Require-In $overlay 'captureShapeRecipeUnsafe[\s\S]*GetNumberOfSupportVertices[\s\S]*GetSupportVertices' `
    'CommonLib shape reads must be confined to guarded publisher-side recipe capture.'
Require-In $overlay 'captureShapeRecipeSeh[\s\S]*captureShapeRecipeUnsafe[\s\S]*EXCEPTION_EXECUTE_HANDLER' `
    'Publisher-side engine recipe capture must fail closed on invalid engine memory.'
Require-In $overlay 'computeShapeGeometryFingerprintSeh[\s\S]*computeShapeGeometryFingerprintUnsafe[\s\S]*EXCEPTION_EXECUTE_HANDLER[\s\S]*makeShapeKey[\s\S]*computeShapeGeometryFingerprintSeh' `
    'Geometry identity must use the same fail-closed SEH boundary as recipe capture.'
Require-In $overlay 'shapePipeline\(\)\.reserve[\s\S]*captureShapeRecipeGuarded[\s\S]*shapePipeline\(\)\.submit' `
    'Queue/cache admission must happen before expensive recipe capture and submission.'
Require-In $overlay 'kScaledConvexScaleOffset\s*=\s*0x40[\s\S]*kScaledConvexTranslationOffset\s*=\s*0x50' `
    'Scaled-convex capture must use the FO4VR constructor-verified scale and translation offsets.'
Require-In $overlay 'kCompoundSlotArrayOffset\s*=\s*0x60[\s\S]*kCompoundSlotCountOffset\s*=\s*0x68[\s\S]*kCompoundSlotStride\s*=\s*0x80' `
    'Compound capture must use the independently verified FO4VR root and slot layout.'
Require-In $overlay 'case 4:[\s\S]*GetSupportVertices[\s\S]*Kind::Triangle' `
    'Triangle recipes must use the verified CommonLib support-vertex virtual and a dedicated triangle path.'
Require-In $overlay 'case 7:[\s\S]*case 8:[\s\S]*kCompoundSlotActiveOffset[\s\S]*captureShapeRecipeUnsafe\(childShapeAddress' `
    'Static and dynamic compound recipes must inspect active verified slots and recursively capture every active child.'
Require-In $geometry 'makeCompound[\s\S]*ShapeDecodeMode::Unsupported[\s\S]*appendMesh' `
    'Compound mesh construction must fail closed instead of publishing partial child geometry.'
Reject-In $overlay 'kScaledConvexScaleOffset\s*=\s*0x38|kScaledConvexTranslationOffset\s*=\s*0x48' `
    'The disputed standalone scaled-convex offsets must not return.'
Reject-In $overlay 'kCompoundSlotArrayOffset\s*=\s*0x58|kCompoundSlotCountOffset\s*=\s*0x60' `
    'The disputed standalone compound root offsets must not return.'
Require-In $overlay 'captureBodyWorldAabb' `
    'The publisher must capture a real engine body AABB.'
Require-In $overlay 'worldAabbMatrix[\s\S]*s_d3d\.aabbProxy' `
    'Missing, pending, and unsupported detail must retain a real captured-AABB proxy.'
Require-In $overlay 'processCompletedUploads\([\s\S]*maxShapeUploadsPerFrame' `
    'The compositor must enforce a per-rendered-frame upload budget.'
Require-In $pipeline 'THREAD_PRIORITY_BELOW_NORMAL' `
    'The owned mesh worker must run below normal priority.'
Require-In $pipeline 'completedJobs\.size\(\)\s*>=\s*job\.maxCompletedJobs' `
    'The completed CPU-mesh queue must have an enforced bound.'
Require-In $pipeline 'backlog\s*=\s*_impl->reservedJobs[\s\S]*backlog\s*>=\s*limits\.maxQueuedJobs' `
    'Reservations, queued, active, and completed work must share an enforced backlog bound.'
Require-In $pipeline 'completed\.generation\s*!=\s*_impl->generation|job\.generation\s*!=\s*generation' `
    'Stale worker/upload results must be rejected by generation token.'
Require-In $pipeline 'ensureGpuCapacityUnlocked[\s\S]*maxGpuBytes' `
    'GPU cache admission must enforce an approximate byte budget.'
Require-In $pipeline 'lastUseSerial[\s\S]*evictOneUnlocked' `
    'Ready and unsupported cache entries must use deterministic LRU eviction.'
Require-In $pipeline 'worker\.join\(\)' `
    'Shape worker shutdown must join deterministically.'
Require-In $physics 'debug::ShutdownShapePipeline\(\)' `
    'Physics teardown must stop and join the overlay worker before module destruction completes.'

$expectedCommonLibCalls = @{
    '->GetType\(\)' = 2
    '->GetNumberOfSupportVertices\(\)' = 2
    '->GetSupportVertices\(' = 2
}
foreach ($entry in $expectedCommonLibCalls.GetEnumerator()) {
    $actual = [regex]::Matches($overlay, $entry.Key).Count
    if ($actual -ne $entry.Value) {
        $failures.Add("Expected exactly $($entry.Value) guarded overlay calls matching '$($entry.Key)', found $actual.")
    }
}

$worker = [regex]::Match($pipeline, 'void\s+workerMain\(\)\s*\{(?<body>[\s\S]*?)\n\s*\}\n\n\s*void\s+clearStateUnlocked')
if (-not $worker.Success) {
    $failures.Add('Could not isolate the shape worker for boundary checks.')
} else {
    Reject-In $worker.Groups['body'].Value '\bRE::|hknp|ID3D11|CreateBuffer|GetSupportVertices|GetType\(' `
        'The shape worker must not dereference engine memory or call D3D.'
}

$draw = [regex]::Match($overlay, 'void\s+drawOverlayToSubmittedTexture\([^\)]*\)\s*\{(?<body>[\s\S]*?)\n\s*void\s+reportOverlayExceptionOnce')
if (-not $draw.Success) {
    $failures.Add('Could not isolate the compositor draw function.')
} else {
    Reject-In $draw.Groups['body'].Value 'captureShapeRecipe|GetSupportVertices|GetNumberOfSupportVertices|computeShapeGeometryFingerprint|buildMeshFromRecipe' `
        'The compositor must not capture, fingerprint, or build CPU shape geometry.'
}

if ($failures.Count -gt 0) {
    Write-Host 'DebugOverlayShapePipelineSourceTests failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'DebugOverlayShapePipelineSourceTests passed.' -ForegroundColor Green
