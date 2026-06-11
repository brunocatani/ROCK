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

function Reject-FunctionPattern {
    param(
        [string]$Path,
        [string]$FunctionStart,
        [string]$FunctionEnd,
        [string]$Pattern,
        [string]$Message
    )

    $text = Get-Content -Raw -LiteralPath (Join-Path $Root $Path)
    $start = $text.IndexOf($FunctionStart)
    $end = if ($start -ge 0) { $text.IndexOf($FunctionEnd, $start) } else { -1 }
    if ($start -lt 0 -or $end -lt 0) {
        $failures.Add("Function boundary could not be located for $FunctionStart.")
        return
    }

    $body = $text.Substring($start, $end - $start)
    if ($body -match $Pattern) {
        $failures.Add($Message)
    }
}

Require-Text 'src/physics-interaction/object/ObjectPhysicsBodySet.h' 'struct ObjectPhysicsBodyScanCache' 'Object body acquisition must expose a reusable scan cache.'
Require-Text 'src/physics-interaction/object/ObjectPhysicsBodySet.h' 'captureObjectPhysicsBodyScanCache' 'Object body acquisition must be able to prewarm scan evidence outside the grab edge.'
Require-Text 'src/physics-interaction/object/ObjectPhysicsBodySet.h' 'buildObjectPhysicsBodySetFromScanCache' 'Cached acquisition evidence must rebuild live body records after active prep.'
Require-Text 'src/physics-interaction/object/ObjectPhysicsBodySet.h' 'cachedScanHits' 'Object-body diagnostics must expose cached scan use.'
Require-Text 'src/physics-interaction/object/ObjectPhysicsBodySet.cpp' 'captureScanNode' 'Cache capture must walk the selected tree before grab commit.'
Require-Text 'src/physics-interaction/object/ObjectPhysicsBodySet.cpp' 'appendCachedBodyRecords' 'Cached acquisition must classify cached body ids through the normal body-record path.'

Require-Text 'src/physics-interaction/hand/Hand.h' 'struct GrabAcquisitionCache' 'Each hand must own an acquisition cache keyed to the selected object.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'updateGrabAcquisitionCache\(bhkWorld,\s*hknpWorld\)' 'Selection update must prewarm acquisition evidence while an object is selected.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'grabAcquisitionCacheMatches' 'Grab cache use must be guarded by explicit selection/world identity checks.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' '_grabAcquisitionCache\.rootNode\s*==\s*rootNode' 'Cache identity must include the selected root node pointer.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' '_grabAcquisitionCache\.selectedBodyId\s*==\s*options\.seedBodyId' 'Cache identity must include the selected body id.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' '_grabAcquisitionCache\.hknpWorld\s*==\s*hknpWorld' 'Cache identity must include the hknp world.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' '_grabAcquisitionCache\.bhkWorld\s*==\s*bhkWorld' 'Cache identity must include the bhk world.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'clearGrabAcquisitionCache\(rememberDeselect \? "selection-cleared' 'Selection cleanup must invalidate non-owning cached engine pointers.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'tryUseGrabAcquisitionBeforePrepCache[\s\S]*scanObjectPhysicsBodySet' 'Grab/pull startup must try cached before-prep evidence before direct tree scan fallback.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'tryBuildGrabAcquisitionPreparedBodySetFromCache[\s\S]*scanObjectPhysicsBodySet' 'Grab/pull startup must try cached prepared evidence before direct tree scan fallback.'

Require-Text 'src/physics-interaction/performance/PerformanceProfiler.h' 'GrabAcquisitionBodyScan' 'Profiler must expose grab acquisition body scan timing.'
Require-Text 'src/physics-interaction/performance/PerformanceProfiler.h' 'GrabAcquisitionActivePrep' 'Profiler must expose recursive active-prep timing.'
Require-Text 'src/physics-interaction/performance/PerformanceProfiler.h' 'GrabMeshExtraction' 'Profiler must expose mesh extraction timing.'
Require-Text 'src/physics-interaction/performance/PerformanceProfiler.h' 'GrabNearbyDampingBegin' 'Profiler must expose nearby damping begin timing.'
Require-Text 'src/physics-interaction/performance/PerformanceProfiler.h' 'GrabAcquisitionCacheHit' 'Profiler must count grab acquisition cache hits.'
Require-Text 'src/physics-interaction/performance/PerformanceProfiler.h' 'GrabAcquisitionCacheMiss' 'Profiler must count grab acquisition cache misses.'

Reject-FunctionPattern `
    'src/physics-interaction/hand/HandGrab.cpp' `
    'bool Hand::grabSelectedObject' `
    'void Hand::updateHeldObject' `
    'const auto beforePrepBodySet\s*=\s*object_physics_body_set::scanObjectPhysicsBodySet[\s\S]*object_physics_body_set::scanObjectPhysicsBodySet\(bhkWorld,\s*world,\s*sel\.refr' `
    'grabSelectedObject must not return to unconditional double full-tree scans on the grab edge.'

if ($failures.Count -gt 0) {
    Write-Host 'Grab acquisition cache source boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Grab acquisition cache source boundary passed.'
