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

Require-Text 'src/RockConfig.h' `
    'rockSurfaceMeshGrabEnabled\s*=\s*false' `
    'The mesh-authoritative surface grab must default off.'
Require-Text 'src/RockConfig.cpp' `
    'GetBoolValue\(\s*SECTION,\s*"bSurfaceMeshGrabEnabled"[\s\S]*rockSurfaceMeshGrabEnabled' `
    'The surface mesh toggle must load from [PhysicsInteraction].'
Require-Text 'src/RockConfig.cpp' `
    'rockSurfaceMeshGrabMaxProjectionDistanceGameUnits[\s\S]*std::clamp[\s\S]*1\.0f[\s\S]*128\.0f' `
    'The shell-to-mesh projection distance must be bounded.'

foreach ($configPath in @('data/config/ROCK_example.ini')) {
    $text = Get-Content -Raw -LiteralPath (Join-Path $Root $configPath)
    $physicsMatch = [regex]::Match(
        $text,
        '(?ms)^\[PhysicsInteraction\]\s*(?<body>.*?)(?=^\[[^\]]+\]|\z)')
    if (!$physicsMatch.Success) {
        $failures.Add("$configPath`: Missing [PhysicsInteraction] section.")
        continue
    }
    $physicsBody = $physicsMatch.Groups['body'].Value
    if ($physicsBody -notmatch
        '(?m)^bSurfaceMeshGrabEnabled\s*=\s*false\s*$') {
        $failures.Add("$configPath`: Mesh-grab toggle must default off under [PhysicsInteraction].")
    }
    if ($physicsBody -notmatch
        '(?m)^iSurfaceMeshGrabMaxTriangles\s*=\s*20000\s*$') {
        $failures.Add("$configPath`: Mesh-grab triangle budget must be under [PhysicsInteraction].")
    }
    if ($text -match '(?m)^\[Experimental\]\s*$|ExperimentalSurfaceMesh') {
        $failures.Add("$configPath`: Retired experimental surface-grab names must stay removed.")
    }
}

Require-Text 'src/physics-interaction/grab/MeshGrab.h' `
    'struct BoundedSurfaceMeshExtraction[\s\S]*candidateShapes[\s\S]*triangleBudgetExceeded' `
    'Mesh acquisition must expose bounded extraction telemetry.'
Require-Text 'src/physics-interaction/grab/MeshGrab.h' `
    'geometry\.numTriangles\s*>\s*remaining[\s\S]*triangleBudgetExceeded\s*=\s*true[\s\S]*return;' `
    'An oversized shape must fall back before its triangle payload is scanned.'
Require-Text 'src/physics-interaction/grab/TouchGrabRuntime.cpp' `
    'tryAcquireSurfaceMeshPresentation[\s\S]*extractBoundedSurfaceTriangles\([\s\S]*findClosestGrabSurfaceHitToPointPositionOnly\([\s\S]*buildTargetLocalPatch\([\s\S]*solveFrozenMeshFingerPose\(' `
    'Surface acquisition must resolve a bounded visible-mesh anchor and one-shot finger pose.'
Require-Text 'src/physics-interaction/grab/TouchGrabRuntime.cpp' `
    'isSurfaceLatchMeshAuthoritative\(isLeft\)[\s\S]*contactPoint\s*=\s*meshAcquisition\.presentation\.meshAnchorWorld' `
    'The API contact point must become the mesh hit only after mesh authority is accepted.'
Require-Text 'src/physics-interaction/grab/TouchGrabRuntime.cpp' `
    'rockSurfaceMeshGrabEnabled[\s\S]*surfaceGripMode\s*=[\s\S]*CollisionFallback' `
    'An enabled mesh-authoritative grab must report collision fallback when mesh acquisition is unavailable.'

Require-Text 'src/physics-interaction/hand/DynamicHandCollision.cpp' `
    'candidate\.lastHandWorld\s*=\s*meshPresentationRequested[\s\S]*presentation->handWorld[\s\S]*candidate\.lastProxyWorld\[bodyIndex\]\s*=\s*proxyWorld' `
    'The visible hand must use the mesh seat while twins retain their collision-safe shell relation.'
Require-Text 'src/physics-interaction/hand/DynamicHandCollision.cpp' `
    'clearSurfaceMeshPose\([\s\S]*surfaceLatch\s*=\s*\{\}' `
    'Latch teardown must clear mesh pose authority before state reset.'

Require-Text 'src/api/ROCKProviderApi.h' `
    'enum class RockProviderSurfaceGripModeV1[\s\S]*CollisionAnchor\s*=\s*0[\s\S]*MeshAnchor\s*=\s*1[\s\S]*CollisionFallback\s*=\s*2' `
    'The public API must distinguish collision, mesh, and collision-fallback modes.'
Require-Text 'src/api/ROCKProviderApi.h' `
    'struct RockProviderHandInteractionStateV1[\s\S]*surfaceAnchorGame[\s\S]*surfaceGripMode' `
    'The public hand state must report the authoritative surface anchor and mode.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionProvider.inl' `
    'touchGrabReport\.surfaceGripMode[\s\S]*Flag::SurfaceAnchorValid[\s\S]*Flag::MeshSurfaceAnchor[\s\S]*Flag::MeshCollisionFallback' `
    'The aggregate hand API must publish the mesh-grab mode flags and anchor.'

if ($failures.Count -gt 0) {
    Write-Host 'SurfaceMeshGrabSourceTests failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure" -ForegroundColor Red
    }
    exit 1
}

Write-Host 'SurfaceMeshGrabSourceTests passed.' -ForegroundColor Green
