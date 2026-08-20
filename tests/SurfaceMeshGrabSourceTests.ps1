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
    'rockExperimentalSurfaceMeshGrabEnabled\s*=\s*false' `
    'The mesh-authoritative surface grab must remain experimental and default off.'
Require-Text 'src/RockConfig.cpp' `
    'GetBoolValue\(\s*EXPERIMENTAL_SECTION,\s*"bExperimentalSurfaceMeshGrabEnabled"[\s\S]*rockExperimentalSurfaceMeshGrabEnabled' `
    'The experimental toggle must load exclusively from [Experimental].'
Require-Text 'src/RockConfig.cpp' `
    'rockExperimentalSurfaceMeshGrabMaxProjectionDistanceGameUnits[\s\S]*std::clamp[\s\S]*1\.0f[\s\S]*128\.0f' `
    'The shell-to-mesh projection distance must be bounded.'

foreach ($configPath in @('data/config/ROCK.ini')) {
    $text = Get-Content -Raw -LiteralPath (Join-Path $Root $configPath)
    $experimentalMatch = [regex]::Match(
        $text,
        '(?ms)^\[Experimental\]\s*(?<body>.*?)(?=^\[[^\]]+\])')
    if (!$experimentalMatch.Success) {
        $failures.Add("$configPath`: Missing [Experimental] section.")
        continue
    }
    $experimentalBody = $experimentalMatch.Groups['body'].Value
    if ($experimentalBody -notmatch
        '(?m)^bExperimentalSurfaceMeshGrabEnabled\s*=\s*false\s*$') {
        $failures.Add("$configPath`: Mesh-grab toggle must default off under [Experimental].")
    }
    if ($experimentalBody -notmatch
        '(?m)^iExperimentalSurfaceMeshGrabMaxTriangles\s*=\s*20000\s*$') {
        $failures.Add("$configPath`: Mesh-grab triangle budget must be under [Experimental].")
    }
}

Require-Text 'src/physics-interaction/grab/MeshGrab.h' `
    'struct BoundedSurfaceMeshExtraction[\s\S]*candidateShapes[\s\S]*triangleBudgetExceeded' `
    'Mesh acquisition must expose bounded extraction telemetry.'
Require-Text 'src/physics-interaction/grab/MeshGrab.h' `
    'geometry\.numTriangles\s*>\s*remaining[\s\S]*triangleBudgetExceeded\s*=\s*true[\s\S]*return;' `
    'An oversized shape must fall back before its triangle payload is scanned.'

Require-Text 'src/api/ROCKProviderApi.h' `
    'enum class RockProviderSurfaceGripModeV1[\s\S]*CollisionAnchor\s*=\s*0[\s\S]*MeshAnchor\s*=\s*1[\s\S]*CollisionFallback\s*=\s*2' `
    'The public API must distinguish collision, mesh, and collision-fallback modes.'
Require-Text 'src/api/ROCKProviderApi.h' `
    'struct RockProviderHandInteractionStateV1[\s\S]*surfaceAnchorGame[\s\S]*surfaceGripMode' `
    'The public hand state must report the authoritative surface anchor and mode.'
if ($failures.Count -gt 0) {
    Write-Host 'SurfaceMeshGrabSourceTests failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure" -ForegroundColor Red
    }
    exit 1
}

Write-Host 'SurfaceMeshGrabSourceTests passed.' -ForegroundColor Green
