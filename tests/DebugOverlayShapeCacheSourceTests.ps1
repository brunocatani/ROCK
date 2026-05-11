$ErrorActionPreference = 'Stop'

$repoRoot = Split-Path -Parent $PSScriptRoot

function Read-RepoFile {
    param([Parameter(Mandatory = $true)][string]$RelativePath)

    return Get-Content -Raw -LiteralPath (Join-Path $repoRoot $RelativePath)
}

function Require-Text {
    param(
        [Parameter(Mandatory = $true)][string]$RelativePath,
        [Parameter(Mandatory = $true)][string]$Pattern,
        [Parameter(Mandatory = $true)][string]$Message
    )

    $content = Read-RepoFile $RelativePath
    if ($content -notmatch $Pattern) {
        throw $Message
    }
}

function Reject-Text {
    param(
        [Parameter(Mandatory = $true)][string]$RelativePath,
        [Parameter(Mandatory = $true)][string]$Pattern,
        [Parameter(Mandatory = $true)][string]$Message
    )

    $content = Read-RepoFile $RelativePath
    if ($content -match $Pattern) {
        throw $Message
    }
}

Require-Text 'src/physics-interaction/debug/DebugBodyOverlay.cpp' 'geometryFingerprint' 'Debug overlay shape cache keys must include geometry identity, not only a raw Havok shape pointer.'
Require-Text 'src/physics-interaction/debug/DebugBodyOverlay.cpp' 'computeShapeGeometryFingerprint' 'Debug overlay must fingerprint generated collider shape geometry so rebuilt bodies cannot draw stale meshes.'
Require-Text 'src/physics-interaction/debug/DebugBodyOverlay.cpp' 'GetSupportVertices\(supportBuffer,\s*vertexCount\)' 'Convex generated collider fingerprints must sample support vertices.'
Require-Text 'src/physics-interaction/debug/DebugBodyOverlay.cpp' 'case 11:[\s\S]{0,800}computeShapeGeometryFingerprint\(innerShape,\s*depth \+ 1\)' 'Scaled hknp shapes must include inner shape geometry in the debug overlay cache key.'
Require-Text 'src/physics-interaction/debug/DebugBodyOverlay.cpp' 'power armor change' 'The debug overlay cache fix must document the generated profile-change failure mode.'
Require-Text 'src/physics-interaction/debug/DebugBodyOverlay.cpp' 'const ShapeKey key = makeShapeKey\(body\.shapeAddress\);' 'Debug overlay body rendering must use the geometry-aware shape cache key.'
Require-Text 'src/physics-interaction/debug/DebugBodyOverlay.cpp' 'kMaxBodyIndex\s*=\s*body_frame::kMaxReadableBodyIndex' 'Debug overlay body extraction must share the runtime readable body bound so long-session generated colliders still visualize.'

Reject-Text 'src/physics-interaction/debug/DebugBodyOverlay.cpp' 'ShapeKey key\{\s*body\.shapeAddress\s*\};' 'Debug overlay must not cache GPU meshes by shape pointer alone.'
Reject-Text 'src/physics-interaction/debug/DebugBodyOverlay.cpp' 'kMaxBodyIndex\s*=\s*8192' 'Debug overlay must not keep the obsolete 8192 body-id cap after runtime accepts long-session body ids.'

Write-Host 'Debug overlay shape cache source checks passed.'
