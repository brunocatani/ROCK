param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$failures = [System.Collections.Generic.List[string]]::new()
$Root = (Resolve-Path -LiteralPath $Root).Path

function Get-RepoText {
    param([string]$Path)

    $fullPath = Join-Path $Root $Path
    if (-not (Test-Path -LiteralPath $fullPath)) {
        return $null
    }
    return Get-Content -Raw -LiteralPath $fullPath
}

function Require-Text {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )

    $text = Get-RepoText $Path
    if ($null -eq $text -or $text -notmatch $Pattern) {
        $failures.Add($Message)
    }
}

function Reject-Text {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )

    $text = Get-RepoText $Path
    if ($null -ne $text -and $text -match $Pattern) {
        $failures.Add($Message)
    }
}

Require-Text 'src/physics-interaction/native/NativeMemory.cpp' 'VirtualQuery' 'Native memory helper must use page readability checks before pointer reads.'
Require-Text 'src/physics-interaction/native/NativeMemory.cpp' '__try' 'Native memory helper must protect final native memory copies with SEH on MSVC.'
Require-Text 'src/physics-interaction/grab/MeshGrab.h' 'physics-interaction/native/NativeMemory.h' 'Mesh grab extraction must use the shared native memory helper.'
Require-Text 'src/physics-interaction/grab/MeshGrab.h' 'tryReadValue\(boneNodes \+ b, boneNode\)' 'Skinned mesh extraction must read bone node array entries through guarded reads.'
Require-Text 'src/physics-interaction/grab/MeshGrab.h' 'pointerRangeLooksReadable\(reinterpret_cast<const char\*>\(boneNode\) \+ 0x70, 0x40\)' 'Skinned mesh extraction must reject unreadable bone world transforms before dereferencing.'
Require-Text 'src/physics-interaction/grab/MeshGrab.h' 'guardedCopyFromMemory\(skinToBoneArray \+ b \* 0x50 \+ 0x10' 'Skinned mesh extraction must guard skin-to-bone transform copies.'
Require-Text 'src/physics-interaction/grab/MeshGrab.h' 'worldVertexValid' 'Skinned mesh extraction must track vertex validity instead of emitting zeroed vertices.'
Require-Text 'src/physics-interaction/grab/MeshGrab.h' 'skippedInvalidVertexTriangles' 'Skinned mesh extraction must skip triangles whose skinned vertices cannot be trusted.'
Require-Text 'src/physics-interaction/grab/MeshGrab.h' 'invalidBoneNodePointers' 'Skinned mesh extraction must log invalid bone pointer skips for crash diagnosis.'
Require-Text 'src/physics-interaction/grab/MeshGrab.h' 'DynamicTriShapeVertexLock\s+dynamicVertexLock\(dynamicSkinned \? triShape : nullptr\)' 'Dynamic skinned extraction must lock live dynamic vertices instead of rejecting the shape.'
Require-Text 'src/physics-interaction/grab/MeshGrab.h' 'readDynamicVertexPosition\(dynamicVerts \+ vi \* dynamicStride\)' 'Dynamic skinned extraction must use the locked dynamic vertex positions for palm-pocket surface authority.'
Require-Text 'src/physics-interaction/grab/MeshGrab.h' 'vertexInfluences\[vi\]\[k\]\s*=\s*GrabSurfaceVertexInfluence\{\s*boneNodesByIndex\[bIdx\],\s*w\s*\}' 'Dynamic skinned extraction must preserve guarded bone influence ownership for body matching.'
Require-Text 'src/physics-interaction/grab/MeshGrab.h' 'GrabSurfaceSourceKind::Skinned' 'Dynamic skinned triangles must remain skinned surface evidence, not generic collision fallback.'

Reject-Text 'src/physics-interaction/grab/MeshGrab.h' 'auto\*\s+boneNode\s*=\s*boneNodes\[b\]' 'Skinned mesh extraction must not directly dereference boneNodes[b].'
Reject-Text 'src/physics-interaction/grab/MeshGrab.h' 'GrabSurfaceVertexInfluence\{\s*boneNodes\[bIdx\]' 'Skinned surface influence capture must not reread boneNodes[bIdx] directly.'
Reject-Text 'src/physics-interaction/grab/MeshGrab.h' 'Skipping skinned BSDynamicTriShape' 'Skinned BSDynamicTriShape must no longer be hard-skipped from grab mesh authority.'

if ($failures.Count -gt 0) {
    Write-Error ($failures -join [Environment]::NewLine)
}

Write-Host 'Mesh grab skinned safety source boundaries passed.'
