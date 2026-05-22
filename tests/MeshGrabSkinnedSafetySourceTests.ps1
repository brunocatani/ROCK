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
Require-Text 'src/physics-interaction/grab/MeshGrab.h' 'static_assert\(sizeof\(RE::BSSkin::Instance\) == 0xC0\)' 'Skinned mesh extraction must bind itself to the verified FO4VR BSSkin::Instance size.'
Require-Text 'src/physics-interaction/grab/MeshGrab.h' 'constexpr int bonesData = 0x10' 'Skinned mesh extraction must read the FO4VR BSSkin bone-node array from the verified +0x10 field.'
Require-Text 'src/physics-interaction/grab/MeshGrab.h' 'constexpr int bonesCount = 0x18' 'Skinned mesh extraction must read the FO4VR BSSkin bone count from the verified +0x18 field.'
Require-Text 'src/physics-interaction/grab/MeshGrab.h' 'constexpr int boneData = 0x40' 'Skinned mesh extraction must read the FO4VR BSSkin bone-data pointer from the verified +0x40 field.'
Require-Text 'src/physics-interaction/grab/MeshGrab.h' 'tryReadValue\(boneNodes \+ b, boneNode\)' 'Skinned mesh extraction must read bone node array entries through guarded reads.'
Require-Text 'src/physics-interaction/grab/MeshGrab.h' 'tryReadField\(skinInst, BSSkinOffset::bonesData, boneNodes\)' 'Skinned mesh extraction must read the verified BSSkin bone node array field.'
Require-Text 'src/physics-interaction/grab/MeshGrab.h' 'tryReadField\(skinInst, BSSkinOffset::bonesCount, boneCount\)' 'Skinned mesh extraction must read the verified BSSkin bone count field.'
Require-Text 'src/physics-interaction/grab/MeshGrab.h' 'tryReadField\(skinInst, BSSkinOffset::boneData, boneData\)' 'Skinned mesh extraction must read the verified BSSkin bone data field.'
Require-Text 'src/physics-interaction/grab/MeshGrab.h' 'pointerRangeLooksReadable\(reinterpret_cast<const char\*>\(boneNode\) \+ 0x70, 0x40\)' 'Skinned mesh extraction must reject unreadable bone world transforms before dereferencing.'
Require-Text 'src/physics-interaction/grab/MeshGrab.h' 'guardedCopyFromMemory\(skinToBoneArray \+ b \* 0x50 \+ 0x10' 'Skinned mesh extraction must guard skin-to-bone transform copies.'
Require-Text 'src/physics-interaction/grab/MeshGrab.h' 'worldVertexValid' 'Skinned mesh extraction must track vertex validity instead of emitting zeroed vertices.'
Require-Text 'src/physics-interaction/grab/MeshGrab.h' 'skippedInvalidVertexTriangles' 'Skinned mesh extraction must skip triangles whose skinned vertices cannot be trusted.'
Require-Text 'src/physics-interaction/grab/MeshGrab.h' 'invalidBoneNodePointers' 'Skinned mesh extraction must log invalid bone pointer skips for crash diagnosis.'
Require-Text 'src/physics-interaction/grab/MeshGrab.h' 'DynamicTriShapeVertexLock\s+dynamicVertexLock\(dynamicSkinned \? triShape : nullptr\)' 'Dynamic skinned extraction must lock live dynamic vertices instead of rejecting the shape.'
Require-Text 'src/physics-interaction/grab/MeshGrab.h' 'readDynamicVertexPosition\(dynamicVerts \+ vi \* dynamicStride\)' 'Dynamic skinned extraction must use the locked dynamic vertex positions for palm-pocket surface authority.'
Require-Text 'src/physics-interaction/grab/MeshGrab.h' 'BSDynamicTriShape already exposes live vertex positions' 'Dynamic skinned surface authority must not depend on readable bone owners.'
Require-Text 'src/physics-interaction/grab/MeshGrab.h' 'allowPositionOnlySkinnedSurface' 'Close hand-pocket grabs must be able to request position-only skinned fallback evidence.'
Require-Text 'src/physics-interaction/grab/MeshGrab.h' 'hasSkinInfluences = hasAnySkinInfluence' 'Skinned surface influence metadata must be marked present only when a real guarded bone owner was captured.'
Require-Text 'src/physics-interaction/grab/MeshGrab.h' 'triangleHasSkinInfluences \? &skinInfluences : nullptr' 'Position-only skinned triangles must not pretend to have bone owner influence data.'
Require-Text 'src/physics-interaction/grab/MeshGrab.h' 'vertexInfluences\[vi\]\[k\]\s*=\s*GrabSurfaceVertexInfluence\{\s*boneNodesByIndex\[bIdx\],\s*w\s*\}' 'Dynamic skinned extraction must preserve guarded bone influence ownership for body matching.'
Require-Text 'src/physics-interaction/grab/MeshGrab.h' 'GrabSurfaceSourceKind::Skinned' 'Dynamic skinned triangles must remain skinned surface evidence, not generic collision fallback.'

Reject-Text 'src/physics-interaction/grab/MeshGrab.h' 'auto\*\s+boneNode\s*=\s*boneNodes\[b\]' 'Skinned mesh extraction must not directly dereference boneNodes[b].'
Reject-Text 'src/physics-interaction/grab/MeshGrab.h' 'GrabSurfaceVertexInfluence\{\s*boneNodes\[bIdx\]' 'Skinned surface influence capture must not reread boneNodes[bIdx] directly.'
Reject-Text 'src/physics-interaction/grab/MeshGrab.h' 'tryReadField\(skinInst,\s*0x18,\s*boneNodes\)' 'Skinned mesh extraction must not use the old incorrect +0x18 bone-node pointer read.'
Reject-Text 'src/physics-interaction/grab/MeshGrab.h' 'tryReadField\(skinInst,\s*0x38,\s*boneCount\)' 'Skinned mesh extraction must not use the old incorrect +0x38 bone-count read.'
Reject-Text 'src/physics-interaction/grab/MeshGrab.h' 'Skipping skinned BSDynamicTriShape' 'Skinned BSDynamicTriShape must no longer be hard-skipped from grab mesh authority.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'const int meshExtractionDepth = \(std::max\)\(1,\s*g_rockConfig\.rockObjectPhysicsTreeMaxDepth\)' 'Mesh extraction must honor the configured object tree depth instead of a hardcoded recursion cap.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'extractAllSurfaceTriangles\(meshSourceNode,[\s\S]*meshExtractionDepth' 'Hand grab must pass the configured mesh extraction depth into surface traversal.'

if ($failures.Count -gt 0) {
    Write-Error ($failures -join [Environment]::NewLine)
}

Write-Host 'Mesh grab skinned safety source boundaries passed.'
