$ErrorActionPreference = 'Stop'

$repoRoot = Split-Path -Parent $PSScriptRoot

function Read-RepoFile {
    param([Parameter(Mandatory = $true)][string]$RelativePath)

    return Get-Content -Raw -Path (Join-Path $repoRoot $RelativePath)
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

Require-Text 'src/physics-interaction/hand/HandColliderTypes.h' 'struct\s+ColliderDimensionLimits' 'Generated hand/body collider geometry must keep a shared dimension-limit policy.'
Require-Text 'src/physics-interaction/hand/HandColliderTypes.h' 'colliderDimensionsWithinLimits' 'Generated collider dimensions must be validated through the shared policy.'
Require-Text 'src/physics-interaction/hand/HandColliderTypes.h' 'finiteVector\(start\).*finiteVector\(end\)' 'Segment frame derivation must reject non-finite skeleton endpoints before measuring length.'
Require-Text 'src/physics-interaction/hand/HandColliderTypes.h' 'std::isfinite\(result\.length\)' 'Segment frame derivation must reject non-finite segment lengths.'
Require-Text 'src/physics-interaction/hand/HandColliderTypes.h' 'std::isfinite\(palmLength\)' 'Palm-anchor derivation must reject non-finite palm spans.'
Require-Text 'src/physics-interaction/hand/HandColliderTypes.h' 'std::isfinite\(lengthValue\)\s*\?\s*lengthValue\s*:\s*0\.05f' 'Capsule hull construction must not turn non-finite length input into non-finite Havok points.'
Require-Text 'src/physics-interaction/hand/HandColliderTypes.h' 'std::isfinite\(radius\)\s*\?\s*radius\s*:\s*0\.01f' 'Capsule hull construction must not turn non-finite radius input into non-finite Havok points.'
Require-Text 'src/physics-interaction/hand/HandColliderTypes.h' 'runtime trust boundary' 'The generated collider dimension guard must retain its design rationale in source.'

Require-Text 'src/physics-interaction/hand/HandBoneColliderSet.cpp' 'handRoleDimensionLimits' 'Hand generated colliders must use role-aware plausibility limits.'
Require-Text 'src/physics-interaction/hand/HandBoneColliderSet.cpp' 'handRoleFrameDimensionsValid' 'Hand generated colliders must validate final role dimensions before shape creation or drive.'
Require-Text 'src/physics-interaction/hand/HandBoneColliderSet.cpp' 'implausible dimensions' 'Rejected oversized hand collider frames must be logged with dimensions.'

Require-Text 'src/physics-interaction/body/BodyBoneColliderSet.cpp' 'bodyRoleDimensionLimits' 'Body generated colliders must use role-aware plausibility limits.'
Require-Text 'src/physics-interaction/body/BodyBoneColliderSet.cpp' 'descriptorFrameDimensionsValid' 'Body generated colliders must validate descriptor dimensions after all scale sources are applied.'
Require-Text 'src/physics-interaction/body/BodyBoneColliderSet.cpp' 'Rejected implausible body collider frame' 'Rejected oversized body collider frames must be logged with descriptor context.'

Require-Text 'tests/TransformConventionTests.cpp' 'generated collider dimensions reject oversized hand segment' 'Unit tests must reject oversized generated hand collider lengths.'
Require-Text 'tests/TransformConventionTests.cpp' 'hand bone segment frame rejects non-finite endpoint' 'Unit tests must reject non-finite generated collider skeleton endpoints.'

Reject-Text 'src/physics-interaction/hand/HandBoneColliderSet.cpp' 'buildShapeForRole\(frame,\s*role\)[\s\S]{0,900}buildConvexShapeFromLocalHavokPoints' 'Hand shape creation should remain behind frame validation; update this boundary test if shape creation grows a direct unguarded path.'

Write-Host 'Generated collider dimension source checks passed.'
