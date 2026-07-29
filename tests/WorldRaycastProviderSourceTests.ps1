param(
    [Parameter(Mandatory = $true)]
    [string]$Root
)

$ErrorActionPreference = 'Stop'
$failures = [System.Collections.Generic.List[string]]::new()

function Read-Source([string]$RelativePath) {
    $path = Join-Path $Root $RelativePath
    if (-not (Test-Path -LiteralPath $path -PathType Leaf)) {
        $failures.Add("Missing source file: $RelativePath")
        return ''
    }
    return Get-Content -LiteralPath $path -Raw
}

function Require-Match(
    [string]$Name,
    [string]$Text,
    [string]$Pattern,
    [string]$Message
) {
    if ($Text -notmatch $Pattern) {
        $failures.Add("$Name`: $Message")
    }
}

function Reject-Match(
    [string]$Name,
    [string]$Text,
    [string]$Pattern,
    [string]$Message
) {
    if ($Text -match $Pattern) {
        $failures.Add("$Name`: $Message")
    }
}

$apiHeader = Read-Source 'src/api/ROCKProviderApi.h'
$apiSource = Read-Source 'src/api/ROCKProviderApi.cpp'
$rayHeader = Read-Source 'src/physics-interaction/native/PhysicsRayCast.h'
$raySource = Read-Source 'src/physics-interaction/native/PhysicsRayCast.cpp'
$providerGlue = Read-Source 'src/physics-interaction/core/PhysicsInteractionProvider.inl'
$selectionSource = Read-Source 'src/physics-interaction/object/ObjectDetection.cpp'

Require-Match 'API header' $apiHeader `
    'ROCK_PROVIDER_MAX_WORLD_RAYCASTS_PER_OWNER_PER_FRAME_V1\s*=\s*8' `
    'The public fixed raycast budget must remain eight queries per owner per frame.'
Require-Match 'API header' $apiHeader `
    'struct\s+RockProviderWorldRaycastRequestV1[\s\S]*RockProviderPoint3\s+startGame[\s\S]*RockProviderPoint3\s+directionGame[\s\S]*maxDistanceGame' `
    'The request must stay value-only and direction/distance based.'
Reject-Match 'API header' $apiHeader `
    'struct\s+RockProviderWorldRaycastRequestV1[\s\S]{0,500}(bhkWorld|hknpWorld|collisionFilter)' `
    'Consumers must not receive a native world pointer or inject collision filters.'
Require-Match 'API source' $apiSource `
    'apiQueryWorldRaycastV1[\s\S]{0,1800}onAnimationOwnerThread\(\)[\s\S]{0,2200}worldRaycastCount[\s\S]{0,500}CapacityFull' `
    'The query must fail closed off the owner thread and enforce the fixed per-frame budget.'
Require-Match 'API source' $apiSource `
    'apiQueryWorldRaycastV1[\s\S]{0,6000}queryProviderWorldRaycastV1' `
    'The public boundary must delegate the native query to PhysicsInteraction.'
Require-Match 'Ray helper header' $rayHeader `
    'Bethesda''s bhkWorld PickObject wrapper[\s\S]*required world synchronization' `
    'The synchronization authority must be documented at the native wrapper.'
Require-Match 'Ray helper source' $raySource `
    'bhkPickData[\s\S]*SetStartEnd[\s\S]*collisionFilter\.filter[\s\S]*PickObject[\s\S]*GetHitFraction' `
    'The helper must use Bethesda PickObject and its normalized closest-hit fraction.'
Reject-Match 'Ray helper source' $raySource `
    '(CastRay|castRay)\s*\(' `
    'The provider must not bypass Bethesda synchronization with a direct hknp cast.'
Require-Match 'Provider glue' $providerGlue `
    'queryProviderWorldRaycastV1[\s\S]*castClosestSegment[\s\S]*rockFarClipRayFilterInfo' `
    'The provider query must use ROCK''s validated far-world filter.'
Require-Match 'Selection source' $selectionSource `
    'findFarObject[\s\S]*castClosestSegment' `
    'ROCK selection and public raycasts must share one native closest-segment primitive.'

if ($failures.Count -gt 0) {
    Write-Host 'WorldRaycastProviderSourceTests failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'WorldRaycastProviderSourceTests passed.' -ForegroundColor Green
