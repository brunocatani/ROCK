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

function Reject-Text {
    param(
        [string]$RelativePath,
        [string]$Pattern,
        [string]$Message
    )

    $path = Join-Path $Root $RelativePath
    $text = Get-Content -Raw -LiteralPath $path
    if ($text -match $Pattern) {
        $failures.Add("$RelativePath`: $Message")
    }
}

Require-Text 'src/physics-interaction/core/PhysicsInteractionContacts.inl' '_generatedBodyContactRegistry\.tryClassify\(bodyIdA' 'Native contact callback must classify endpoint A through the generated-body registry.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionContacts.inl' '_generatedBodyContactRegistry\.tryClassify\(bodyIdB' 'Native contact callback must classify endpoint B through the generated-body registry.'
Reject-Text 'src/physics-interaction/core/PhysicsInteractionContacts.inl' 'tryGetWeaponContactAtomic|tryGetWeaponBodySampledVelocityAtomic|isWeaponBodyIdAtomic|tryGetBodyMetadataAtomic|isColliderBodyIdAtomic|isHandColliderBodyId|tryGetHandColliderMetadata' 'Native contact callback must not perform generated owner scans after registry classification.'

Reject-Text 'src/physics-interaction/hand/HandBoneColliderSet.cpp' 'snapshotGeneratedKeyframedBodyDriveSampledVelocity' 'Hand generated body queue path must publish queue-result sampled velocity without immediately snapshotting drive state.'
Reject-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'snapshotGeneratedKeyframedBodyDriveSampledVelocity' 'Weapon generated body queue path must publish queue-result sampled velocity without immediately snapshotting drive state.'
Require-Text 'src/physics-interaction/hand/HandBoneColliderSet.cpp' 'publishSampledVelocityAtomic\(publicationIndex, queueResult\)' 'Hand generated body queue path must publish sampled velocity by cached publication slot.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'publishSampledVelocityAtomic\(instance\.publicationIndex, queueResult\)' 'Weapon generated body queue path must publish sampled velocity by cached publication slot.'
Require-Text 'src/physics-interaction/contact/GeneratedBodyContactRegistry.h' 'std::atomic<std::uint64_t> _publicationVersion' 'Generated-body callback registry must use atomic publication versioning.'
Require-Text 'src/physics-interaction/contact/GeneratedBodyContactRegistry.h' 'std::sort' 'Generated-body callback registry must publish sorted fixed storage for bounded lookup.'

$physicsInteraction = Get-Content -Raw -LiteralPath (Join-Path $Root 'src/physics-interaction/core/PhysicsInteraction.cpp')
$frameGeneratedBodyBlock = [regex]::Match(
    $physicsInteraction,
    '(?s)updateHandCollisions\(frame\);(?<block>.*?)updateSelection\(frame\);')
if (!$frameGeneratedBodyBlock.Success) {
    $failures.Add('src/physics-interaction/core/PhysicsInteraction.cpp: Could not locate generated-body frame update block for registry publication checks.')
} else {
    $block = $frameGeneratedBodyBlock.Groups['block'].Value
    $publishCount = [regex]::Matches($block, 'refreshGeneratedBodyContactRegistry\(\);').Count
    if ($publishCount -ne 1) {
        $failures.Add("src/physics-interaction/core/PhysicsInteraction.cpp: Generated-body frame update must publish the contact registry exactly once after queueing targets; found $publishCount.")
    }
    if ($block -notmatch '(?s)updateBodiesFromCurrentSourceTransforms\(hknp,\s*weaponNode,\s*frame\.deltaSeconds\).*?refreshGeneratedBodyContactRegistry\(\);\s*_generatedBodyStepDrive\.registerForNextStep\(bhk,\s*hknp\);') {
        $failures.Add('src/physics-interaction/core/PhysicsInteraction.cpp: Generated-body registry publication and step-drive registration must run after weapon transform target queueing.')
    }
}

if ($failures.Count -gt 0) {
    Write-Host 'GeneratedBodyContactRegistrySourceTests failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'GeneratedBodyContactRegistrySourceTests passed.' -ForegroundColor Green
