param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
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

function Require-Pattern(
    [string]$RelativePath,
    [string]$Pattern,
    [string]$Message
) {
    if ((Read-Source $RelativePath) -notmatch $Pattern) {
        $failures.Add("$RelativePath`: $Message")
    }
}

function Require-Order(
    [string]$RelativePath,
    [string[]]$Patterns,
    [string]$Message
) {
    $text = Read-Source $RelativePath
    $offset = 0
    foreach ($pattern in $Patterns) {
        $match = [regex]::Match(
            $text.Substring($offset),
            $pattern,
            [System.Text.RegularExpressions.RegexOptions]::Singleline)
        if (-not $match.Success) {
            $failures.Add("$RelativePath`: $Message")
            return
        }
        $offset += $match.Index + $match.Length
    }
}

$interaction = 'src/physics-interaction/core/PhysicsInteraction.cpp'
$frame = 'src/physics-interaction/core/PhysicsInteractionFrame.inl'
$weaponHeader = 'src/physics-interaction/weapon/TwoHandedGrip.h'
$actorState = 'src/rock_support/Fo4VrActorStatePolicy.h'

Require-Pattern $actorState `
    'kReloadingGunState\s*=\s*4[\s\S]*isNativeReloading\([\s\S]*gunState\s*==\s*kReloadingGunState' `
    'Reload ownership must use the verified native gun-state value instead of a CommonLib bitfield.'
Require-Pattern $interaction `
    'nativeReloadHandAuthorityActive\([\s\S]*currentNativeAnimationAuthorityFlagsV1\(\)[\s\S]*kArms[\s\S]*kHands[\s\S]*isNativeReloading\([\s\S]*getNativeGunState\(f4vr::getPlayer\(\)\)' `
    'Provider arms/hands authority and the verified native gun state must share one reload predicate.'
Require-Pattern $frame `
    'frame\.reloadBoundaryActive\s*=\s*nativeReloadHandAuthorityActive\(\)' `
    'The physics frame must snapshot the shared reload predicate.'

Require-Order $interaction @(
    '_twoHandedGrip\.setNativeReloadHandAuthorityActive\(',
    '_twoHandedGrip\.refreshRetainedHandVisualAuthoritiesBeforeFrik\(',
    '_twoHandedGrip\.refreshWeaponCollisionHandAuthorityBeforeFrik\('
) 'Reload must clear the support-hand claims before any pre-FRIK refresh can retain them.'
Require-Pattern $interaction `
    'nativeReloadHandAuthorityActive\s*=\s*[\r\n\s]*frame\.reloadBoundaryActive[\s\S]*gunstockPresentationBlocked\s*=[\r\n\s]*frame\.menuBlocked\s*\|\|\s*frame\.reloadBoundaryActive' `
    'The coherent grip input must carry reload ownership and block gunstock presentation.'

Require-Pattern $weaponHeader `
    'nativeReloadHandAuthorityActive[\s\S]*setNativeReloadHandAuthorityActive\(bool active\)[\s\S]*isNativeReloadSupportHand[\s\S]*_nativeReloadHandAuthorityActive' `
    'TwoHandedGrip must retain an explicit reload handoff state across pre-FRIK and physics phases.'
Require-Pattern $weaponHeader `
    'weaponCollisionAttachedHands\(\)[\s\S]*selectAttachedHands\([\s\S]*!_nativeReloadHandAuthorityActive' `
    'Dynamic collision must exclude the support hand while native reload owns it.'

if ($failures.Count -gt 0) {
    $failures | ForEach-Object { Write-Error $_ }
    exit 1
}

Write-Host 'Reload hand-authority source tests passed.'
