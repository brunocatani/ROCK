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


$actorState = 'src/rock_support/Fo4VrActorStatePolicy.h'
$reloadPolicy = 'src/physics-interaction/weapon/NativeReloadHandAuthorityPolicy.h'
$interaction = 'src/physics-interaction/core/PhysicsInteractionInternal.cpp'
$inputRuntime = 'src/physics-interaction/input/InputRemapRuntime.cpp'

Require-Pattern $actorState `
    'kReloadingGunState\s*=\s*4[\s\S]*isNativeReloading\([\s\S]*gunState\s*==\s*kReloadingGunState' `
    'Reload ownership must use the verified native gun-state value instead of a CommonLib bitfield.'
Require-Pattern $reloadPolicy `
    'enteredFromFire[\s\S]*explicitReloadPending[\s\S]*automaticEmptyReload[\s\S]*!enteredFromFire' `
    'Post-fire state four must be separated from routed, empty-magazine, and independent native reloads.'
Require-Pattern $reloadPolicy `
    'providerOwnsArmsOrHands\s*\|\|[\r\n\s]*state\.rawReloadOwnsSupportHand' `
    'Explicit provider arms or hands authority must remain the highest-priority reload handoff.'
Require-Pattern $inputRuntime `
    'nativeActionDispatcher\([\s\S]*s_nativeReloadDispatchSequence\.fetch_add' `
    'Only an accepted native reload dispatch may publish the explicit reload witness.'
Require-Pattern $interaction `
    'native_reload_hand_authority_policy::update\([\s\S]*nativeReloadDispatchSequence\(\)[\s\S]*ammoCount\s*==\s*0' `
    'The shared pre-FRIK and physics-frame predicate must classify state four with reload, firing, and magazine evidence.'

if ($failures.Count -gt 0) {
    $failures | ForEach-Object { Write-Error $_ }
    exit 1
}

Write-Host 'Reload hand-authority source tests passed.'
