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

Require-Pattern $actorState `
    'kReloadingGunState\s*=\s*4[\s\S]*isRawNativeReloadState\([\s\S]*gunState\s*==\s*kReloadingGunState' `
    'Raw reload telemetry must use the verified native gun-state value instead of a CommonLib bitfield.'
Require-Pattern $reloadPolicy `
    'explicitReloadPending[\s\S]*automaticEmptyReload[\s\S]*nativeReloadState[\s\S]*rawReloadOwnsSupportHand\s*=\s*explicitReloadPending\s*\|\|\s*automaticEmptyReload' `
    'Native reload hand ownership must require positive routed or empty-magazine evidence in raw state four.'
Require-Pattern $reloadPolicy `
    'kReloadAuthorityFrameWindow[\s\S]*reloadAuthorityExpiresAtFrame[\s\S]*frameIndex\s*>\s*state\.reloadAuthorityExpiresAtFrame[\s\S]*rawReloadOwnsSupportHand\s*=\s*false' `
    'A stuck native state must have a bounded authority lifetime.'

if ($failures.Count -gt 0) {
    $failures | ForEach-Object { Write-Error $_ }
    exit 1
}

Write-Host 'Reload hand-authority source tests passed.'
