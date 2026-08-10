param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
$failures = [System.Collections.Generic.List[string]]::new()

function Require-Text {
    param([string]$Path, [string]$Pattern, [string]$Message)
    $text = Get-Content -Raw -LiteralPath (Join-Path $Root $Path)
    if ($text -notmatch $Pattern) {
        $failures.Add($Message)
    }
}

$gripRuntime = 'src/physics-interaction/weapon/TwoHandedGrip.cpp'

Require-Text $gripRuntime 'capturePartGrip\([\s\S]*grip\.attachmentRoot\s*=\s*supportAttachmentRoot' `
    'Part-grip capture must retain the concrete scene source root for provider re-resolution.'
Require-Text $gripRuntime 'providerPartTargetNewlyMatchesGrip\([\s\S]*query\.sourceRoot\s*=\s*reinterpret_cast<std::uintptr_t>\(grip\.attachmentRoot\)[\s\S]*resolveWeaponPartTargetV1\(query, resolution\)' `
    'A mid-hold provider target query must include the captured source root before resolving source-root targets.'
Require-Text $gripRuntime 'getHandGripReport\([\s\S]*outReport\.sourceRoot\s*=\s*reinterpret_cast<std::uintptr_t>\(grip\.attachmentRoot\)' `
    'Provider grip reports and mid-hold target re-resolution must publish the same captured source-root identity.'

if ($failures.Count -gt 0) {
    Write-Host 'WeaponPartMidHoldTargetSourceTests failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'WeaponPartMidHoldTargetSourceTests passed.' -ForegroundColor Green
