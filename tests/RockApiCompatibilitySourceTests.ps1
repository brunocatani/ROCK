$ErrorActionPreference = 'Stop'

<#
ROCK's public header intentionally exposes only ROCK-owned behavior, but the DLL export must
preserve the older function-table length consumed by sibling plugins. This test guards that
split: no public legacy surface, but an inert private tail remains behind ROCKAPI_GetApi().
#>

$repoRoot = Resolve-Path (Join-Path $PSScriptRoot '..')
$apiCppPath = Join-Path $repoRoot 'src/api/ROCKApi.cpp'
$apiHeaderPath = Join-Path $repoRoot 'src/api/ROCKApi.h'

$apiCpp = Get-Content -LiteralPath $apiCppPath -Raw
$apiHeader = Get-Content -LiteralPath $apiHeaderPath -Raw
$failures = New-Object System.Collections.Generic.List[string]

function Require-Pattern {
    param(
        [string]$Content,
        [string]$Pattern,
        [string]$Message
    )

    if ($Content -notmatch $Pattern) {
        $script:failures.Add($Message)
    }
}

function Reject-Pattern {
    param(
        [string]$Content,
        [string]$Pattern,
        [string]$Message
    )

    if ($Content -match $Pattern) {
        $script:failures.Add($Message)
    }
}

Require-Pattern $apiCpp 'struct\s+ROCKApiPrefixWithLegacyTail' `
    'ROCK API export must keep a private compatibility table with inert trailing slots.'
Require-Pattern $apiCpp 'legacyUintSlot0' `
    'ROCK API compatibility table must retain opaque uint trailing slots.'
Require-Pattern $apiCpp 'legacyBoolSlotWithUint' `
    'ROCK API compatibility table must retain the legacy bool-with-uint trailing slot.'
Require-Pattern $apiCpp 'static_assert\(offsetof\(ROCKApiPrefixWithLegacyTail,\s*getLastTouchedWeaponPartKind\)\s*==\s*offsetof\(ROCKApi,\s*getLastTouchedWeaponPartKind\)' `
    'ROCK API compatibility table must statically guard the public prefix offset.'
Require-Pattern $apiCpp 'static_assert\(sizeof\(ROCKApiPrefixWithLegacyTail\)\s*>\s*sizeof\(ROCKApi\)' `
    'ROCK API compatibility table must statically guard that legacy trailing slots remain exported.'
Require-Pattern $apiCpp 'reinterpret_cast<const ROCKApi\*>\(&ROCK_API_FUNCTIONS_TABLE\)' `
    'ROCKAPI_GetApi must return the public prefix pointer from the larger compatibility table.'

Reject-Pattern $apiHeader 'legacyUintSlot|legacyBoolSlot|ROCKApiPrefixWithLegacyTail' `
    'ROCK public API header must not expose private compatibility tail implementation details.'

if ($failures.Count -gt 0) {
    $failures | ForEach-Object { Write-Error $_ }
    exit 1
}

Write-Host 'ROCK API compatibility source boundary passed.'
