param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
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

function Require-Text(
    [string]$RelativePath,
    [string]$Pattern,
    [string]$Message
) {
    $text = Read-Source $RelativePath
    if ($text -notmatch $Pattern) {
        $failures.Add("$RelativePath`: $Message")
    }
}

Require-Text 'src/api/ROCKProviderApi.h' `
    'RockProviderEquippedWeaponGripStateFlagV1[\s\S]*MuzzleWorldValid\s*=\s*1u\s*<<\s*7[\s\S]*RockProviderEquippedWeaponGripStateV1[\s\S]*muzzleOriginGame[\s\S]*muzzleDirectionGame[\s\S]*reserved\[2\]' `
    'ROCK V1 must consume only reserved grip-state storage for the muzzle snapshot and preserve the record size.'


Require-Text 'src/rock_support/Fo4VrRuntime.h' `
    'EquippedWeaponData_vtable\s*\{\s*REL::Offset\(0x2D7FCF8\)\s*\}' `
    'The equipped-data type gate must retain the FO4VR 1.2.72 vtable verified from raw constructor disassembly.'


Require-Text 'tests/ProviderApiAbiTests.cpp' `
    'RockProviderEquippedWeaponGripStateV1,\s*224,\s*8[\s\S]*muzzleOriginGame\)\s*==\s*188[\s\S]*muzzleDirectionGame\)\s*==\s*200' `
    'ABI regression coverage must lock the unchanged V1 record size and new reserved-field offsets.'

if ($failures.Count -gt 0) {
    Write-Host 'EquippedWeaponMuzzleProviderSourceTests failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'EquippedWeaponMuzzleProviderSourceTests passed.' -ForegroundColor Green
