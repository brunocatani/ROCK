param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$failures = [System.Collections.Generic.List[string]]::new()

function Read-Source {
    param([string]$RelativePath)

    return Get-Content -Raw -LiteralPath (Join-Path $Root $RelativePath)
}

function Require-Text {
    param(
        [string]$RelativePath,
        [string]$Pattern,
        [string]$Message
    )

    $text = Read-Source $RelativePath
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

    $text = Read-Source $RelativePath
    if ($text -match $Pattern) {
        $failures.Add("$RelativePath`: $Message")
    }
}

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'bool equippedWeaponHasKeyword\(\s*const RE::TESObjectWEAP\* weapon,\s*const RE::BGSKeyword\* keyword\)' `
    'Base equipped weapon keyword checks must not accept TBO instance data.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'weapon->HasKeyword\(keyword,\s*nullptr\)' `
    'Base WEAP keywords must call HasKeyword with nullptr instance data.'

Reject-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'weapon->HasKeyword\(keyword,\s*instanceData\)' `
    'Base WEAP keyword checks must not pass modded TBO instance data into BGSKeywordForm::HasKeyword.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'instanceData\s*\?\s*instanceData->GetKeywordData\(\)\s*:\s*nullptr' `
    'Instance keywords must remain on the guarded TBO_InstanceData::GetKeywordData path.'

Require-Text 'src/physics-interaction/weapon/WeaponSupport.h' `
    'Base WEAP keywords and instance keyword data are read[\s\S]*separate guarded paths' `
    'Weapon support documentation must record why base and instance keyword authority are separated.'

if ($failures.Count -gt 0) {
    Write-Host 'Weapon support keyword safety source boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Weapon support keyword safety source boundary passed.'
