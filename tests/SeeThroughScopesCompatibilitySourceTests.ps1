param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$failures = [System.Collections.Generic.List[string]]::new()

function Require-Text {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )

    $fullPath = Join-Path $Root $Path
    if (-not (Test-Path -LiteralPath $fullPath)) {
        $failures.Add($Message)
        return
    }

    $text = Get-Content -Raw -LiteralPath $fullPath
    if ($text -notmatch $Pattern) {
        $failures.Add($Message)
    }
}

Require-Text `
    'src/physics-interaction/weapon/SeeThroughScopesCompatibility.cpp' `
    'formHasSeeThroughScopesSourceFile' `
    'STS overlay suppression must be gated by each OMOD source-file chain, not only by global plugin detection.'

Require-Text `
    'src/physics-interaction/weapon/SeeThroughScopesCompatibility.cpp' `
    'form\.sourceFiles\.array' `
    'STS overlay suppression must inspect TESForm source files so vanilla/non-STS scopes keep native overlays.'

Require-Text `
    'src/physics-interaction/weapon/SeeThroughScopesCompatibility.cpp' `
    'if\s*\(\s*!formHasSeeThroughScopesSourceFile\(\*omod\)\s*\)\s*\{\s*continue;\s*\}' `
    'The OMOD overlay patch loop must skip records that were not sourced from a 3dscopes plugin.'

Require-Text `
    'src/physics-interaction/weapon/SeeThroughScopesCompatibility.cpp' `
    'See-Through Scopes OMOD records' `
    'The overlay patch log should identify that only STS-sourced OMOD records are patched.'

Require-Text `
    'data/config/ROCK.ini' `
    'Native scope overlays stay enabled for OMODs that are not sourced from those plugins\.' `
    'ROCK.ini must document mixed native/STS scope overlay behavior.'

if ($failures.Count -gt 0) {
    Write-Host 'See-through scopes compatibility source boundary failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'See-through scopes compatibility source boundary passed.' -ForegroundColor Green
