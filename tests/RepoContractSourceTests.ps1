param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

# Repository contract check: runtime INIs must never ship, deploy or embed
# (ROCK configuration contract). Catalog/default parity belongs to
# ConfigurationStoreTests.

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$failures = [System.Collections.Generic.List[string]]::new()

# Distribution checks execute the production preflight. Compiled catalog/default
# parity belongs to ConfigurationStoreTests; input ownership to adapter tests.
$guard = Join-Path $Root 'cmake/VerifyDistributionInputs.cmake'
& cmake "-DINPUT_TREES=$Root/data/mod" -P $guard
if ($LASTEXITCODE -ne 0) { $failures.Add('The deployment input tree contains an INI.') }

$fixture = Join-Path ([System.IO.Path]::GetTempPath()) ('rock-distribution-' + [guid]::NewGuid())
try {
    $null = New-Item -ItemType Directory -Path (Join-Path $fixture 'nested') -Force
    $resource = Join-Path $fixture 'version.rc'
    Set-Content -LiteralPath $resource -Value '1 VERSIONINFO'
    & cmake "-DINPUT_TREES=$fixture" -P $guard
    if ($LASTEXITCODE -ne 0) { $failures.Add('Generic resources must be accepted.') }
    $ini = Join-Path $fixture 'nested/ROCK_Developer.INI'
    Set-Content -LiteralPath $ini -Value '[Developer]'
    & cmake "-DINPUT_TREES=$fixture" -P $guard *> $null
    if ($LASTEXITCODE -eq 0) { $failures.Add('Nested INIs must fail the distribution preflight.') }
    Remove-Item -LiteralPath $ini
    Set-Content -LiteralPath $resource -Value '101 RCDATA "outside/ROCK_example.ini"'
    & cmake "-DINPUT_FILES=$resource" -P $guard *> $null
    if ($LASTEXITCODE -eq 0) { $failures.Add('Embedded reference INIs must fail the resource preflight.') }
} finally {
    $resolvedFixture = [System.IO.Path]::GetFullPath($fixture)
    $tempRoot = [System.IO.Path]::GetFullPath([System.IO.Path]::GetTempPath()).TrimEnd('\', '/') + [System.IO.Path]::DirectorySeparatorChar
    if (-not $resolvedFixture.StartsWith($tempRoot, [System.StringComparison]::OrdinalIgnoreCase)) {
        throw "Fixture escaped temporary root: $resolvedFixture"
    }
    Remove-Item -LiteralPath $resolvedFixture -Recurse -Force
}

if ($failures.Count -gt 0) {
    Write-Host 'ROCK repository contract checks failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'ROCK repository contract checks passed.'
