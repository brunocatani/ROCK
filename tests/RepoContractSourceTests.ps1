param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

# Repository contract checks. These verify structural project contracts that
# no C++ policy test can observe: the CommonLibF4VR-only dependency boundary,
# the ROCK.ini configuration-authority distribution contract. Input ownership
# and configuration values are covered by the C++ tests.

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$failures = [System.Collections.Generic.List[string]]::new()

function Read-Source {
    param([string]$RelativePath)

    $path = Join-Path $Root $RelativePath
    if (-not (Test-Path -LiteralPath $path)) {
        $failures.Add("Missing required file: $RelativePath")
        return ''
    }
    return Get-Content -Raw -LiteralPath $path
}

# --- CommonLibF4VR-only dependency boundary -------------------------------

$cmakeText = Read-Source 'CMakeLists.txt'
if ($cmakeText -notmatch 'CommonLibF4::CommonLibF4') {
    $failures.Add('CMakeLists.txt: ROCK must link the CommonLibF4 target directly.')
}
if ($cmakeText -match 'F4VR_COMMON_FRAMEWORK|F4VRCommon::framework') {
    $failures.Add('CMakeLists.txt: ROCK must not configure or link the removed F4VR-CommonFramework.')
}
if ((Read-Source 'vcpkg.json') -match 'cpptrace') {
    $failures.Add('vcpkg.json: ROCK must not retain the Framework-only cpptrace package.')
}
if (Test-Path -LiteralPath (Join-Path $Root '.gitmodules')) {
    $failures.Add('.gitmodules: ROCK must not retain the removed Framework submodule declaration.')
}

$productionFiles = Get-ChildItem -LiteralPath (Join-Path $Root 'src') -Recurse -File -Include *.h,*.cpp,*.inl
foreach ($file in $productionFiles) {
    $text = Get-Content -Raw -LiteralPath $file.FullName
    if ($text -match '#include\s*[<"](?:f4sevr|f4vr|vrcf|common)/' -or $text -match 'f4cf::') {
        $relative = [System.IO.Path]::GetRelativePath($Root, $file.FullName)
        $failures.Add("$relative`: production source still imports the removed Framework surface.")
    }
}

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
