param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

# Repository contract checks. These verify structural project contracts that
# no C++ policy test can observe: the CommonLibF4VR-only dependency boundary,
# the ROCK.ini configuration-authority packaging contract, and negative guards
# for specific past bugs. They intentionally do not pin source wording, call
# order, identifier names, or comment text.

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

# --- ROCK.ini configuration-authority packaging contract ------------------
# Runtime configuration is consumer ROCK.ini plus optional developer overrides.
# Both examples are human references, never build inputs or runtime sources.

foreach ($removedPath in @(
        'data/config/ROCK.ini',
        'data/mod/ROCK_Config/ROCK.ini',
        'cmake/resources.rc.in',
        'src/resources.h')) {
    if (Test-Path -LiteralPath (Join-Path $Root $removedPath)) {
        $failures.Add("Shipped or embedded ROCK configuration artifact exists: $removedPath")
    }
}

foreach ($example in @('ROCK_example.ini', 'ROCK_Developer_example.ini')) {
    if (-not (Test-Path -LiteralPath (Join-Path $Root ('data/config/' + $example)))) {
        $failures.Add("Missing configuration reference: $example")
    }
}

if ($cmakeText -match 'ROCK_(Developer_)?example\.ini|data/config/ROCK(_Developer)?\.ini|resources\.rc') {
    $failures.Add('CMakeLists.txt: ROCK_example.ini must not be a build input, resource, or packaged file.')
}

$configSource = (Read-Source 'src/RockConfig.cpp') + (Read-Source 'src/config/ConfigurationStore.cpp')
if ($configSource -match 'ROCK_(Developer_)?example\.ini|IDR_ROCK_INI|createFileFromResourceIfMissing|Data\\+F4SE\\+Plugins\\+ROCK\.ini') {
    $failures.Add('src/RockConfig.cpp: runtime configuration must not read the example, use an embedded INI, or retain a Data-folder fallback.')
}
if ((Read-Source 'src/rock_support/ResourceUtils.cpp') -match 'FindResource|LoadResource|LockResource') {
    $failures.Add('src/rock_support/ResourceUtils.cpp: resource helpers must not retain removed INI-resource creation code.')
}

# The two examples together must mirror the loadable key catalog exactly.
$exampleText = (Read-Source 'data/config/ROCK_example.ini') + "`n" + (Read-Source 'data/config/ROCK_Developer_example.ini')
$exampleKeyMatches = [regex]::Matches($exampleText, '(?m)^\s*([A-Za-z][A-Za-z0-9]*)\s*=')
$exampleKeys = @($exampleKeyMatches | ForEach-Object { $_.Groups[1].Value })
$duplicateExampleKeys = @($exampleKeys | Group-Object | Where-Object Count -gt 1)
if ($duplicateExampleKeys.Count -ne 0) {
    $failures.Add("The configuration examples contain duplicate keys: $($duplicateExampleKeys.Name -join ', ')")
}
$loaderSource = Read-Source 'src/RockConfigLoad.cpp'
$loaderKeys = @(
    [regex]::Matches($loaderSource, '"([bifs][A-Z][A-Za-z0-9]*)"') |
        ForEach-Object { $_.Groups[1].Value } |
        Sort-Object -Unique
)
$exampleUniqueKeys = @($exampleKeys | Sort-Object -Unique)
$catalogDifference = @(Compare-Object $loaderKeys $exampleUniqueKeys)
if ($catalogDifference.Count -ne 0) {
    $failures.Add("The configuration examples and loadable code catalog differ: $($catalogDifference.InputObject -join ', ')")
}

# --- Negative guards for specific past bugs -------------------------------
# ROCK controller identity must never be reinterpreted through the Fallout 4
# VR native left-handed-mode setting.

$interactionFiles = Get-ChildItem -LiteralPath (Join-Path $Root 'src/physics-interaction') -Recurse -File -Include *.h,*.cpp,*.inl
foreach ($file in $interactionFiles) {
    if ((Get-Content -Raw -LiteralPath $file.FullName) -match 'isLeftHandedMode') {
        $relative = [System.IO.Path]::GetRelativePath($Root, $file.FullName)
        $failures.Add("$relative`: ROCK must not reinterpret controller identity through native handedness.")
    }
}

if ($failures.Count -gt 0) {
    Write-Host 'ROCK repository contract checks failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'ROCK repository contract checks passed.'
