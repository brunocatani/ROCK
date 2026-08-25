param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

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

function Require-Pattern {
    param([string]$RelativePath, [string]$Pattern, [string]$Message)

    if ((Read-Source $RelativePath) -notmatch $Pattern) {
        $failures.Add($Message)
    }
}

function Reject-Pattern {
    param([string]$RelativePath, [string]$Pattern, [string]$Message)

    if ((Read-Source $RelativePath) -match $Pattern) {
        $failures.Add($Message)
    }
}

foreach ($removedPath in @(
        'data/config/ROCK.ini',
        'data/mod/ROCK_Config/ROCK.ini',
        'cmake/resources.rc.in',
        'src/resources.h')) {
    if (Test-Path -LiteralPath (Join-Path $Root $removedPath)) {
        $failures.Add("Removed ROCK configuration artifact still exists: $removedPath")
    }
}

if (-not (Test-Path -LiteralPath (Join-Path $Root 'data/config/ROCK_example.ini')) {
    $failures.Add('The sole Git-tracked ROCK_example.ini is missing.')
}

$exampleText = Read-Source 'data/config/ROCK_example.ini'
$configHeader = Read-Source 'src/RockConfig.h'
$configSource = Read-Source 'src/RockConfig.cpp'

Require-Pattern 'src/RockConfig.h' `
    'struct RockConfigValues[\s\S]*class RockConfig\s*:\s*public RockConfigValues' `
    'Loadable values must have one copyable compiled-default authority separate from runtime watcher state.'

Require-Pattern 'src/RockConfig.cpp' `
    'void RockConfig::resetToDefaults\(\)[\s\S]{0,180}static_cast<RockConfigValues&>\(\*this\)\s*=\s*RockConfigValues\{\};' `
    'Every reload must reset the complete loadable value set from its canonical member defaults.'

$exampleKeyMatches = [regex]::Matches($exampleText, '(?m)^\s*([A-Za-z][A-Za-z0-9]*)\s*=')
$exampleKeys = @($exampleKeyMatches | ForEach-Object { $_.Groups[1].Value })
$duplicateExampleKeys = @($exampleKeys | Group-Object | Where-Object Count -gt 1)
if ($duplicateExampleKeys.Count -ne 0) {
    $failures.Add("ROCK_example.ini contains duplicate keys: $($duplicateExampleKeys.Name -join ', ')")
}

$loaderKeys = @(
    [regex]::Matches($configSource, '"([bifs][A-Z][A-Za-z0-9]*)"') |
        ForEach-Object { $_.Groups[1].Value } |
        Sort-Object -Unique
)
$exampleUniqueKeys = @($exampleKeys | Sort-Object -Unique)
$catalogDifference = @(Compare-Object $loaderKeys $exampleUniqueKeys)
if ($catalogDifference.Count -ne 0) {
    $failures.Add("ROCK_example.ini and the loadable code catalog differ: $($catalogDifference.InputObject -join ', ')")
}

$expectedExampleDefaults = [ordered]@{
    iLogLevel = '2'
    fRightGrabAuthorityProxyOffsetYGameUnits = '-2.0'
    fLeftGrabAuthorityProxyOffsetYGameUnits = '-2.0'
    sHandPalmColliderDimensionScaleOverrides = ''
    fGrabReleaseHandCollisionDelaySeconds = '0.10'
    fGrabPinchCompactMaxExtentGameUnits = '8.0'
    bDebugDrawGrabPockets = 'false'
    iDebugWorldObjectOriginLogIntervalFrames = '120'
}
$exampleValues = @{}
foreach ($line in ($exampleText -split "`r?`n")) {
    if ($line -match '^\s*([A-Za-z][A-Za-z0-9]*)\s*=\s*(.*?)\s*$') {
        $exampleValues[$Matches[1]] = $Matches[2]
    }
}
foreach ($entry in $expectedExampleDefaults.GetEnumerator()) {
    if (-not $exampleValues.ContainsKey($entry.Key) -or $exampleValues[$entry.Key] -ne $entry.Value) {
        $actual = if ($exampleValues.ContainsKey($entry.Key)) { $exampleValues[$entry.Key] } else { '<missing>' }
        $failures.Add("ROCK_example.ini default mismatch for $($entry.Key): expected '$($entry.Value)', found '$actual'.")
    }
}

if ($exampleText -notmatch 'documents first-run and missing-key defaults[\s\S]*never loads, packages, deploys, or copies this file') {
    $failures.Add('ROCK_example.ini must state that it documents compiled defaults and has no runtime authority.')
}
if ($exampleText -match 'HIGGS|custom dynamic grab|dynamic weapon box') {
    $failures.Add('ROCK_example.ini retains retired authority or naming.')
}

Reject-Pattern 'CMakeLists.txt' `
    'ROCK_example\.ini|data/config/ROCK\.ini|resources\.rc' `
    'ROCK_example.ini must not be a build input, resource, or packaged file.'

Reject-Pattern 'src/RockConfig.cpp' `
    'ROCK_example\.ini|IDR_ROCK_INI|createFileFromResourceIfMissing|Data\\F4SE\\Plugins\\ROCK\.ini' `
    'Runtime configuration must not read the example, use an embedded INI, or retain a Data-folder fallback.'

Require-Pattern 'src/RockConfig.cpp' `
    'getPathInDocuments\([\s\S]{0,160}My Games\\Fallout4VR\\ROCK_Config\\ROCK\.ini' `
    'ROCK must resolve only the production My Games ROCK.ini path.'

Require-Pattern 'src/RockConfig.cpp' `
    'class RockIniReader[\s\S]*SetBoolValue[\s\S]*SetLongValue[\s\S]*SetDoubleValue[\s\S]*SetValue' `
    'First-run generation must materialize every supported scalar type through the normal loader catalog.'

Require-Pattern 'src/RockConfig.cpp' `
    'createDefaultIniIfMissing\([\s\S]*resetToDefaults\(\)[\s\S]*readValuesFromIni\(defaults, true\)[\s\S]*SaveFile[\s\S]*filesystem::rename' `
    'Missing production configuration must be generated from compiled defaults and published without overwriting an existing file.'

Reject-Pattern 'src/RockConfig.h' `
    '_fileWatchInitThread|_ignoreNextIniFileChange|suppressNextFileWatchReload' `
    'Configuration watching must not retain the redundant wrapper thread or duplicate self-write suppression flag.'

Require-Pattern 'src/RockConfig.cpp' `
    'targetExists[\s\S]{0,100}return true;[\s\S]*Loading the only active ROCK config' `
    'An existing production ROCK.ini must be loaded without replacement or default merging.'

Reject-Pattern 'src/rock_support/ResourceUtils.cpp' `
    'FindResource|LoadResource|LockResource|createFileFromResourceIfMissing' `
    'Generic resource helpers must not retain removed INI-resource creation code.'

if ($failures.Count -gt 0) {
    Write-Host 'ROCK configuration authority source boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'ROCK configuration authority source boundary passed.'
