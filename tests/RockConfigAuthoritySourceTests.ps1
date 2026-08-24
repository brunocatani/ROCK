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
