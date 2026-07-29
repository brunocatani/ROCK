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
    param([string]$RelativePath, [string]$Pattern, [string]$Message)
    if ((Read-Source $RelativePath) -notmatch $Pattern) {
        $failures.Add("$RelativePath`: $Message")
    }
}

function Reject-Text {
    param([string]$RelativePath, [string]$Pattern, [string]$Message)
    if ((Read-Source $RelativePath) -match $Pattern) {
        $failures.Add("$RelativePath`: $Message")
    }
}

Require-Text 'CMakeLists.txt' 'add_subdirectory\(\$\{COMMON_LIB_F4VR_PATH_SELECTED\} "CommonLibF4" EXCLUDE_FROM_ALL\)' `
    'ROCK must build directly against the selected CommonLibF4VR checkout.'
Require-Text 'CMakeLists.txt' 'CommonLibF4::CommonLibF4' `
    'ROCK must link the CommonLibF4 target directly.'
Reject-Text 'CMakeLists.txt' 'F4VR_COMMON_FRAMEWORK|F4VRCommon::framework' `
    'ROCK must not configure or link F4VR-CommonFramework.'
Reject-Text 'CMakeUserPresets.json.template' 'F4VR_COMMON_FRAMEWORK' `
    'Portable user presets must not expose a removed Framework path.'
Reject-Text 'vcpkg.json' 'cpptrace' `
    'ROCK must not retain the Framework-only cpptrace package.'

if (Test-Path -LiteralPath (Join-Path $Root '.gitmodules')) {
    $failures.Add('.gitmodules: ROCK must not retain the removed Framework submodule declaration.')
}

foreach ($requiredPath in @(
        'src/rock_support/Logger.h',
        'src/rock_support/ResourceUtils.cpp',
        'src/rock_support/Fo4VrRuntime.cpp',
        'src/rock_support/GameMenus.h',
        'src/rock_support/VRControllers.cpp',
        'third_party/openvr/openvr.h',
        'third_party/openvr/openvr_api.lib',
        'third_party/openvr/LICENSE'
    )) {
    if (-not (Test-Path -LiteralPath (Join-Path $Root $requiredPath))) {
        $failures.Add("$requiredPath`: required ROCK-owned support file is missing.")
    }
}

$productionFiles = Get-ChildItem -LiteralPath (Join-Path $Root 'src') -Recurse -File -Include *.h,*.cpp,*.inl
foreach ($file in $productionFiles) {
    $text = Get-Content -Raw -LiteralPath $file.FullName
    if ($text -match '#include\s*[<"](?:f4sevr|f4vr|vrcf|common)/' -or $text -match 'f4cf::') {
        $relative = [System.IO.Path]::GetRelativePath($Root, $file.FullName)
        $failures.Add("$relative`: production source still imports the removed Framework surface.")
    }
}

if ($failures.Count -gt 0) {
    $failures | ForEach-Object { Write-Error $_ }
    exit 1
}
