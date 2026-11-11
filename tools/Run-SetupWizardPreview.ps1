<#
.SYNOPSIS
Builds the setup wizard preview and starts it.

.DESCRIPTION
ROCKSetupWizardPreview runs ROCK's setup wizard on the desktop: the wizard's own
panel code, RPS UI's panel host and ROCK's settings store over a sandbox folder.
It is outside ROCK's default build, so this script builds it first (an
incremental build when nothing changed) in build-fast, Release.

.PARAMETER Settings
The sandbox folder ROCK's settings store reads and writes. Defaults to
build-fast/setup-wizard-preview. Never point it at the game's Mods_Config\ROCK.

.PARAMETER Capture
Walks every page offscreen and writes one PNG per page into this folder instead
of opening the window. Without -Settings the walk uses a fresh sandbox of its
own inside that folder.

.PARAMETER NoBuild
Starts the last build without building.
#>
param(
    [string]$Settings,
    [string]$Capture,
    [switch]$NoBuild
)

$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$build = Join-Path $root 'build-fast'
$exe = Join-Path $build 'Release/ROCKSetupWizardPreview.exe'

if (-not $NoBuild) {
    cmake --build $build --config Release --target ROCKSetupWizardPreview
    if ($LASTEXITCODE -ne 0) { throw "ROCKSetupWizardPreview build failed ($LASTEXITCODE)" }
}
if (-not (Test-Path $exe)) { throw "Not built: $exe" }

$arguments = @()
if ($Settings) { $arguments += @('--settings', ('"' + [IO.Path]::GetFullPath($Settings) + '"')) }
if ($Capture) {
    $arguments += @('--capture', ('"' + [IO.Path]::GetFullPath($Capture) + '"'))
    $process = Start-Process -FilePath $exe -ArgumentList $arguments -Wait -PassThru
    if ($process.ExitCode -ne 0) { throw "Capture failed ($($process.ExitCode))" }
    Write-Host "Pages written to $Capture"
} else {
    if ($arguments.Count) { Start-Process -FilePath $exe -ArgumentList $arguments }
    else { Start-Process -FilePath $exe }
}
