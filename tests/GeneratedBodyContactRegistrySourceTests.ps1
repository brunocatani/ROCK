param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$failures = [System.Collections.Generic.List[string]]::new()

function Require-Text {
    param(
        [string]$RelativePath,
        [string]$Pattern,
        [string]$Message
    )

    $path = Join-Path $Root $RelativePath
    $text = Get-Content -Raw -LiteralPath $path
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

    $path = Join-Path $Root $RelativePath
    $text = Get-Content -Raw -LiteralPath $path
    if ($text -match $Pattern) {
        $failures.Add("$RelativePath`: $Message")
    }
}


Require-Text 'src/physics-interaction/contact/GeneratedBodyContactRegistry.h' 'std::atomic<std::uint64_t> _publicationVersion' 'Generated-body callback registry must use atomic publication versioning.'
Require-Text 'src/physics-interaction/contact/GeneratedBodyContactRegistry.h' 'std::sort' 'Generated-body callback registry must publish sorted fixed storage for bounded lookup.'


if ($failures.Count -gt 0) {
    Write-Host 'GeneratedBodyContactRegistrySourceTests failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'GeneratedBodyContactRegistrySourceTests passed.' -ForegroundColor Green
