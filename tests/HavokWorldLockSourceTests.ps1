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

function Require-OrderedText {
    param(
        [string]$Path,
        [string[]]$Patterns,
        [string]$Message
    )

    $fullPath = Join-Path $Root $Path
    if (-not (Test-Path -LiteralPath $fullPath)) {
        $failures.Add($Message)
        return
    }

    $text = Get-Content -Raw -LiteralPath $fullPath
    $offset = 0
    foreach ($pattern in $Patterns) {
        $remaining = $text.Substring($offset)
        $match = [regex]::Match($remaining, $pattern, [System.Text.RegularExpressions.RegexOptions]::Singleline)
        if (-not $match.Success) {
            $failures.Add($Message)
            return
        }
        $offset += $match.Index + $match.Length
    }
}

# The hknp broadphase cast dereferences candidate body shapes without a null
# check; every ROCK-issued cast must hold the world read lock across the native
# call (crash family: castShape AV at [shape+0x14], 2026-07-12/13). See
# Docs/ROCK/lessons/2026-07-13-hknp-world-query-lock-discipline.md.

# All world shape casts must route through the locked wrapper. A direct
# world->CastShape call anywhere else bypasses the lock and reintroduces the
# crash class.
$srcDir = Join-Path $Root 'src'
$directCastFiles = Get-ChildItem -Path $srcDir -Recurse -Include '*.cpp', '*.h', '*.inl' |
    Where-Object { $_.Name -ne 'PhysicsShapeCast.cpp' } |
    Where-Object { (Get-Content -Raw -LiteralPath $_.FullName) -match '->CastShape\(' }
foreach ($file in $directCastFiles) {
    $failures.Add("Direct world->CastShape call outside the locked wrapper: $($file.FullName)")
}

if ($failures.Count -gt 0) {
    Write-Host 'Havok world lock source boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Havok world lock source boundary passed.'
