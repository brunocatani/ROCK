# ROCK's runtime namespace is intentionally owned by ROCK, not by FRIK. The
# project still depends on frik::api for FRIK integration, but keeping internal
# ROCK systems under rock avoids presenting implementation classes as part of
# FRIK and makes API/provider boundaries easier to reason about.

$ErrorActionPreference = "Stop"

$repoRoot = Split-Path -Parent $PSScriptRoot

$sourceRoots = @(
    Join-Path $repoRoot "src",
    Join-Path $repoRoot "tests"
)

$files = foreach ($root in $sourceRoots) {
    Get-ChildItem -Path $root -Recurse -File -Include *.h,*.hpp,*.cpp,*.inl,*.ps1
}

$legacyNamespace = "frik" + "::" + "rock"

$forbiddenPatterns = @(
    ("namespace\s+" + [regex]::Escape($legacyNamespace) + "\b"),
    ("\b" + [regex]::Escape($legacyNamespace) + "::"),
    ("using\s+namespace\s+" + [regex]::Escape($legacyNamespace) + "\b")
)

$violations = @()

foreach ($file in $files) {
    $relativePath = [System.IO.Path]::GetRelativePath($repoRoot, $file.FullName)
    $lines = Get-Content -Path $file.FullName

    for ($index = 0; $index -lt $lines.Count; ++$index) {
        foreach ($pattern in $forbiddenPatterns) {
            if ($lines[$index] -match $pattern) {
                $violations += "{0}:{1}: {2}" -f $relativePath, ($index + 1), $lines[$index].Trim()
            }
        }
    }
}

if ($violations.Count -gt 0) {
    Write-Error ("ROCK runtime code must use namespace rock, not {0}:`n{1}" -f $legacyNamespace, ($violations -join "`n"))
}

Write-Host "ROCK namespace source boundary test passed."
