param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
$failures = [System.Collections.Generic.List[string]]::new()

function Require-Text {
    param([string]$Path, [string]$Pattern, [string]$Message)
    $text = Get-Content -Raw -LiteralPath (Join-Path $Root $Path)
    if ($text -notmatch $Pattern) {
        $failures.Add($Message)
    }
}

function Require-OrderedText {
    param([string]$Path, [string[]]$Patterns, [string]$Message)
    $text = Get-Content -Raw -LiteralPath (Join-Path $Root $Path)
    $offset = 0
    foreach ($pattern in $Patterns) {
        $remaining = $text.Substring($offset)
        $match = [regex]::Match(
            $remaining,
            $pattern,
            [System.Text.RegularExpressions.RegexOptions]::Singleline)
        if (-not $match.Success) {
            $failures.Add($Message)
            return
        }
        $offset += $match.Index + $match.Length
    }
}

$source = 'src/physics-interaction/native/NativeCollisionFilterSafety.cpp'
$offsets = 'src/physics-interaction/native/HavokOffsets.h'

Require-Text $offsets `
    'kFunc_BhkNPCollisionObject_GetCollisionFilterInfo\s*=\s*0x1E08D60[\s\S]*kFault_BhkNPCollisionObject_BodyFilterRead\s*=\s*0x1E08DF3' `
    'The guard must retain the verified FO4VR function and fault addresses.'
Require-Text $source `
    'kExpectedFilterInfoEntry\{[\s\S]*0x48,\s*0x89,\s*0x5C,\s*0x24,\s*0x18[\s\S]*0x48,\s*0x89,\s*0x7C,\s*0x24,\s*0x20[\s\S]*0x41,\s*0x56[\s\S]*0x48,\s*0x83,\s*0xEC,\s*0x20' `
    'The entry hook must stay pinned to the verified whole-instruction FO4VR prologue.'
Require-OrderedText $source @(
    'captureVerifiedBodyArrayFault\(',
    'ExceptionCode\s*!=\s*EXCEPTION_ACCESS_VIOLATION',
    'ExceptionInformation\[0\]\s*!=\s*0',
    'exceptionAddress\s*!=\s*expectedAddress',
    'bodyAddress\s*>=\s*kMaximumNullDerivedReadAddress',
    'accessAddress\s*!=\s*bodyAddress\s*\+\s*kBodyFilterInfoOffset',
    'EXCEPTION_EXECUTE_HANDLER'
) 'The exception filter must handle only the exact verified null-derived body read.'
Require-OrderedText $source @(
    '__try\s*\{',
    'original\(collisionObject,\s*filterInfoOut\)',
    '__except\s*\(captureVerifiedBodyArrayFault\(GetExceptionInformation\(\)\)\)',
    'return\s+false;'
) 'The native lookup must be enclosed by the exact SEH boundary.'
Require-OrderedText $source @(
    't_invalidFilterInfo\s*=\s*kInvalidFilterInfo',
    'native_memory::tryWriteValue\(',
    'recordSuppressedFault\(',
    'return\s+outputStored\s*\?\s*filterInfoOut\s*:\s*&t_invalidFilterInfo'
) 'The handled fault must return the native invalid-filter value through a writable output or safe fallback.'
Require-Text $source `
    'occurrence\s*<=\s*4\s*\|\|\s*std::has_single_bit\(occurrence\)' `
    'Repeated collision-filter teardown diagnostics must use bounded exponential logging.'
Require-Text 'src/ROCKMain.cpp' `
    'native_collision_filter_safety::install\(\)' `
    'Native collision-filter safety must install during plugin load.'

if ($failures.Count -gt 0) {
    $failures | ForEach-Object { Write-Error $_ }
    exit 1
}

Write-Host 'Native collision-filter safety source boundaries passed.'
