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

$source = 'src/physics-interaction/native/NativeRagdollSafety.cpp'
$offsets = 'src/physics-interaction/native/HavokOffsets.h'

Require-Text $offsets `
    'kHknpWorld_ConstraintArrayPtr\s*=\s*0x128[\s\S]*kHknpWorld_ConstraintCount\s*=\s*0x130[\s\S]*kFunc_HkbnpRagdollInterface_UpdateConstraints\s*=\s*0x17B74B0[\s\S]*kFault_HkbnpRagdollInterface_ConstraintRead\s*=\s*0x17B7534' `
    'The guard must retain the raw-disassembly verified FO4VR constraint layout and fault addresses.'
Require-Text $source `
    'kExpectedUpdateConstraintsEntry\{[\s\S]*0x48,\s*0x89,\s*0x4C,\s*0x24,\s*0x08[\s\S]*0x41,\s*0x56[\s\S]*0x41,\s*0x57[\s\S]*0x48,\s*0x83,\s*0xEC,\s*0x28' `
    'The entry hook must stay pinned to the verified whole-instruction FO4VR prologue.'
Require-OrderedText $source @(
    'captureVerifiedTeardownFault\(',
    'ExceptionCode\s*!=\s*EXCEPTION_ACCESS_VIOLATION',
    'ExceptionInformation\[0\]\s*!=\s*0',
    'exceptionAddress\s*!=\s*expectedAddress',
    'accessAddress\s*>=\s*kMaximumNullDerivedReadAddress',
    'context->R12\)\s*!=\s*accessAddress',
    'EXCEPTION_EXECUTE_HANDLER'
) 'The exception filter must handle only the exact verified null-derived read fault.'
Require-OrderedText $source @(
    '__try\s*\{',
    'original\(ragdollInterface\)',
    '__except\s*\(captureVerifiedTeardownFault\(GetExceptionInformation\(\)\)\)',
    'return\s+false;'
) 'The native call must be enclosed by the exact SEH boundary.'
Require-Text $source `
    'occurrence\s*<=\s*4\s*\|\|\s*std::has_single_bit\(occurrence\)' `
    'Repeated ragdoll teardown diagnostics must use bounded exponential logging.'
Require-Text 'src/ROCKMain.cpp' `
    'native_ragdoll_safety::install\(\)' `
    'Native ragdoll safety must install during plugin load.'

if ($failures.Count -gt 0) {
    $failures | ForEach-Object { Write-Error $_ }
    exit 1
}

Write-Host 'Native ragdoll safety source boundaries passed.'
