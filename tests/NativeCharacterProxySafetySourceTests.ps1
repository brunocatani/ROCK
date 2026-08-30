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

$source = 'src/physics-interaction/native/NativeCharacterProxySafety.cpp'
$offsets = 'src/physics-interaction/native/HavokOffsets.h'

Require-Text $offsets `
    'kFunc_BhkCharProxyController_WorldAccessor\s*=\s*0x1E4DEC0[\s\S]*kFault_BhkCharProxyController_BodyFieldRead\s*=\s*0x1E4DEED' `
    'The guard must retain the verified FO4VR accessor and fault addresses.'
Require-Text $source `
    'kExpectedWorldAccessorEntry\{[\s\S]*0x40,\s*0x53[\s\S]*0x48,\s*0x83,\s*0xEC,\s*0x20[\s\S]*0x48,\s*0x8B,\s*0x01[\s\S]*0x48,\s*0x8D,\s*0x54,\s*0x24,\s*0x30' `
    'The entry hook must stay pinned to the verified whole-instruction FO4VR prologue.'
Require-OrderedText $source @(
    'captureVerifiedBodyFieldFault\(',
    'ExceptionCode\s*!=\s*EXCEPTION_ACCESS_VIOLATION',
    'ExceptionInformation\[0\]\s*!=\s*0',
    'exceptionAddress\s*!=\s*expectedAddress',
    'bodyAddress\s*>=\s*kMaximumNullDerivedReadAddress',
    'bodyAddress\s*%\s*kBodyStride\s*!=\s*0',
    'accessAddress\s*!=\s*bodyAddress\s*\+\s*kBodyFieldOffset',
    'EXCEPTION_EXECUTE_HANDLER'
) 'The exception filter must handle only the exact verified null-derived body-field read.'
Require-OrderedText $source @(
    '__try\s*\{',
    'original\(controller\)',
    '__except\s*\(captureVerifiedBodyFieldFault\(GetExceptionInformation\(\)\)\)',
    'result\s*=\s*nullptr',
    'return\s+false;'
) 'The native accessor must be enclosed by the exact SEH boundary.'
Require-OrderedText $source @(
    'if\s*\(invokeOriginal\(controller,\s*result\)\)',
    'return\s+result;',
    'recordSuppressedFault\(controller,\s*caller\)',
    'return\s+nullptr;'
) 'The handled teardown race must return the accessor native null result.'
Require-Text $source `
    'occurrence\s*<=\s*4\s*\|\|\s*std::has_single_bit\(occurrence\)' `
    'Repeated character-proxy teardown diagnostics must use bounded exponential logging.'
Require-Text 'src/ROCKMain.cpp' `
    'native_character_proxy_safety::install\(\)' `
    'Native character-proxy safety must install during plugin load.'

if ($failures.Count -gt 0) {
    $failures | ForEach-Object { Write-Error $_ }
    exit 1
}

Write-Host 'Native character-proxy safety source boundaries passed.'
