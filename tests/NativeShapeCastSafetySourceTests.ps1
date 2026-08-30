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

$source = 'src/physics-interaction/native/NativeShapeCastSafety.cpp'
$offsets = 'src/physics-interaction/native/HavokOffsets.h'
$main = 'src/ROCKMain.cpp'

Require-Text $offsets `
    'kHknpWorld_CollisionQueryDispatcher\s*=\s*0x200[\s\S]*kVtable_HknpWorldDestroying\s*=\s*0x2DFC540[\s\S]*kFunc_HknpWorld_CastShape\s*=\s*0x15A6C00[\s\S]*kFunc_HknpCollisionQueryDispatcherBase_CastShape\s*=\s*0x15FFC30' `
    'The guard must retain the raw-disassembly verified FO4VR world and function offsets.'
Require-Text $source `
    'kExpectedWorldCastShapeEntry\{[\s\S]*0x48,\s*0x89,\s*0x5C,\s*0x24,\s*0x08[\s\S]*kExpectedDispatcherCastShapeEntry\{[\s\S]*0x4C,\s*0x8B,\s*0xDC[\s\S]*0x57,[\s\S]*0x41,\s*0x54' `
    'Both entry hooks must remain pinned to verified whole-instruction FO4VR prologues.'
Require-OrderedText $source @(
    'onWorldCastShape\(',
    'tryReadFastWorldState\(world,\s*worldState\)',
    'worldState\.destroying\s*\|\|[\s\S]{0,120}worldState\.dispatcher\s*==\s*0',
    'recordSuppressedCast\(',
    'return;',
    's_originalWorldCastShape'
) 'The world entry hook must fail closed before native dispatch when teardown or a missing dispatcher is observed.'
Require-OrderedText $source @(
    'onDispatcherCastShape\(',
    'if\s*\(!dispatcher\)',
    'recordSuppressedCast\(',
    'return;',
    's_originalDispatcherCastShape'
) 'The exact dispatcher boundary must suppress a null dispatcher before invoking the original function.'
Require-Text $source `
    'occurrence\s*<=\s*4\s*\|\|\s*std::has_single_bit\(occurrence\)' `
    'Repeated teardown diagnostics must use bounded exponential logging.'
Require-Text $main `
    'native_shape_cast_safety::install\(\)' `
    'Native shape-cast safety must install during plugin load before gameplay.'

if ($failures.Count -gt 0) {
    $failures | ForEach-Object { Write-Error $_ }
    exit 1
}

Write-Host 'Native shape-cast safety source boundaries passed.'
