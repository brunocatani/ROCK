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
        $failures.Add("$Path`: $Message")
    }
}

$factoryHeader = 'src/physics-interaction/native/NativeNiNodeFactory.h'
$factorySource = 'src/physics-interaction/native/NativeNiNodeFactory.cpp'
$weaponSource = 'src/physics-interaction/weapon/WeaponCollision.cpp'

Require-Text $factoryHeader `
    'RE::NiPointer<RE::NiNode>\s+createEngineNiNode\(std::uint16_t\s+childCapacity\)' `
    'The native factory must return a strong NiPointer instead of exposing unowned constructed storage.'
Require-Text $factorySource `
    'guardedCopyFromMemory[\s\S]*RUNTIME_VR_1_2_72[\s\S]*kFunc_BethesdaAlloc[\s\S]*kFunc_BethesdaAllocatorInit[\s\S]*kFunc_NiNode_Ctor' `
    'The hardcoded FO4VR constructor must retain exact executable-version and live-byte identity gates.'
Require-Text $factorySource `
    'kFunc_BethesdaAlloc,\s*std::array<std::uint8_t,\s*10>\{\s*0x48,\s*0x89,\s*0x5C,\s*0x24,\s*0x10,\s*0x48,\s*0x89,\s*0x6C,\s*0x24,\s*0x18\s*\}' `
    'The Bethesda allocator gate must retain its independently verified Fallout4VR.exe 1.2.72 entry bytes.'
Require-Text $factorySource `
    'kFunc_BethesdaAllocatorInit,\s*std::array<std::uint8_t,\s*12>\{\s*0x57,\s*0x48,\s*0x83,\s*0xEC,\s*0x20,\s*0x44,\s*0x8B,\s*0x05,\s*0xEF,\s*0xAC,\s*0xD0,\s*0x04\s*\}' `
    'The Bethesda allocator initializer gate must retain its independently verified Fallout4VR.exe 1.2.72 entry bytes.'
Require-Text $factorySource `
    'kFunc_NiNode_Ctor,\s*std::array<std::uint8_t,\s*10>\{\s*0x48,\s*0x89,\s*0x5C,\s*0x24,\s*0x08,\s*0x48,\s*0x89,\s*0x74,\s*0x24,\s*0x10\s*\}' `
    'The native NiNode constructor gate must retain its independently verified Fallout4VR.exe 1.2.72 entry bytes.'
Require-Text $factorySource `
    'static_assert\(sizeof\(RE::NiNode\)\s*==\s*offsets::kNiNodeSize\)[\s\S]*static_assert\(alignof\(RE::NiNode\)\s*==\s*offsets::kNiNodeAlignment\)' `
    'The factory must refuse to compile when the CommonLib VR NiNode size or alignment diverges from the verified engine layout.'
Require-Text $factorySource `
    'RE::aligned_alloc\(alignof\(RE::NiNode\),\s*sizeof\(RE::NiNode\)\)[\s\S]*std::memset\([\s\S]*constructor\(storage,\s*childCapacity\)[\s\S]*RE::NiPointer<RE::NiNode>\{\s*node\s*\}' `
    'NiNode storage and its child array must be allocated and constructed entirely in the engine domain before RAII adoption.'
Require-Text $weaponSource `
    'native_scene::createEngineNiNode\(1\)[\s\S]*if\s*\(!enrichmentContainer\)[\s\S]*enrichmentContainer->name' `
    'OMOD physical enrichment must use the engine-native factory and fail closed before dereferencing an unavailable container.'

$unsafePatterns = @(
    'new\s*(?:\([^\)]*\)\s*)?RE::NiNode\b',
    'make_nismart\s*<\s*RE::NiNode\s*>',
    'make_unique\s*<\s*RE::NiNode\s*>',
    'make_shared\s*<\s*RE::NiNode\s*>',
    'construct_at\s*<\s*RE::NiNode\s*>',
    'RE::NiNode\s+[A-Za-z_][A-Za-z0-9_]*\s*[\{\(]'
)

Get-ChildItem -LiteralPath (Join-Path $Root 'src') -Recurse -File |
    Where-Object { $_.Extension -in @('.cpp', '.h', '.inl') } |
    ForEach-Object {
        $relativePath = [System.IO.Path]::GetRelativePath($Root, $_.FullName)
        $text = Get-Content -Raw -LiteralPath $_.FullName
        foreach ($pattern in $unsafePatterns) {
            if ($text -match $pattern) {
                $failures.Add("$relativePath`: Direct CommonLib NiNode construction is forbidden; use native_scene::createEngineNiNode().")
                break
            }
        }
    }

if ($failures.Count -gt 0) {
    Write-Host 'NativeNiNodeFactorySourceTests failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'NativeNiNodeFactorySourceTests passed.' -ForegroundColor Green
