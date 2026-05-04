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

    $text = Get-Content -Raw -LiteralPath (Join-Path $Root $Path)
    if ($text -notmatch $Pattern) {
        $failures.Add($Message)
    }
}

function Reject-Text {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )

    $text = Get-Content -Raw -LiteralPath (Join-Path $Root $Path)
    if ($text -match $Pattern) {
        $failures.Add($Message)
    }
}

Require-Text 'src/physics-interaction/HavokOffsets.h' 'kFunc_BhkWorld_SetMotionRecursive\s*=\s*0x1DF95B0' 'Recursive SetMotion wrapper offset must be named as the bhkWorld NiAVObject wrapper.'
Require-Text 'src/physics-interaction/PhysicsRecursiveWrappers.h' 'bool recursive = false;' 'SetMotionCommand must name the native child-walk flag as recursive.'
Require-Text 'src/physics-interaction/PhysicsRecursiveWrappers.h' 'makeSetMotionCommand\(MotionPreset preset,\s*bool recursive,\s*bool force,\s*bool activate\)' 'SetMotion command helper must expose native bool order.'
Require-Text 'src/physics-interaction/PhysicsRecursiveWrappers.h' 'setMotionRecursive\(RE::NiAVObject\* root,\s*MotionPreset preset,\s*bool recursive,\s*bool force,\s*bool activate\)' 'SetMotion wrapper declaration must expose native bool order.'
Require-Text 'src/physics-interaction/PhysicsRecursiveWrappers.cpp' 'kFunc_BhkWorld_SetMotionRecursive' 'SetMotion wrapper must call the renamed recursive bhkWorld offset.'
Require-Text 'src/physics-interaction/PhysicsRecursiveWrappers.cpp' 'setMotion\(root,\s*toNativeMotionPreset\(preset\),\s*recursive,\s*force,\s*activate\)' 'SetMotion wrapper must pass recursive, force, activate in verified native order.'
Require-Text 'src/physics-interaction/PhysicsRecursiveWrappers.cpp' '#include "HavokRuntime.h"' 'Direct hknp body activation must route through the guarded Havok runtime boundary.'
Require-Text 'src/physics-interaction/PhysicsRecursiveWrappers.cpp' 'return havok_runtime::activateBody\(world,\s*bodyId\)' 'Recursive wrapper activation must use the body-slot-guarded Havok runtime helper.'
Require-Text 'src/physics-interaction/HandGrab.cpp' 'command\.recursive,\s*command\.force,\s*command\.activate' 'Motion restore commands must pass recursive first, not activate first.'

Reject-Text 'src/physics-interaction/HavokOffsets.h' 'kFunc_World_SetMotion\s*=' 'Ambiguous kFunc_World_SetMotion name must not remain.'
Reject-Text 'src/physics-interaction/PhysicsRecursiveWrappers.cpp' 'setMotion\(root,\s*toNativeMotionPreset\(preset\),\s*activate,\s*force,\s*propagate\)' 'SetMotion wrapper must not pass the old activate, force, propagate order.'
Reject-Text 'src/physics-interaction/PhysicsRecursiveWrappers.h' 'bool propagate' 'SetMotion wrapper must not keep the old propagate name for the verified recursive flag.'
Reject-Text 'src/physics-interaction/PhysicsRecursiveWrappers.cpp' 'REL::Offset\(offsets::kFunc_ActivateBody\)' 'PhysicsRecursiveWrappers must not call hknpWorld::ActivateBody without a readable body-slot guard.'
Reject-Text 'src/physics-interaction/HandGrab.cpp' 'REL::Offset\(offsets::kFunc_ActivateBody\)' 'HandGrab must not bypass the guarded activation wrapper.'

if ($failures.Count -gt 0) {
    Write-Host 'Physics recursive wrapper source boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Physics recursive wrapper source boundary passed.'
