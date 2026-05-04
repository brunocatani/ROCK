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

Require-Text 'src/physics-interaction/native/HavokRuntime.h' 'bool activateBody\(RE::hknpWorld\* world,\s*std::uint32_t bodyId\)' 'Havok runtime must expose guarded hknp body activation.'
Require-Text 'src/physics-interaction/native/HavokRuntime.cpp' 'getReadableBodySlot\(RE::hknpWorld\* world,\s*RE::hknpBodyId bodyId\)' 'Havok runtime must separate readable body-slot validation from usable-motion validation.'
Require-Text 'src/physics-interaction/native/HavokRuntime.cpp' 'bool setFilterInfo[\s\S]{0,260}auto\* body = getReadableBodySlot\(world,\s*bodyId\)' 'SetBodyCollisionFilterInfo must verify the hknp body slot before calling the native direct-index writer.'
Require-Text 'src/physics-interaction/native/HavokRuntime.cpp' 'bool setBodyVelocityDeferred[\s\S]{0,320}auto\* body = getBody\(world,\s*RE::hknpBodyId\{ bodyId \}\)' 'SetBodyVelocityDeferred must verify the hknp body slot before calling the native direct-index writer.'
Require-Text 'src/physics-interaction/native/HavokRuntime.cpp' 'bool setBodyVelocityDeferred[\s\S]{0,520}getMotion\(world,\s*body->motionIndex\)' 'SetBodyVelocityDeferred must verify the target motion slot before calling the native writer.'
Require-Text 'src/physics-interaction/native/HavokRuntime.cpp' 'bool activateBody[\s\S]{0,260}auto\* body = getReadableBodySlot\(world,\s*RE::hknpBodyId\{ bodyId \}\)' 'hknpWorld::ActivateBody must verify the body slot before calling the native direct-index writer.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionContacts.inl' 'callbackWorld = reinterpret_cast<std::uintptr_t>\(\*worldPtrHolder\)' 'Contact callback must read the callback hknpWorld from the verified FO4VR hkSignal ABI slot.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionContacts.inl' 'evaluateCallbackAcceptance\(snapshot,\s*callbackWorld\)' 'Contact callback acceptance must use the actual callback world.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionContacts.inl' 'bodySlotLooksReadable\(world,\s*RE::hknpBodyId\{ bodyIdA \}\)[\s\S]{0,180}bodySlotLooksReadable\(world,\s*RE::hknpBodyId\{ bodyIdB \}\)' 'Contact processing must validate both event body slots before extracting native contact points.'

Reject-Text 'src/physics-interaction/native/HavokRuntime.cpp' 'bool setFilterInfo[\s\S]{0,180}if \(!world \|\| !isValidBodyId\(bodyId\)\)' 'SetBodyCollisionFilterInfo must not rely only on a non-null world and non-invalid body id.'
Reject-Text 'src/physics-interaction/native/HavokRuntime.cpp' 'bool setBodyVelocityDeferred[\s\S]{0,220}if \(!world \|\| bodyId == body_frame::kInvalidBodyId\)' 'SetBodyVelocityDeferred must not rely only on a non-null world and non-invalid body id.'
Reject-Text 'src/physics-interaction/core/PhysicsInteractionContacts.inl' 'evaluateCallbackAcceptance\(snapshot,\s*0\)' 'Contact callbacks must not force subscribed-world fallback by passing a zero callback world.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'REL::Offset\(offsets::kFunc_ActivateBody\)' 'HandGrab must not bypass the guarded activation helper.'

if ($failures.Count -gt 0) {
    Write-Host 'Havok runtime native boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Havok runtime native boundary passed.'
