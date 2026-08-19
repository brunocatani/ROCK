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

function Reject-Text {
    param([string]$Path, [string]$Pattern, [string]$Message)

    $text = Get-Content -Raw -LiteralPath (Join-Path $Root $Path)
    if ($text -match $Pattern) {
        $failures.Add($Message)
    }
}

Require-Text 'src/physics-interaction/hand/HandVisual.h' `
    'struct VisualReturnTransition[\s\S]*lastApplied[\s\S]*computeVisualReturnDuration[\s\S]*\(std::max\)\(linearDuration, angularDuration\)[\s\S]*advanceVisualReturn' `
    'The shared return transition must retain its last applied pose and choose the greater translation/rotation duration.'

Require-Text 'src/RockConfig.cpp' `
    'bWeaponVisualReturnEnabled[\s\S]*fWeaponVisualReturnTimeMin[\s\S]*fWeaponVisualReturnTimeMax[\s\S]*fWeaponVisualReturnMinDistance[\s\S]*fWeaponVisualReturnMaxDistance[\s\S]*fWeaponVisualReturnMinAngleDegrees[\s\S]*fWeaponVisualReturnMaxAngleDegrees' `
    'Equipped-weapon visual return must have an independently loaded and clamped setting family.'
Require-Text 'src/RockConfig.cpp' `
    'bGrabHandReturnEnabled[\s\S]*fGrabHandReturnTimeMin[\s\S]*fGrabHandReturnTimeMax[\s\S]*fGrabHandReturnMinDistance[\s\S]*fGrabHandReturnMaxDistance[\s\S]*fGrabHandReturnMinAngleDegrees[\s\S]*fGrabHandReturnMaxAngleDegrees' `
    'Generic grabbed-hand return must have an independently loaded and clamped setting family.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'processHand\(_rightHand,\s*false\);[\s\S]*processHand\(_leftHand,\s*true\);[\s\S]*updateGrabVisualReturn\(\s*frame\.right\.rawHandWorld[\s\S]*updateGrabVisualReturn\(\s*frame\.left\.rawHandWorld' `
    'Empty-hand generic returns must advance after normal input processing for both physical hands.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.h' `
    'struct ReturningWeaponVisualState[\s\S]*VisualReturnTransition[\s\S]*weaponGenerationKey[\s\S]*equippedWeaponOwnershipKey[\s\S]*nativeBaselineLocal' `
    'Weapon return must be a generation-bound visual overlay with its own native local baseline.'
Reject-Text 'src/physics-interaction/weapon/TwoHandedGrip.h' `
    'enum class TwoHandedState[\s\S]{0,240}Returning' `
    'Visual return must not remain gameplay/manual grip ownership in TwoHandedState.'


foreach ($configPath in @('data/config/ROCK.ini', 'data/mod/ROCK_Config/ROCK.ini')) {
    Require-Text $configPath `
        'bWeaponVisualReturnEnabled\s*=\s*true[\s\S]*fWeaponVisualReturnMaxAngleDegrees[\s\S]*bGrabHandReturnEnabled\s*=\s*true[\s\S]*fGrabHandReturnMaxAngleDegrees' `
        "$configPath must publish both visual-return setting families."
}

if ($failures.Count -gt 0) {
    foreach ($failure in $failures) {
        Write-Error $failure
    }
    exit 1
}

Write-Host 'Visual return source boundaries passed.'
