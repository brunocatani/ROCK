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

Require-Text 'src/physics-interaction/hand/HandGrab.cpp' `
    'GRAB_RETURN_HAND_PRIORITY\s*=\s*85[\s\S]*beginGrabVisualReturn\(\);[\s\S]*clearGrabExternalHandWorldTransform\(_isLeft\)' `
    'Generic release must publish the lower-priority return pose before clearing active grab authority.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' `
    'GrabReleaseOutcome Hand::releaseGrabbedObject[\s\S]*const bool applyReleaseVelocity[\s\S]*setHeldVelocity\([\s\S]*beginGrabVisualReturn\(\);[\s\S]*clearGrabExternalHandWorldTransform\(_isLeft\)' `
    'Generic visual return must begin only after physical release velocity and lifecycle work are complete.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' `
    '_grabVisualReturn\.active[\s\S]*_grabVisualReturn\.lastApplied[\s\S]*_grabVisualHandLerpStartTransform\s*=\s*acquisitionStart' `
    'A new generic acquisition must start from the last pose rendered by an interrupted return.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'processHand\(_rightHand,\s*false\);[\s\S]*processHand\(_leftHand,\s*true\);[\s\S]*updateGrabVisualReturn\(frame\.right\.rawHandWorld[\s\S]*updateGrabVisualReturn\(frame\.left\.rawHandWorld' `
    'Empty-hand generic returns must advance after normal input processing for both physical hands.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.h' `
    'struct ReturningWeaponVisualState[\s\S]*VisualReturnTransition[\s\S]*weaponGenerationKey[\s\S]*equippedWeaponOwnershipKey[\s\S]*nativeBaselineLocal' `
    'Weapon return must be a generation-bound visual overlay with its own native local baseline.'
Reject-Text 'src/physics-interaction/weapon/TwoHandedGrip.h' `
    'enum class TwoHandedState[\s\S]{0,240}Returning' `
    'Visual return must not remain gameplay/manual grip ownership in TwoHandedState.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'transitionToPartCarry\(\)[\s\S]*blockFrikPrimaryWeaponPose\(\)[\s\S]*beginHandVisualReturn\(_firingHandIsLeft,\s*"primary-detach-part-carry"\)[\s\S]*_state\s*=\s*TwoHandedState::PartCarry' `
    'Primary detach must acquire the required blocker before returning only the departing firing hand while PartCarry takes authority immediately.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'releasePartGrip\(supportHandIsLeft,\s*"support-grip-released",\s*true\)[\s\S]*releasePartGrip\(firingHandIsLeft,\s*"free-hand-grip-released",\s*true\)' `
    'Player-driven PartCarry releases must start a per-hand return in both physical directions.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'beginHandVisualReturn\(_firingHandIsLeft,\s*"ambidextrous-firing-hand-promotion"\)[\s\S]*setFiringHand\(supportHandIsLeft' `
    'Ambidextrous promotion must capture the departing firing hand before role ownership changes.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'worldTargetToParentLocal\(nativeParent->world,\s*startWorld\)[\s\S]*releaseFiringHandWeaponNodeOwnership\(_activeWeaponNode\)[\s\S]*_activeWeaponNode->parent\s*!=\s*nativeParent[\s\S]*nativeBaselineLocal' `
    'Weapon return must restore native right-hand topology before converging in native parent-local space.'
Reject-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'void TwoHandedGrip::beginWeaponVisualReturn[\s\S]{0,5000}frik_visual_authority::blockPrimaryWeaponNodeOwnership' `
    'Weapon return must not engage hFRIK external-left-carry topology as a transform-write blocker.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'new-two-hand-acquisition[\s\S]*nativeBaselineLocal[\s\S]*new-primary-acquisition' `
    'New weapon acquisition must interrupt return without losing the original native baseline.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'requestEquippedWeaponDrop[\s\S]*clearWeaponVisualReturn\("equipped-weapon-drop"[\s\S]*transitionToInactive\(false\)' `
    'A real equipped-weapon drop must cancel any in-flight weapon return before gameplay teardown.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'clearAllVisualReturns\("weapon-identity-or-parent-changed"' `
    'Weapon node or parent changes must clear the complete visual-return overlay.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'clearAllVisualReturns\("equipped-weapon-identity-changed"' `
    'Weapon generation or ownership changes must clear the complete visual-return overlay.'

Require-Text 'src/physics-interaction/hand/DynamicHandCollision.cpp' `
    'physicallyOwnedByStrongerSystem[\s\S]*visuallyOwnedByStrongerSystem\s*=\s*physicallyOwnedByStrongerSystem\s*\|\|\s*visualReturnActive[\s\S]*!physicallyOwnedByStrongerSystem[\s\S]*if \(visuallyOwnedByStrongerSystem' `
    'Visual return must suppress only the competing dynamic-hand visual writer, not free-hand haptics or proxy tracking.'

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
