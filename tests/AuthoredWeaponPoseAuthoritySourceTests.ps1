param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
$failures = [System.Collections.Generic.List[string]]::new()

function Read-Source {
    param([string]$Path)
    Get-Content -Raw -LiteralPath (Join-Path $Root $Path)
}

function Require-Pattern {
    param([string]$Path, [string]$Text, [string]$Pattern, [string]$Message)
    if ($Text -notmatch $Pattern) {
        $failures.Add("$Path`: $Message")
    }
}

function Reject-Pattern {
    param([string]$Path, [string]$Text, [string]$Pattern, [string]$Message)
    if ($Text -match $Pattern) {
        $failures.Add("$Path`: $Message")
    }
}

$interactionPath = 'src/physics-interaction/core/PhysicsInteraction.cpp'
$authoredHeaderPath = 'src/physics-interaction/weapon/AuthoredPrimaryFiringGrip.h'
$authoredPath = 'src/physics-interaction/weapon/AuthoredPrimaryFiringGrip.cpp'
$gripHeaderPath = 'src/physics-interaction/weapon/TwoHandedGrip.h'

$interaction = Read-Source $interactionPath
$authoredHeader = Read-Source $authoredHeaderPath
$authored = Read-Source $authoredPath
$gripHeader = Read-Source $gripHeaderPath

Require-Pattern $authoredHeaderPath $authoredHeader `
    'AuthoredPrimaryFiringGripFrameInput[\s\S]*controllerHandWorld[\s\S]*controllerHandWorldValid' `
    'Authored weapon alignment must receive an explicit controller-reconstructed hand frame.'

Require-Pattern $interactionPath $interaction `
    'updateAuthoredPrimaryFiringGrip\(\)[\s\S]*controllerWand\s*=\s*rockFiringHandIsLeft\s*\?[\s\S]*getLeftHandNode\(\)[\s\S]*getRightHandNode\(\)[\s\S]*tryReconstructCalibratedHand\([\s\S]*controllerWand->world,[\s\S]*controllerHandWorld\)[\s\S]*\.controllerHandWorld\s*=\s*controllerHandWorld[\s\S]*\.controllerHandWorldValid\s*=\s*controllerHandWorldValid' `
    'The authored primary solve must reconstruct its physical firing hand from the raw controller wand.'

Require-Pattern $authoredPath $authored `
    'trackedHandWorld\s*=\s*input\.controllerHandWorld[\s\S]*trackedHandWorldValid\s*=\s*input\.controllerHandWorldValid[\s\S]*!trackedHandWorldValid[\s\S]*composeTransforms\(trackedHandWorld,[\s\S]*invertTransform\(authoredPrimaryHandInWeapon\)' `
    'The authored weapon transform must invert the captured hand-in-weapon relation onto the validated physical hand.'

Reject-Pattern $authoredPath $authored `
    'getHandWorldTransform\(' `
    'Authored aim must not read hFRIK presented hand bones back as controller intent.'

$preFrikMatch = [regex]::Match(
    $interaction,
    'void\s+PhysicsInteraction::refreshExternalHandWorldTransformsBeforeFrik[\s\S]*?(?=\r?\n\s*void\s+PhysicsInteraction::sampleHandTransformParity)')
if (-not $preFrikMatch.Success) {
    $failures.Add("$interactionPath`: Could not isolate the pre-FRIK hand refresh implementation.")
} else {
    Require-Pattern $interactionPath $preFrikMatch.Value `
        'leftWeaponHandDriver\s*=\s*[\r\n\s]*\{\s*leftRawHandValid,\s*leftRawHandWorld\s*\}[\s\S]*rightWeaponHandDriver\s*=\s*[\r\n\s]*\{\s*rightRawHandValid,\s*rightRawHandWorld\s*\}[\s\S]*refreshRetainedHandVisualAuthoritiesBeforeFrik[\s\S]*refreshWeaponCollisionHandAuthorityBeforeFrik' `
        'Every deferred weapon-hand owner must be refreshed from controller-reconstructed physical hands.'
    Reject-Pattern $interactionPath $preFrikMatch.Value `
        'primaryWeaponOffsetNOde|SecondaryMeleeWeaponOffsetNode2|primaryWeaponKickbackRecoilNode' `
        'Weapon animation and recoil nodes must not transport retained hand targets.'
}

Require-Pattern $gripHeaderPath $gripHeader `
    'EquippedWeaponGripFrameInput[\s\S]*leftHandDriverFrame[\s\S]*rightHandDriverFrame[\s\S]*leftScopeHandDriverFrame[\s\S]*rightScopeHandDriverFrame' `
    'Normal physical hand input and scope-only hFRIK drivers must remain separate fields.'

Require-Pattern $interactionPath $interaction `
    'leftHandDriverFrame\s*\{[\s\S]*frame\.left\.rawHandWorld[\s\S]*rightHandDriverFrame\s*\{[\s\S]*frame\.right\.rawHandWorld[\s\S]*leftScopeHandDriverFrame\s*=\s*captureScopeHandDriverFrame\(scopeHandDriverNode\(true\)\)[\s\S]*rightScopeHandDriverFrame\s*=\s*captureScopeHandDriverFrame\(scopeHandDriverNode\(false\)\)' `
    'The coherent grip frame must pair physical hand inputs with separately named scope drivers.'

Require-Pattern $authoredPath $authored `
    'shouldRetainAuthoredFiringFingerPoseWhileWeaponTransformYields\([\s\S]*eligibility\)[\s\S]*if \(!input\.rockFiringHandIsLeft && !retainFingerPose\)[\s\S]*clearAuthoredPrimaryFiringGripFingerPose\(\)[\s\S]*weapon-transform-authority-yield' `
    'Two-hand transform authority must not clear the independent authored right-hand finger lease.'

if ($failures.Count -gt 0) {
    Write-Host 'Authored weapon-pose authority source boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Authored weapon-pose authority source boundary passed.'
