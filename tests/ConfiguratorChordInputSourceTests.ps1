param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$failures = [System.Collections.Generic.List[string]]::new()

function Require-Text {
    param([string]$RelativePath, [string]$Pattern, [string]$Message)

    $text = Get-Content -Raw -LiteralPath (Join-Path $Root $RelativePath)
    if ($text -notmatch $Pattern) {
        $failures.Add("$RelativePath`: $Message")
    }
}

# Physical B and Y share OpenVR's ApplicationMenu ID and are distinguished by
# their controller. The reservation must remain a ROCK-owned input policy, not
# a provider API change or a Configurator-specific alternate raw-input path.
Require-Text 'src/physics-interaction/input/InputRemapPolicy.h' `
    'kOpenVrApplicationMenuButtonId\s*=\s*1[\s\S]{0,240}kOpenVrConfiguratorChordButtonId\s*=\s*kOpenVrApplicationMenuButtonId[\s\S]{0,240}kOpenVrGrenadeQuickDrawButtonId\s*=\s*kOpenVrApplicationMenuButtonId' `
    'B, Y, and grenade quick draw must use the verified shared ApplicationMenu button ID.'
Require-Text 'src/physics-interaction/input/InputRemapPolicy.h' `
    'evaluateConfiguratorChordReservation[\s\S]{0,900}input\.eligible\s*&&\s*input\.leftHeld\s*&&\s*input\.rightHeld[\s\S]{0,300}kConfiguratorChordBothHandsMask[\s\S]{0,800}sampledHandJustPressed[\s\S]{0,500}~sampledHandMask' `
    'Chord ownership must latch both hands and rearm each hand only on a later standalone press.'

# The OpenVR hook captures physical state before any mutation. Game-facing
# callers lose only the shared chord button, while the Configurator bypass and
# provider raw-button API continue to observe both real physical levels.
Require-Text 'src/physics-interaction/input/InputRemapRuntime.cpp' `
    'captureControllerState[\s\S]{0,4200}updateConfiguratorChordReservation[\s\S]{0,3500}applyGameFacingControllerState' `
    'Raw controller capture must establish chord ownership before game-facing state is filtered.'
Require-Text 'src/physics-interaction/input/InputRemapRuntime.cpp' `
    'applyGameFacingControllerState[\s\S]{0,1200}configuratorChordReserved[\s\S]{0,700}shouldBypassOpenVrGameInputMutation[\s\S]{0,900}clearConfiguratorChordButtonForGame' `
    'Game-facing filtering must honor the Configurator raw-input bypass and clear only the chord button.'
Require-Text 'src/physics-interaction/input/InputRemapRuntime.cpp' `
    'shouldBypassOpenVrGameInputMutation[\s\S]{0,700}ROCKConfigurator\.dll' `
    'The Configurator module must retain physical OpenVR input while the chord is reserved.'

# Every existing owner of B or Y must observe the same latched reservation so
# no press, hold, or release phase escapes through another behavior path.
Require-Text 'src/physics-interaction/input/InputRemapRuntime.cpp' `
    'hookedNativeVatsVansDecision[\s\S]{0,900}isConfiguratorChordHandReserved\([\s\S]{0,120}Hand::Right[\s\S]{0,2500}\.suppressAll\s*=\s*suppressAll' `
    'The complete right-B VATS/V.A.N.S. transaction must be suppressed during the chord.'
Require-Text 'src/physics-interaction/input/InputRemapRuntime.cpp' `
    'hookedMenuOpenEventHandler[\s\S]{0,1000}isConfiguratorChordHandReserved\([\s\S]{0,120}Hand::Left[\s\S]{0,700}\.suppressGesture\s*=\s*configuratorChordReserved' `
    'The complete left-Y Pip-Boy/Pause transaction must be suppressed during the chord.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'serviceLooseGrenadeQuickDraw[\s\S]{0,500}consumeRawButtonState[\s\S]{0,500}isConfiguratorChordInputReserved\(false\)[\s\S]{0,900}resolveEquippedGrenadeSelection' `
    'Grenade quick draw must drain but reject the reserved right-B press edge.'

Require-Text 'tests/InputRemapPolicyTests.cpp' `
    'simultaneous B\+Y begins Configurator chord ownership[\s\S]{0,1800}releasing B keeps both chord release phases blocked[\s\S]{0,2200}fresh standalone B press rearms only B[\s\S]{0,1600}fresh standalone Y press rearms Y' `
    'Policy coverage must lock overlap, release latching, and independent rearming.'
Require-Text 'tests/InputRemapPolicyTests.cpp' `
    'Configurator chord takes complete ownership of a pending Y gesture[\s\S]{0,1000}fresh Y press rearms after a masked chord release' `
    'Pip-Boy/Pause coverage must prevent chord releases from replaying either Y action.'

if ($failures.Count -gt 0) {
    Write-Host 'Configurator B+Y chord input boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Configurator B+Y chord input boundary passed.'
