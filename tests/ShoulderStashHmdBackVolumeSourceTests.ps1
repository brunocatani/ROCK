param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$failures = [System.Collections.Generic.List[string]]::new()

function Require-Text {
    param(
        [string]$RelativePath,
        [string]$Pattern,
        [string]$Message
    )

    $path = Join-Path $Root $RelativePath
    $text = Get-Content -Raw -LiteralPath $path
    if ($text -notmatch $Pattern) {
        $failures.Add("$RelativePath`: $Message")
    }
}

function Reject-Text {
    param(
        [string]$RelativePath,
        [string]$Pattern,
        [string]$Message
    )

    $path = Join-Path $Root $RelativePath
    $text = Get-Content -Raw -LiteralPath $path
    if ($text -match $Pattern) {
        $failures.Add("$RelativePath`: $Message")
    }
}

Require-Text 'src/physics-interaction/stash/ShoulderStashDetector.h' `
    'hmdBackRightOffsetGameUnits\{\s*14\.0f,\s*-18\.0f,\s*-6\.85f\s*\}' `
    'Detector fallback should bias the right HMD back volume farther behind the shoulder.'
Require-Text 'src/physics-interaction/stash/ShoulderStashDetector.h' `
    'hmdBackLeftOffsetGameUnits\{\s*-14\.0f,\s*-18\.0f,\s*-6\.85f\s*\}' `
    'Detector fallback should bias the left HMD back volume farther behind the shoulder.'
Require-Text 'src/physics-interaction/stash/ShoulderStashDetector.h' 'hmdBackEnterPaddingGameUnits\s*=\s*0\.0f' `
    'Detector fallback should use precise HMD-specific enter padding.'
Require-Text 'src/physics-interaction/stash/ShoulderStashDetector.h' 'hmdBackExitPaddingGameUnits\s*=\s*2\.0f' `
    'Detector fallback should use narrow HMD-specific exit padding.'
Require-Text 'src/physics-interaction/stash/ShoulderStashDetector.h' 'hmdBackMinBehindGameUnits\s*=\s*4\.0f' `
    'Detector fallback should require the hand behind the HMD for HMD stash.'
Require-Text 'src/physics-interaction/stash/ShoulderStashDetector.cpp' 'hmdBackBehindGateAllows\(\s*hmdProbeLocal\.y,\s*input\.config\.hmdBackMinBehindGameUnits\s*\)' `
    'HMD stash detector must apply its behind gate in the yaw-only HMD-local frame.'
Require-Text 'src/physics-interaction/stash/ShoulderStashMath.h' 'planarForward\{\s*hmdForwardWorld\.x,\s*hmdForwardWorld\.y,\s*0\.0f\s*\}' `
    'HMD stash geometry must remove head pitch before building the back frame.'
Require-Text 'src/physics-interaction/stash/ShoulderStashDetector.cpp' 'ProbeMotionFrame::HmdBackLocal' `
    'HMD stash speed must use HMD-local hand motion so locomotion cancels.'
Require-Text 'src/physics-interaction/stash/ShoulderStashDetector.cpp' 'if \(hmdAuthorityAvailable\)[\s\S]{0,500}findHmdBackVolumeCandidate[\s\S]{0,500}else' `
    'Valid HMD authority must exclude the body-zone fallback pocket.'
Require-Text 'src/physics-interaction/stash/ShoulderStashDetector.cpp' 'input\.config\.hmdBackExitPaddingGameUnits\s*:\s*input\.config\.hmdBackEnterPaddingGameUnits' `
    'HMD stash detector should use HMD-specific padding rather than body-zone padding.'





Require-Text 'src/RockConfig.h' `
    'rockEquippedWeaponShoulderStashEnabled\s*=\s*true' `
    'RockConfig must default the standalone equipped-weapon sheath feature on.'
Require-Text 'src/RockConfig.cpp' `
    'GetBoolValue\(\s*SECTION,\s*"bEquippedWeaponShoulderStashEnabled"' `
    'RockConfig must load equipped-weapon shoulder stash from [PhysicsInteraction].'
Require-Text 'data/config/ROCK.dev.ini' `
    '(?m)^bEquippedWeaponShoulderStashEnabled\s*=\s*true\s*$' `
    'The reference INI must expose ROCK''s standalone equipped-weapon sheath switch.'
Require-Text 'src/RockConfig.h' `
    'rockShoulderStashHmdBackRightOffsetGameUnits\s*=\s*RE::NiPoint3\(14\.0f,\s*-18\.0f,\s*-6\.85f\)' `
    'RockConfig header default should match the behind-shoulder HMD right volume.'
Require-Text 'src/RockConfig.h' `
    'rockShoulderStashHmdBackLeftOffsetGameUnits\s*=\s*RE::NiPoint3\(-14\.0f,\s*-18\.0f,\s*-6\.85f\)' `
    'RockConfig header default should match the behind-shoulder HMD left volume.'
Require-Text 'src/RockConfig.cpp' `
    'rockShoulderStashHmdBackRightOffsetGameUnits\s*=\s*RE::NiPoint3\(14\.0f,\s*-18\.0f,\s*-6\.85f\)' `
    'RockConfig reset default should match the behind-shoulder HMD right volume.'
Require-Text 'src/RockConfig.cpp' `
    'rockShoulderStashHmdBackLeftOffsetGameUnits\s*=\s*RE::NiPoint3\(-14\.0f,\s*-18\.0f,\s*-6\.85f\)' `
    'RockConfig reset default should match the behind-shoulder HMD left volume.'
Require-Text 'src/RockConfig.h' 'rockShoulderStashHmdBackEnterPaddingGameUnits\s*=\s*0\.0f' `
    'RockConfig header default should expose precise HMD enter padding.'
Require-Text 'src/RockConfig.h' 'rockShoulderStashHmdBackExitPaddingGameUnits\s*=\s*2\.0f' `
    'RockConfig header default should expose precise HMD exit padding.'
Require-Text 'src/RockConfig.h' 'rockShoulderStashHmdBackMinBehindGameUnits\s*=\s*4\.0f' `
    'RockConfig header default should expose the behind-HMD gate.'
Require-Text 'src/RockConfig.cpp' 'fShoulderStashHmdBackEnterPaddingGameUnits' `
    'RockConfig loader should read HMD-specific enter padding.'
Require-Text 'src/RockConfig.cpp' 'fShoulderStashHmdBackExitPaddingGameUnits' `
    'RockConfig loader should read HMD-specific exit padding.'
Require-Text 'src/RockConfig.cpp' 'fShoulderStashHmdBackMinBehindGameUnits' `
    'RockConfig loader should read the behind-HMD gate.'

Reject-Text 'src/physics-interaction/stash/ShoulderStashDetector.h' '17\.5f,\s*-5\.0f,\s*-6\.85f' `
    'Detector fallback must not keep the old front-biased HMD back volume.'
Reject-Text 'src/physics-interaction/stash/ShoulderStashDetector.h' '14\.0f,\s*-12\.0f,\s*-6\.85f' `
    'Detector fallback must not keep the previous still-too-forward HMD back volume.'
Reject-Text 'src/RockConfig.h' '17\.5f,\s*-5\.0f,\s*-6\.85f' `
    'RockConfig header default must not keep the old front-biased HMD back volume.'
Reject-Text 'src/RockConfig.h' '14\.0f,\s*-12\.0f,\s*-6\.85f' `
    'RockConfig header default must not keep the previous still-too-forward HMD back volume.'
Reject-Text 'src/RockConfig.cpp' '17\.5f,\s*-5\.0f,\s*-6\.85f' `
    'RockConfig reset default must not keep the old front-biased HMD back volume.'
Reject-Text 'src/RockConfig.cpp' '14\.0f,\s*-12\.0f,\s*-6\.85f' `
    'RockConfig reset default must not keep the previous still-too-forward HMD back volume.'

if ($failures.Count -gt 0) {
    Write-Host 'ShoulderStashHmdBackVolumeSourceTests failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'ShoulderStashHmdBackVolumeSourceTests passed.' -ForegroundColor Green
