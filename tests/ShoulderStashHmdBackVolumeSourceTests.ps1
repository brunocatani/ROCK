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

function Read-IniFloat {
    param(
        [string]$Text,
        [string]$Key
    )

    if ($Text -notmatch "(?m)^\s*$([regex]::Escape($Key))\s*=\s*(-?\d+(?:\.\d+)?)\s*$") {
        $failures.Add("data/config/ROCK.ini: missing $Key")
        return 0.0
    }

    return [double]$Matches[1]
}

function Require-HmdBackAngle {
    param(
        [string]$Side,
        [double]$SideOffset,
        [double]$ForwardOffset
    )

    if ($ForwardOffset -ge 0.0) {
        $failures.Add("data/config/ROCK.ini: $Side HMD stash Y offset must be behind the HMD")
        return
    }

    $horizontalLength = [Math]::Sqrt(($SideOffset * $SideOffset) + ($ForwardOffset * $ForwardOffset))
    if ($horizontalLength -le 0.0) {
        $failures.Add("data/config/ROCK.ini: $Side HMD stash horizontal offset must not be zero")
        return
    }

    $angleFromForwardDegrees = [Math]::Acos($ForwardOffset / $horizontalLength) * 180.0 / [Math]::PI
    if ($angleFromForwardDegrees -lt 125.0) {
        $failures.Add("data/config/ROCK.ini: $Side HMD stash center must sit at least 35 degrees behind pure side")
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
Require-Text 'src/physics-interaction/stash/ShoulderStashDetector.cpp' 'hmdBackBehindGateAllows\(\s*hmdForwardOffset,\s*input\.config\.hmdBackMinBehindGameUnits\s*\)' `
    'HMD stash detector must reject forward-side probes before sphere scoring.'
Require-Text 'src/physics-interaction/stash/ShoulderStashDetector.cpp' 'input\.config\.hmdBackExitPaddingGameUnits\s*:\s*input\.config\.hmdBackEnterPaddingGameUnits' `
    'HMD stash detector should use HMD-specific padding rather than body-zone padding.'

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

Require-Text 'data/config/ROCK.ini' 'fShoulderStashHmdBackRightOffsetXGameUnits\s*=\s*14\.0' `
    'Packaged config should move the right HMD back volume inward from the old side-biased pocket.'
Require-Text 'data/config/ROCK.ini' 'fShoulderStashHmdBackRightOffsetYGameUnits\s*=\s*-18\.0' `
    'Packaged config should move the right HMD back volume behind the player.'
Require-Text 'data/config/ROCK.ini' 'fShoulderStashHmdBackLeftOffsetXGameUnits\s*=\s*-14\.0' `
    'Packaged config should move the left HMD back volume inward from the old side-biased pocket.'
Require-Text 'data/config/ROCK.ini' 'fShoulderStashHmdBackLeftOffsetYGameUnits\s*=\s*-18\.0' `
    'Packaged config should move the left HMD back volume behind the player.'
Require-Text 'data/config/ROCK.ini' 'fShoulderStashHmdBackEnterPaddingGameUnits\s*=\s*0\.0' `
    'Packaged config should keep HMD stash entry precise.'
Require-Text 'data/config/ROCK.ini' 'fShoulderStashHmdBackExitPaddingGameUnits\s*=\s*2\.0' `
    'Packaged config should keep HMD stash exit hysteresis narrow.'
Require-Text 'data/config/ROCK.ini' 'fShoulderStashHmdBackMinBehindGameUnits\s*=\s*4\.0' `
    'Packaged config should prevent forward-side HMD stash candidates.'

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
Reject-Text 'data/config/ROCK.ini' 'fShoulderStashHmdBackRightOffsetXGameUnits\s*=\s*17\.5|fShoulderStashHmdBackRightOffsetYGameUnits\s*=\s*-5\.0|fShoulderStashHmdBackRightOffsetYGameUnits\s*=\s*-12\.0|fShoulderStashHmdBackLeftOffsetXGameUnits\s*=\s*-17\.5|fShoulderStashHmdBackLeftOffsetYGameUnits\s*=\s*-5\.0|fShoulderStashHmdBackLeftOffsetYGameUnits\s*=\s*-12\.0' `
    'Packaged config must not keep the old side/front-biased HMD back volume.'

$iniText = Get-Content -Raw -LiteralPath (Join-Path $Root 'data/config/ROCK.ini')
$rightX = Read-IniFloat $iniText 'fShoulderStashHmdBackRightOffsetXGameUnits'
$rightY = Read-IniFloat $iniText 'fShoulderStashHmdBackRightOffsetYGameUnits'
$leftX = Read-IniFloat $iniText 'fShoulderStashHmdBackLeftOffsetXGameUnits'
$leftY = Read-IniFloat $iniText 'fShoulderStashHmdBackLeftOffsetYGameUnits'
$hmdEnterPadding = Read-IniFloat $iniText 'fShoulderStashHmdBackEnterPaddingGameUnits'
$hmdExitPadding = Read-IniFloat $iniText 'fShoulderStashHmdBackExitPaddingGameUnits'
$hmdMinBehind = Read-IniFloat $iniText 'fShoulderStashHmdBackMinBehindGameUnits'

if ($rightX -le 0.0) {
    $failures.Add('data/config/ROCK.ini: right HMD stash X offset must stay on the player right side')
}
if ($leftX -ge 0.0) {
    $failures.Add('data/config/ROCK.ini: left HMD stash X offset must stay on the player left side')
}

Require-HmdBackAngle 'right' $rightX $rightY
Require-HmdBackAngle 'left' $leftX $leftY

if ($hmdEnterPadding -ne 0.0) {
    $failures.Add('data/config/ROCK.ini: HMD stash enter padding should be zero for precise entry')
}
if ($hmdExitPadding -gt 2.0) {
    $failures.Add('data/config/ROCK.ini: HMD stash exit padding should stay narrow')
}
if ($hmdMinBehind -lt 4.0) {
    $failures.Add('data/config/ROCK.ini: HMD stash min-behind gate should reject forward-side probes')
}

if ($failures.Count -gt 0) {
    Write-Host 'ShoulderStashHmdBackVolumeSourceTests failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'ShoulderStashHmdBackVolumeSourceTests passed.' -ForegroundColor Green
