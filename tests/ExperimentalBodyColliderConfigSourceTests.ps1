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

Require-Text 'src/RockConfig.h' `
    'rockBodyBoneCollidersEnabled\s*=\s*true[\s\S]*rockBodyBoneLegAndFootCollidersEnabled\s*=\s*false' `
    'Full-body colliders must remain enabled by default while leg and foot colliders default off.'

Require-Text 'src/RockConfig.cpp' `
    'rockBodyBoneCollidersEnabled\s*=\s*true;[\s\S]*rockBodyBoneLegAndFootCollidersEnabled\s*=\s*false;' `
    'Config reset defaults must keep leg and foot colliders disabled.'
Require-Text 'src/RockConfig.cpp' `
    'constexpr auto EXPERIMENTAL_SECTION\s*=\s*"Experimental"' `
    'Body collider experiment switches must have a dedicated INI section.'
Require-Text 'src/RockConfig.cpp' `
    'GetBoolValue\(EXPERIMENTAL_SECTION,\s*"bBodyBoneCollidersEnabled"[\s\S]*GetBoolValue\(EXPERIMENTAL_SECTION,\s*"bBodyBoneLegAndFootCollidersEnabled"' `
    'Both body collider experiment switches must load exclusively from [Experimental].'
Reject-Text 'src/RockConfig.cpp' `
    'GetBoolValue\(SECTION,\s*"bBodyBoneCollidersEnabled"|GetBoolValue\(SECTION,\s*"bBodyBoneLegAndFootCollidersEnabled"' `
    'Body collider experiment switches must not retain a hidden [PhysicsInteraction] compatibility path.'

Require-Text 'src/physics-interaction/body/BodyBoneColliderSet.cpp' `
    'role\s*==\s*BoneColliderRole::LegSegment\s*\|\|\s*role\s*==\s*BoneColliderRole::FootSegment[\s\S]*rockBodyBoneLegAndFootCollidersEnabled' `
    'The granular switch must own both leg and foot roles.'
Require-Text 'src/physics-interaction/body/BodyBoneColliderSet.cpp' `
    'if\s*\(!bodyColliderRoleEnabled\(descriptor\.role\)\)\s*\{\s*continue;' `
    'Disabled leg and foot descriptors must be omitted before shape or body creation.'
Require-Text 'src/physics-interaction/body/BodyBoneColliderSet.cpp' `
    'bodyColliderTuningSignature[\s\S]*rockBodyBoneLegAndFootCollidersEnabled' `
    'A live leg/foot config change must invalidate the body collider set for rebuild.'

foreach ($configPath in @('data/config/ROCK.ini', 'data/mod/ROCK_Config/ROCK.ini')) {
    $text = Get-Content -Raw -LiteralPath (Join-Path $Root $configPath)
    $experimentalMatch = [regex]::Match($text, '(?ms)^\[Experimental\]\s*(?<body>.*?)(?=^\[[^\]]+\])')
    if (!$experimentalMatch.Success) {
        $failures.Add("$configPath`: Missing [Experimental] section.")
        continue
    }

    $experimentalBody = $experimentalMatch.Groups['body'].Value
    if ($experimentalBody -notmatch '(?m)^bBodyBoneCollidersEnabled\s*=\s*true\s*$') {
        $failures.Add("$configPath`: Full-body switch must be present under [Experimental].")
    }
    if ($experimentalBody -notmatch '(?m)^bBodyBoneLegAndFootCollidersEnabled\s*=\s*false\s*$') {
        $failures.Add("$configPath`: Leg and foot switch must be present under [Experimental] and default off.")
    }

    $physicsMatch = [regex]::Match($text, '(?ms)^\[PhysicsInteraction\]\s*(?<body>.*?)(?=^\[[^\]]+\]|\z)')
    if (!$physicsMatch.Success) {
        $failures.Add("$configPath`: Missing [PhysicsInteraction] section.")
    } elseif ($physicsMatch.Groups['body'].Value -match '(?m)^bBodyBone(?:LegAndFoot)?CollidersEnabled\s*=') {
        $failures.Add("$configPath`: Experimental body collider switches must not remain under [PhysicsInteraction].")
    }
}

if ($failures.Count -gt 0) {
    Write-Host 'ExperimentalBodyColliderConfigSourceTests failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure" -ForegroundColor Red
    }
    exit 1
}

Write-Host 'ExperimentalBodyColliderConfigSourceTests passed.' -ForegroundColor Green
