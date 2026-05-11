<#
Shoulder stash is intentionally a held-object release disposition backed by generated
body-zone collider evidence. This boundary test keeps the implementation from drifting
back to HMD-only proximity, ordinary throw release, or ad hoc inventory filtering.
#>

param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..'))
)

$ErrorActionPreference = 'Stop'

$failures = New-Object System.Collections.Generic.List[string]

function Read-RepoFile {
    param([string]$RelativePath)
    $path = Join-Path $Root $RelativePath
    if (-not (Test-Path -LiteralPath $path)) {
        $script:failures.Add("Missing required file: $RelativePath")
        return ''
    }
    return Get-Content -LiteralPath $path -Raw
}

function Require-Text {
    param(
        [string]$RelativePath,
        [string]$Pattern,
        [string]$Message
    )

    $content = Read-RepoFile $RelativePath
    if ($content -notmatch $Pattern) {
        $script:failures.Add($Message)
    }
}

function Require-ProdIniKey {
    param(
        [string]$Key,
        [string]$Value
    )

    $prodPath = 'C:\Users\SENECA\Documents\My Games\Fallout4VR\ROCK_Config\ROCK.ini'
    if (-not (Test-Path -LiteralPath $prodPath)) {
        $script:failures.Add('Active prod ROCK.ini is missing; shoulder stash keys cannot be verified.')
        return
    }

    $content = Get-Content -LiteralPath $prodPath -Raw
    $escapedKey = [regex]::Escape($Key)
    $escapedValue = [regex]::Escape($Value)
    if ($content -notmatch "(?m)^\s*$escapedKey\s*=\s*$escapedValue\s*$") {
        $script:failures.Add("Active prod ROCK.ini must expose $Key = $Value.")
    }
}

Require-Text 'src/physics-interaction/stash/ShoulderStashDetector.h' 'body-zone interaction first' `
    'Shoulder stash detector must document body-zone collider authority.'
Require-Text 'src/physics-interaction/stash/ShoulderStashDetector.cpp' 'EvidenceSource::BodyZoneColliderAndContact' `
    'Shoulder stash detector must combine geometric body-zone evidence with recent contact confirmation.'
Require-Text 'src/physics-interaction/stash/ShoulderStashDetector.cpp' 'captureSustainedContactAnchor' `
    'Shoulder stash detector must preserve held-body contact anchors for sustained back contact.'
Require-Text 'src/physics-interaction/stash/ShoulderStashDetector.cpp' 'EvidenceSource::BodyZoneSustainedContact' `
    'Shoulder stash detector must emit a distinct sustained-contact evidence source after fresh contact expires.'
Require-Text 'src/physics-interaction/stash/ShoulderStashDetector.cpp' 'sustainedHeldBodyLocalPointGame' `
    'Shoulder stash sustained contact must be anchored in held-body local space, not only a transient world point.'
Require-Text 'src/physics-interaction/stash/ShoulderStashDetector.cpp' 'findHmdFallbackCandidate' `
    'Shoulder stash must keep HMD fallback isolated behind the detector fallback path.'
Require-Text 'src/physics-interaction/stash/ShoulderStashDetector.cpp' 'bodyAuthorityAvailable' `
    'Shoulder stash HMD fallback must be gated by body-collider authority, not by candidate miss alone.'
Require-Text 'tests/ShoulderStashPolicyTests.cpp' 'hmd fallback blocked when body authority is available' `
    'Compiled policy tests must cover HMD fallback rejection when body colliders are authoritative.'
Require-Text 'src/physics-interaction/stash/ShoulderStashEligibility.cpp' 'RE::ENUM_FORM_ID::kNPC_' `
    'Shoulder stash eligibility must reject actor base forms.'
Require-Text 'src/physics-interaction/stash/ShoulderStashEligibility.cpp' 'GetPlayable\(baseForm->GetBaseInstanceData\(\)\)' `
    'Shoulder stash eligibility must use the same playable-form rule as ordinary takeable objects.'
Require-Text 'src/physics-interaction/stash/ShoulderStashEligibility.cpp' 'kFo4BookCantBeTakenFlag\s*=\s*1u\s*<<\s*1' `
    'Shoulder stash eligibility must preserve the verified FO4VR untakeable-book flag.'
Require-Text 'src/physics-interaction/stash/ShoulderStashTransfer.cpp' 'kExtraCountCountOffset\s*=\s*0x18' `
    'Shoulder stash transfer must preserve the verified FO4VR ExtraCount count offset.'
Require-Text 'src/physics-interaction/stash/ShoulderStashTransfer.cpp' 'ActivateRef\(player,\s*nullptr,\s*result\.count' `
    'Shoulder stash transfer must use TESObjectREFR::ActivateRef as the default pickup path.'
Require-Text 'src/physics-interaction/body/BodyBoneColliderSet.h' 'float\s+lengthGameUnits\s*=\s*0\.0f;[\s\S]*float\s+radiusGameUnits\s*=\s*0\.0f;' `
    'Body collider metadata must publish generated capsule dimensions for stash precision.'
Require-Text 'src/physics-interaction/hand/Hand.h' 'enum class GrabReleaseDisposition' `
    'Hand release context must carry explicit release disposition.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'releaseContext\.disposition\s*==\s*GrabReleaseDisposition::PhysicalDrop[\s\S]*RELEASE VELOCITY' `
    'Stash release must not inherit ordinary throw velocity.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'GrabReleaseDisposition::PendingInventoryTransfer[\s\S]*RELEASE VELOCITY' `
    'Pending stash transfer releases must capture release velocity for transfer-failure fallback.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'shoulder_stash::evaluateEligibility' `
    'PhysicsInteraction must run stash eligibility before release.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'GrabReleaseDisposition::PendingInventoryTransfer' `
    'PhysicsInteraction must release stash commits as pending transfers until native pickup succeeds.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'shoulder_stash::transferToPlayerInventory' `
    'PhysicsInteraction must transfer confirmed stash releases to player inventory.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'applyReleaseVelocitySnapshot' `
    'PhysicsInteraction must restore physical drop velocity when native stash transfer fails.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'dispatchShoulderStashEvent\(GrabEventType::Stashed,\s*nullptr' `
    'Post-transfer stashed events must not publish a potentially stale object pointer.'
Require-Text 'src/RockConfig.cpp' '"bShoulderStashEnabled"' `
    'RockConfig must read bShoulderStashEnabled.'
Require-Text 'src/RockConfig.cpp' '"iShoulderStashSustainedContactMissFrames"' `
    'RockConfig must read iShoulderStashSustainedContactMissFrames.'
Require-Text 'data/config/ROCK.ini' '(?m)^\s*bShoulderStashEnabled\s*=\s*true\s*$' `
    'Packaged ROCK.ini must enable shoulder stash by default.'
Require-Text 'data/config/ROCK.ini' '(?m)^\s*bShoulderStashUseBodyZoneColliders\s*=\s*true\s*$' `
    'Packaged ROCK.ini must keep body-zone colliders as primary stash evidence.'
Require-Text 'data/config/ROCK.ini' '(?m)^\s*bShoulderStashAllowHmdFallback\s*=\s*true\s*$' `
    'Packaged ROCK.ini must expose the HMD fallback toggle.'
Require-Text 'data/config/ROCK.ini' '(?m)^\s*iShoulderStashSustainedContactMissFrames\s*=\s*18\s*$' `
    'Packaged ROCK.ini must expose sustained shoulder stash contact miss tolerance.'
Require-Text 'data/config/ROCK.ini' '(?m)^\s*bShoulderStashSkipActivateBooks\s*=\s*true\s*$' `
    'Packaged ROCK.ini must bypass activation for stash-picked books by default.'
Require-Text 'data/config/ROCK.ini' '(?m)^\s*bShoulderStashSkipActivateNotes\s*=\s*true\s*$' `
    'Packaged ROCK.ini must bypass activation for stash-picked notes by default.'
Require-ProdIniKey 'bShoulderStashEnabled' 'true'
Require-ProdIniKey 'bShoulderStashUseBodyZoneColliders' 'true'
Require-ProdIniKey 'bShoulderStashAllowHmdFallback' 'true'
Require-ProdIniKey 'iShoulderStashSustainedContactMissFrames' '18'
Require-ProdIniKey 'bShoulderStashSkipActivateBooks' 'true'
Require-ProdIniKey 'bShoulderStashSkipActivateNotes' 'true'

if ($failures.Count -gt 0) {
    $failures | ForEach-Object { Write-Error $_ }
    exit 1
}

Write-Host 'ROCK shoulder stash source boundary passed.'
