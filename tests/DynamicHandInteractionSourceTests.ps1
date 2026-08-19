param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
$failures = [System.Collections.Generic.List[string]]::new()

function Read-Source {
    param([string]$Path)
    $fullPath = Join-Path $Root $Path
    if (-not (Test-Path -LiteralPath $fullPath)) {
        $failures.Add("Missing source file: $Path")
        return ''
    }
    return Get-Content -Raw -LiteralPath $fullPath
}

function Require-Pattern {
    param([string]$Path, [string]$Pattern, [string]$Message)
    if ((Read-Source $Path) -notmatch $Pattern) {
        $failures.Add($Message)
    }
}

function Reject-Pattern {
    param([string]$Path, [string]$Pattern, [string]$Message)
    if ((Read-Source $Path) -match $Pattern) {
        $failures.Add($Message)
    }
}

$handHeader = 'src/physics-interaction/hand/DynamicHandCollision.h'
$handSource = 'src/physics-interaction/hand/DynamicHandCollision.cpp'
$layers = 'src/physics-interaction/collision/CollisionLayerPolicy.h'
$contacts = 'src/physics-interaction/core/PhysicsInteractionContacts.inl'
$provider = 'src/physics-interaction/core/PhysicsInteractionProvider.inl'
$api = 'src/api/ROCKProviderApi.h'

foreach ($ini in @('data/config/ROCK.ini', 'data/mod/ROCK_Config/ROCK.ini')) {
    Require-Pattern $ini `
        '(?m)^bHandDynamicInteractionsEnabled\s*=\s*true\s*$' `
        "$ini must ship the experimental dynamic interaction graph enabled."
}
Require-Pattern 'src/RockConfig.h' `
    'rockHandDynamicInteractionsEnabled\s*=\s*true' `
    'The compiled dynamic interaction graph default must remain enabled.'

Require-Pattern $layers `
    'ROCK_LAYER_DYNAMIC_RIGHT_HAND_PROXY\s*=\s*[\s\S]*ROCK_LAYER_DYNAMIC_HAND_PROXY[\s\S]*ROCK_LAYER_DYNAMIC_LEFT_HAND_PROXY\s*=\s*52' `
    'Right and left dynamic twins must retain stable rows 48 and 52.'
Require-Pattern $layers `
    'buildRockDynamicHandProxyExpectedMask\([\s\S]*isLeft[\s\S]*dynamicWeaponInteractionEnabled[\s\S]*isLeft\s*\?\s*ROCK_LAYER_DYNAMIC_RIGHT_HAND_PROXY\s*:[\s\S]*ROCK_LAYER_DYNAMIC_LEFT_HAND_PROXY[\s\S]*dynamicWeaponInteractionEnabled[\s\S]*ROCK_LAYER_DYNAMIC_WEAPON_PROXY' `
    'Each hand row must enable the opposite hand and independently gate its weapon edge.'
Require-Pattern $layers `
    'buildRockDynamicWeaponProxyExpectedMask\([\s\S]*rightHandInteractionEnabled[\s\S]*ROCK_LAYER_DYNAMIC_RIGHT_HAND_PROXY[\s\S]*leftHandInteractionEnabled[\s\S]*ROCK_LAYER_DYNAMIC_LEFT_HAND_PROXY' `
    'The weapon row must symmetrically enable only free-hand rows.'
Require-Pattern $handSource `
    'dynamicHandProxyFilterInfo\([\s\S]*bool isLeft[\s\S]*dynamicHandProxyLayerForHand\(isLeft\)' `
    'Dynamic twin creation and transition restoration must preserve the side-specific stable row.'
Require-Pattern $handSource `
    'kDynamicRightHandProxyCollisionGroup\s*=\s*0x000C[\s\S]*kDynamicLeftHandProxyCollisionGroup\s*=\s*0x000E[\s\S]*static_assert\(kDynamicRightHandProxyCollisionGroup\s*!=\s*kDynamicLeftHandProxyCollisionGroup\)[\s\S]*isLeft\s*\?[\s\S]*kDynamicLeftHandProxyCollisionGroup[\s\S]*kDynamicRightHandProxyCollisionGroup' `
    'Opposing hands must use distinct nonzero Havok groups so FO4VR does not route them through same-system rejection.'
Require-Pattern 'src/physics-interaction/native/BethesdaPhysicsBody.h' `
    'kTrackedDynamicBodyCreationOptions[\s\S]*0\.10f[\s\S]*ForcedLinearCollisionLookAhead' `
    'Tracked dynamic contact bodies must use the verified bounded continuous-collision profile.'
Require-Pattern 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' `
    'ci\s*\+\s*0x1C[\s\S]*collisionLookAheadDistanceHavok[\s\S]*ci\s*\+\s*0x50[\s\S]*bodyQuality[\s\S]*validateGeneratedBodyCollisionProfile' `
    'Generated-body creation must publish and validate FO4VR collision look-ahead and body-quality fields.'
Require-Pattern $handSource `
    'BethesdaMotionType::Dynamic,[\s\S]{0,200}kTrackedDynamicBodyCreationOptions' `
    'Dynamic hand twins must opt into the tracked continuous-collision profile.'
Require-Pattern 'src/physics-interaction/weapon/DynamicWeaponCollision.cpp' `
    'BethesdaMotionType::Dynamic,[\s\S]{0,200}kTrackedDynamicBodyCreationOptions' `
    'The dynamic weapon contact compound must opt into the tracked continuous-collision profile.'

# These offsets and identity gates were independently derived from raw FO4VR
# 1.2.72 disassembly. A source-only regression makes accidental removal fail
# the normal test configure/build path.
Require-Pattern $handSource `
    'flushPendingPhysicsDrive\([\s\S]*_weaponPairLeases\.reconcile\(' `
    'Native pair-table mutations must occur only in the serialized physics drive phase.'
Reject-Pattern $handSource `
    'updateFrame\([\s\S]{0,500}_weaponPairLeases\.reconcile\(' `
    'The main frame must never mutate the native pair table.'

Require-Pattern $contacts `
    'tryClassifyDynamicBodyContactSourceAtomic\([\s\S]*recordDynamicBodyContactCallback\([\s\S]*bodyAIsDynamicWeapon[\s\S]*bodyBIsDynamicWeapon' `
    'Both callback paths must aggregate dynamic hand-hand and hand-weapon pairs before registry filtering.'
Require-Pattern $handHeader `
    'pendingOtherHandContactMaskAtomic[\s\S]*pendingWeaponContactMaskAtomic' `
    'Dynamic pair contact publication must use bounded atomic slot masks.'

Require-Pattern $api `
    'DynamicOtherHandContact\s*=\s*1u\s*<<\s*19[\s\S]*DynamicWeaponContact\s*=\s*1u\s*<<\s*20[\s\S]*DynamicWeaponPairSuppressed\s*=\s*1u\s*<<\s*21' `
    'Hand interaction state must expose dynamic contact and suppression modes.'
Require-Pattern $api `
    'DynamicInteractionsEnabled\s*=\s*1u\s*<<\s*7[\s\S]*DynamicPairFilterReady\s*=\s*1u\s*<<\s*11' `
    'Collision availability must expose graph enablement and native pair-filter readiness.'
Require-Pattern $provider `
    'otherHandContactMask[\s\S]*weaponContactMask[\s\S]*dynamicInteractionLayer[\s\S]*suppressedWeaponPairCount' `
    'Provider collision availability must publish contact masks, stable layer, and exact suppression count.'

if ($failures.Count -gt 0) {
    foreach ($failure in $failures) {
        Write-Error $failure
    }
    exit 1
}

Write-Host 'Dynamic hand interaction source checks passed.'
