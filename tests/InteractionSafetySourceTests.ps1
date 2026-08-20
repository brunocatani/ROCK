param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$failures = [System.Collections.Generic.List[string]]::new()

function Read-Source {
    param([string]$RelativePath)

    $path = Join-Path $Root $RelativePath
    if (-not (Test-Path -LiteralPath $path)) {
        $failures.Add("$RelativePath`: missing source file")
        return ''
    }
    return Get-Content -Raw -LiteralPath $path
}

function Require-Text {
    param(
        [string]$Text,
        [string]$Pattern,
        [string]$Message
    )

    if ($Text -notmatch $Pattern) {
        $failures.Add($Message)
    }
}

function Reject-Text {
    param(
        [string]$Text,
        [string]$Pattern,
        [string]$Message
    )

    if ($Text -match $Pattern) {
        $failures.Add($Message)
    }
}

function Get-BoundedText {
    param(
        [string]$Text,
        [string]$StartToken,
        [string]$EndToken,
        [string]$BoundaryName
    )

    $startIndex = $Text.IndexOf($StartToken, [System.StringComparison]::Ordinal)
    if ($startIndex -lt 0) {
        $failures.Add("$BoundaryName`: start boundary '$StartToken' was not found")
        return ''
    }

    $endIndex = $Text.IndexOf($EndToken, $startIndex + $StartToken.Length, [System.StringComparison]::Ordinal)
    if ($endIndex -lt 0) {
        $failures.Add("$BoundaryName`: end boundary '$EndToken' was not found")
        return ''
    }

    return $Text.Substring($startIndex, $endIndex - $startIndex)
}

function Require-OrderedTokens {
    param(
        [string]$Text,
        [string[]]$Tokens,
        [string]$Message
    )

    $searchFrom = 0
    foreach ($token in $Tokens) {
        $index = $Text.IndexOf($token, $searchFrom, [System.StringComparison]::Ordinal)
        if ($index -lt 0) {
            $failures.Add("$Message (missing or out-of-order token '$token')")
            return
        }
        $searchFrom = $index + $token.Length
    }
}

$grenadeSource = Read-Source 'src/physics-interaction/grenade/LooseGrenadeRuntime.cpp'
$physicsHeader = Read-Source 'src/physics-interaction/core/PhysicsInteraction.h'
$pendingCommitHeader = Read-Source 'src/physics-interaction/core/PendingForceGrabCommit.h'
$grabPhasePolicy = Read-Source 'src/physics-interaction/grab/GrabThreePhase.h'
$forceGrabPolicy = Read-Source 'src/physics-interaction/core/ForceGrabPolicy.h'
$bareFistPolicy = Read-Source 'src/physics-interaction/weapon/BareFistGuardPolicy.h'
$commandPolicy = Read-Source 'src/physics-interaction/api/InteractionCommandPolicy.h'
$commandQueueHeader = Read-Source 'src/physics-interaction/api/InteractionCommandQueue.h'
$providerHeader = Read-Source 'src/api/ROCKProviderApi.h'
$fo4vrRuntime = Read-Source 'src/rock_support/Fo4VrRuntime.cpp'
$fo4vrRuntimeHeader = Read-Source 'src/rock_support/Fo4VrRuntime.h'
$actorStatePolicy = Read-Source 'src/rock_support/Fo4VrActorStatePolicy.h'

$allRuntimeCpp = (
    Get-ChildItem -LiteralPath (Join-Path $Root 'src') -Recurse -File -Filter '*.cpp' |
        Sort-Object FullName |
        ForEach-Object { Get-Content -Raw -LiteralPath $_.FullName }
) -join "`n"

# Pip-Boy grenade selection must remain native. Quick draw resolves the exact
# equipped stack on one B-button edge, with no hook-owned queue or stale cache.
Reject-Text $allRuntimeCpp `
    'installEquipHook|hookedEquipObject|PendingEquipRequest|s_pendingEquipRequest' `
    'The retired grenade equip interception and pending-request state must not return.'

# FO4VR exposes the selected grenade or mine through equippedItems[0] when no
# hand-held weapon is equipped. Filter that native selection once, before any
# presentation, collision, equip-transfer, or hand-occupancy consumer sees it.
$equippedWeaponItemBoundary = Get-BoundedText $fo4vrRuntime 'RE::EquippedItem* getEquippedWeaponItem()' 'RE::EquippedWeaponData* getEquippedWeaponData()' 'equipped hand-weapon item boundary'
Require-OrderedTokens $equippedWeaponItemBoundary @(
    'middleHigh->equippedItems[0]',
    'object->formType != RE::ENUM_FORM_ID::kWEAP',
    'weapon->weaponData.type.get()',
    'weaponType == RE::WEAPON_TYPE::kGrenade',
    'weaponType == RE::WEAPON_TYPE::kMine',
    'return nullptr;',
    'return equippedItem;'
) 'The equipped hand-weapon boundary must fail closed for native grenade and mine selection records.'
Require-Text $fo4vrRuntime `
    'getEquippedWeaponData\(\)[\s\S]*?getEquippedWeaponItem\(\)' `
    'EquippedWeaponData access must inherit the throwable exclusion boundary.'
Require-Text $fo4vrRuntimeHeader `
    'RE::EquippedItem\*\s+getEquippedWeaponItem\(\)\s+noexcept' `
    'The runtime API must expose the explicit equipped hand-weapon boundary.'
Reject-Text ($allRuntimeCpp + "`n" + $fo4vrRuntimeHeader) `
    '\bgetEquippedItem\s*\(' `
    'Runtime consumers must not bypass throwable filtering through the retired raw equipped-item helper.'

$equippedSelection = Get-BoundedText $grenadeSource 'EquippedGrenadeSelectionStatus resolveEquippedGrenadeSelection(' 'const char* selectionStatusName(' 'equipped grenade selection'
Require-OrderedTokens $equippedSelection @(
    'BSAutoReadLock inventoryLock',
    'isGrenadeWeapon(weapon)',
    'stack->GetCount() == 0 || !stack->IsEquipped()',
    'equippedGrenadeStackCount != 1',
    'resolveGrenadeRuntimeDataForSources(',
    's_nextRequestId.fetch_add'
) 'Quick draw must resolve exactly one native-equipped grenade stack and its runtime data only on demand.'

$dropRequest = Get-BoundedText $grenadeSource 'DropResult dropEquippedGrenadeSelectionToWorld(' 'bool createExplosionAtReference(' 'loose grenade inventory drop'
Require-OrderedTokens $dropRequest @(
    'findExactInventoryStack(',
    '!stack.exactInstanceData',
    '!stack.equipped',
    'RemoveItemData removeData(selection.weapon, 1);',
    'result.handle = player->RemoveItem(removeData);'
) 'Inventory removal must revalidate the exact still-equipped stack before dropping one selected grenade.'
Require-OrderedTokens $dropRequest @(
    'result.handle = player->RemoveItem(removeData);',
    'if (!result.handle)',
    'const auto droppedRef = result.handle.get();',
    'if (!result.droppedRef)',
    'result.success = true;',
    'result.reason = "dropped-reference-pending";'
) 'A valid RemoveItem handle with a not-yet-resolved reference must remain an asynchronous pending transaction.'

# One fresh physical right-B edge owns quick draw. Hand selection is decided for
# both hands before inventory removal; right is preferred with left fallback.
Require-Text $forceGrabPolicy `
    'if\s*\(rightAvailable\)[\s\S]*?HandChoice::Right[\s\S]*?if\s*\(leftAvailable\)[\s\S]*?HandChoice::Left[\s\S]*?GrenadeSelectionFailure::HandsBlocked' `
    'Grenade hand policy must prefer right, fall back to left, then report both hands blocked.'
Require-Text $forceGrabPolicy `
    'partGripActiveForHand\s*\|\|[\s\S]*?equippedWeaponPresent\s*&&\s*!partCarryActive\s*&&\s*handIsLeft\s*==\s*firingHandIsLeft' `
    'Equipped-weapon hand occupancy must be role-driven for future left-hand weapon support.'

# A pending force-grab owns its hand and target through settle/commit. Organic
# selection, input, and the peer hand must not steal that authority.


# Every retry reacquires the exact handle target and commits it in the same
# update. Success is published only after the held-ref/body postcondition.
Reject-Text $pendingCommitHeader `
    'ReadyToCommit' `
    'Pending force-grab state must not reintroduce a stale ReadyToCommit phase.'
Require-Text $grabPhasePolicy `
    'if\s*\(!result\.frontHemisphere\)[\s\S]*?if\s*\(input\.programmaticArrival\)[\s\S]*?AcquisitionPhase::NearConverging[\s\S]*?"programmaticArrivalBehindPalm"' `
    'Programmatic exact-target arrival must bypass only the organic behind-palm gate while retaining convergence.'

# API force-grab single-flight is per hand and survives dequeue until a terminal
# result. Provider loss/unregistration must cancel liveness and release leases.
Require-Text $commandPolicy `
    'class ForceGrabReservations[\s\S]*?std::array<ForceGrabReservation,\s*2>\s+_slots' `
    'Provider force-grab reservations must have independent right/left slots.'
Require-Text $commandPolicy `
    'hand\s*==\s*RockProviderHand::Right\s*\?\s*0u\s*:\s*hand\s*==\s*RockProviderHand::Left\s*\?\s*1u' `
    'Provider force-grab reservation indexing must map right and left independently.'
Require-Text $commandQueueHeader `
    'isInteractionCommandActiveV1\(std::uint64_t ownerToken,\s*std::uint64_t commandId\)' `
    'Runtime force-grab service must have an explicit provider-command liveness query.'

# Grenades are globally single-flight even though ordinary API force-grabs can
# proceed independently per hand.

# Bare fists are identified from runtime evidence, not broad hand-to-hand
# weapon type, so real unarmed weapons remain supported.
Require-Text $actorStatePolicy `
    '0x140F78D10[\s\S]*?0x140E77090[\s\S]*?0x140E77100[\s\S]*?0x140FF2B90[\s\S]*?kWeaponStateStorageOffset\s*=\s*0x0C[\s\S]*?kWeaponStateShift\s*=\s*2[\s\S]*?kWeaponStateValueMask\s*=\s*0x7[\s\S]*?kGunStateShift\s*=\s*15[\s\S]*?kGunStateValueMask\s*=\s*0xF[\s\S]*?decodeWeaponState[\s\S]*?decodeGunState' `
    'The FO4VR 1.2.72 actor-state policy must retain independent binary witnesses and both verified state layouts.'
Require-Text $fo4vrRuntime `
    'getNativeWeaponState\(const RE::Actor\* actor\)[\s\S]*?static_cast<const RE::ActorState\*>\(actor\)[\s\S]*?std::memcpy\([\s\S]*?kWeaponStateStorageOffset[\s\S]*?decodeWeaponState[\s\S]*?getNativeGunState\(const RE::Actor\* actor\)[\s\S]*?decodeGunState' `
    'ROCK must decode native weapon and gun state from verified ActorState storage rather than shifted CommonLib bitfields.'
Reject-Text $allRuntimeCpp `
    '(?:->|\.)(?:weaponState|gunState)\b|GetWeaponMagicDrawn\s*\(' `
    'ROCK runtime code must not read CommonLibF4VR weaponState, gunState, or GetWeaponMagicDrawn because that bitfield block is shifted on FO4VR 1.2.72.'

Require-Text $bareFistPolicy `
    'shouldHolster[\s\S]*?rockEnabled\s*&&\s*witness\.weaponDrawn\s*&&\s*witness\.actorUsingMelee\s*&&\s*!witness\.realMeleeWeaponEquipped' `
    'Bare-fist policy must require drawn melee state with no real melee weapon equipped.'
Require-Text $physicsHeader `
    'void enforceNoBareFistState\(bool forceRecheck\)' `
    'PhysicsInteraction must own a central bare-fist state guard.'

# VirtualHolsters compatibility was removed completely. Keep the retired ABI
# probe, config surface, and input-ownership branches from returning.
foreach ($relativePath in @(
    'src/RockConfig.cpp',
    'src/RockConfig.h',
    'src/physics-interaction/input/InputRemapPolicy.h',
    'src/physics-interaction/input/InputRemapRuntime.cpp',
    'src/physics-interaction/input/InputRemapRuntime.h',
    'src/physics-interaction/weapon/WeaponSupport.h',
    'tests/InputRemapPolicyTests.cpp',
    'tests/WeaponInteractionPolicyTests.cpp',
    'data/config/ROCK.ini',
    'data/mod/ROCK_Config/ROCK.ini'
)) {
    Reject-Text (Read-Source $relativePath) `
        'VirtualHolsters|virtualHolsters|VHAPI_GetApi' `
        "VirtualHolsters compatibility must remain fully removed: $relativePath"
}

# This is an internal behavioral correction: the public ABI remains v1.
Require-Text $providerHeader `
    'ROCK_PROVIDER_API_VERSION\s*=\s*1' `
    'Interaction safety fixes must keep the public provider API at v1.'
Reject-Text $providerHeader `
    'ROCK_PROVIDER_API_VERSION\s*=\s*[2-9]' `
    'Interaction safety fixes must not bump the public provider API version.'

if ($failures.Count -gt 0) {
    Write-Host 'Interaction safety source guard failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Interaction safety source guard passed.' -ForegroundColor Green
