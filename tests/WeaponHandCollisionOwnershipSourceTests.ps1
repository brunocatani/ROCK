param(
    [Parameter(Mandatory = $true)]
    [string]$Root
)

$ErrorActionPreference = "Stop"

$physics = Join-Path $Root "src/physics-interaction/core/PhysicsInteraction.cpp"
$physicsHeader = Join-Path $Root "src/physics-interaction/core/PhysicsInteraction.h"
$contacts = Join-Path $Root "src/physics-interaction/core/PhysicsInteractionContacts.inl"

foreach ($path in @($physics, $physicsHeader, $contacts)) {
    if (-not (Test-Path -LiteralPath $path)) {
        throw "Required collision ownership source path missing: $path"
    }
}

$physicsText = Get-Content -LiteralPath $physics -Raw
$headerText = Get-Content -LiteralPath $physicsHeader -Raw
$contactsText = Get-Content -LiteralPath $contacts -Raw

if ($headerText -notmatch "_rightDominantWeaponCollisionSuppressed" -or
    $headerText -notmatch "_rightDominantWeaponCollisionSuppression") {
    throw "Dominant weapon ownership must have explicit right-hand collision suppression state."
}

if ($physicsText -notmatch "suppressRightHandCollisionForDominantWeapon" -or
    $physicsText -notmatch "restoreRightHandCollisionAfterDominantWeapon" -or
    $physicsText -notmatch "CollisionSuppressionOwner::WeaponDominantHand") {
    throw "Dominant weapon ownership must acquire and release the shared collision suppression registry lease."
}

$resolveWeaponIndex = $physicsText.IndexOf("RE::NiNode* weaponNode = resolveEquippedWeaponInteractionNode();")
$updateHandIndex = $physicsText.IndexOf("updateHandCollisions(frame);", $resolveWeaponIndex)
$suppressRightIndex = $physicsText.IndexOf("suppressRightHandCollisionForDominantWeapon(hknp);", $resolveWeaponIndex)
if ($resolveWeaponIndex -lt 0 -or $updateHandIndex -lt 0 -or $suppressRightIndex -lt 0 -or $suppressRightIndex -gt $updateHandIndex) {
    throw "Right-hand weapon collision suppression must be resolved before generated hand colliders are updated for the frame."
}

if ($physicsText -notmatch "restoreRightHandCollisionAfterDominantWeapon\(hknpMenu\)" -or
    $physicsText -notmatch "restoreRightHandCollisionAfterDominantWeapon\(hknp\)") {
    throw "Dominant weapon collision suppression must restore through menu, scale-change, and shutdown lifecycle paths."
}

if ($contactsText -notmatch "_rightDominantWeaponCollisionSuppressed\.load" -or
    $contactsText -notmatch "bodyAIsRight \|\| bodyBIsRight" -or
    $contactsText -notmatch "_leftWeaponSupportCollisionSuppressed\.load") {
    throw "Suppressed weapon-owned hand bodies must be ignored at the contact-route boundary, including stale contact events."
}

if ($contactsText -notmatch "!contactRoute\.recordWorldSurfaceEvidence" -or
    $contactsText -notmatch "!contact_pipeline_policy::isHand\(contactRoute\.source\.kind\)" -or
    $contactsText -notmatch "if \(!ensureRawContactPoint\(\)\)") {
    throw "Native contact evidence recording must be limited to raw hand/world evidence consumed by soft contact."
}

exit 0
