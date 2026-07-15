param([string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
$failures = [System.Collections.Generic.List[string]]::new()

function Require-Text {
    param([string]$Path, [string]$Pattern, [string]$Message)
    $fullPath = Join-Path $Root $Path
    if (-not (Test-Path -LiteralPath $fullPath) -or (Get-Content -Raw -LiteralPath $fullPath) -notmatch $Pattern) {
        $failures.Add($Message)
    }
}

function Reject-Text {
    param([string]$Path, [string]$Pattern, [string]$Message)
    $fullPath = Join-Path $Root $Path
    if ((Test-Path -LiteralPath $fullPath) -and (Get-Content -Raw -LiteralPath $fullPath) -match $Pattern) {
        $failures.Add($Message)
    }
}

function Require-OrderedText {
    param([string]$Path, [string[]]$Patterns, [string]$Message)
    $fullPath = Join-Path $Root $Path
    if (-not (Test-Path -LiteralPath $fullPath)) { $failures.Add($Message); return }
    $text = Get-Content -Raw -LiteralPath $fullPath
    $offset = 0
    foreach ($pattern in $Patterns) {
        $match = [regex]::Match($text.Substring($offset), $pattern, [System.Text.RegularExpressions.RegexOptions]::Singleline)
        if (-not $match.Success) { $failures.Add($Message); return }
        $offset += $match.Index + $match.Length
    }
}

Require-OrderedText 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' @(
    'append\(motionArray, 0x70\)',
    'for \(std::size_t index = 0; index < memberCount;',
    'reinterpret_cast<std::uint32_t\*>\(bytes \+ 0x0C\) = 0'
) 'The group must create exactly one motion cinfo and point every body cinfo at local motion zero.'

Require-Text 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' `
    'snapshot\.motionIndex != sharedMotion' `
    'Every dynamic weapon twin must be runtime-validated against the same world motion.'

Require-OrderedText 'src/physics-interaction/weapon/WeaponCollision.cpp' @(
    'weaponMemberCount\s*=\s*memberCount',
    'if \(weaponMemberCount == 0\)',
    'appendHandTwins',
    'hand->buildDynamicTwinShape\(',
    'DynamicAuthorityMemberKind::LeftHandTwin',
    '_dynamicAuthorityGroup\.create\('
) 'Held-hand palm/fingertip hulls must join the same native body group as the weapon hulls.'

Require-Text 'src/physics-interaction/weapon/WeaponCollision.h' `
    'MAX_DYNAMIC_AUTHORITY_BODIES[\s\S]{0,180}2 \* dynamic_hand_twin::kBodiesPerHand' `
    'The shared group and debug snapshot must reserve both complete six-body held-hand sets beyond the weapon bank.'

Require-OrderedText 'src/physics-interaction/core/PhysicsInteraction.cpp' @(
    'resolveHeldHandCoupling\(',
    'synchronizeWeaponCoupledHands\(',
    'updateDynamicAuthorityFrame\('
) 'Ambidextrous role resolution and independent-proxy retirement must precede shared-group intent publication.'

Require-Text 'src/physics-interaction/weapon/DynamicWeaponCollisionAuthorityPolicy.h' `
    'rightHandWeaponAuthorityActive \|\| input\.rightPartGripActive' `
    'Right firing/support/part-carry roles must all couple the physical right hand.'
Require-Text 'src/physics-interaction/weapon/DynamicWeaponCollisionAuthorityPolicy.h' `
    'leftFiringGripActive \|\| input\.leftSupportOrPartGripActive' `
    'Left firing/support/part-carry roles must all couple the physical left hand.'

Require-Text 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' `
    'niRowsToHavokColumns\(members\[index\]\.initialWorld\.rotate\)' `
    'Body-group creation must convert normal BODY/Ni stored axes to the native quaternion convention.'

Reject-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' `
    '_dynamicAuthorityGroup\.setMemberTransformDeferred\(' `
    'A live shared weapon motion must never be reshaped member-by-member during the physics solve.'

Require-OrderedText 'src/physics-interaction/weapon/WeaponCollision.cpp' @(
    '_dynamicAuthorityMemberWeaponLocal\s*=\s*memberWeaponLocals',
    'intent\.memberWeaponLocal\[groupIndex\]\s*=\s*capturedLocal',
    '_dynamicAuthorityGroup\.member\(0\)',
    'driveGeneratedKeyframedBody\('
) 'Weapon and held-hand member frames must be captured once per rigid group, then only the anchor may drive it.'

Require-OrderedText 'src/physics-interaction/weapon/WeaponCollision.cpp' @(
    'requestedWeaponInverse\s*=\s*transform_math::invertTransform\(requestedWeaponWorld\)',
    'memberWeaponLocal\s*=\s*transform_math::composeTransforms\(',
    'requestedWeaponInverse, handBodyWorld',
    '_dynamicAuthorityMemberWeaponLocal\s*=\s*memberWeaponLocals'
) 'Held-hand fixtures must capture their rigid weapon-local relation from explicit raw requested intent.'

Require-OrderedText 'src/physics-interaction/weapon/WeaponCollision.cpp' @(
    'applySampledAnchorCorrectionToCurrentIntent\(',
    'intent\.requestedWeaponWorld',
    'physics\.commandedAnchorWorld',
    'physics\.liveAnchorWorld'
) 'Weapon/hand collision feedback must retain the requested-commanded-live contract instead of feeding the rendered result back into intent.'
Require-OrderedText 'src/physics-interaction/weapon/WeaponCollision.cpp' @(
    'sampleDynamicAuthorityPostSolve\(',
    'prior\.liveAnchorWorld\s*=\s*liveAnchor'
) 'The live shared-motion pose must be published only by the post-solve phase.'

Require-OrderedText 'src/physics-interaction/weapon/WeaponCollision.cpp' @(
    '!physics\.contactActive',
    'memberArrangementChanged',
    'rebuildForCurrentRoles\(\)'
) 'Animated attached-part changes must rebuild the complete group only outside a real contact solve.'

Require-OrderedText 'src/physics-interaction/core/PhysicsInteractionContacts.inl' @(
    'GeneratedBodyKind::DynamicWeaponAuthority',
    'isWorldSurfaceLayer\(targetLayer\)',
    'recordDynamicAuthorityWorldContact\(',
    'return;'
) 'Layer-49 world callbacks must provide contact truth and exit before gameplay weapon routing.'

Require-OrderedText 'src/physics-interaction/weapon/WeaponCollision.cpp' @(
    '_dynamicAuthorityContactSignalSequence\.load',
    'sourceBelongsToCurrentGroup',
    'hasRecentWorldContactSignal\(',
    'residualIsPlausible\(',
    'const bool contact = recentWorldContact'
) 'Residual magnitude must use a current-group native world-contact signal and fail closed on divergence.'

Reject-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' `
    'std::isfinite\(rotationRadians\)\s*\?' `
    'A non-finite post-solve rotation must propagate into residual validation instead of being masked as zero.'

Require-OrderedText 'src/physics-interaction/core/PhysicsInteraction.cpp' @(
    'getCollisionRequestedWeaponTransform\(',
    'updateDynamicAuthorityFrame\(',
    'applyCollisionResolvedWeaponAuthority\(',
    'updateBodiesFromCurrentSourceTransforms\('
) 'Raw weapon intent must resolve before final keyframed weapon colliders follow the rendered weapon.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    '_weaponCollision\.sampleDynamicAuthorityPostSolve\(world\);' `
    'Dynamic weapon authority must sample the shared motion after Havok solves contacts.'

Require-Text 'src/physics-interaction/collision/CollisionLayerPolicy.h' `
    'ROCK_LAYER_DYNAMIC_WEAPON_PROXY = 49' `
    'Dynamic weapon twins must remain on dedicated layer 49.'

Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' `
    'generatedDynamicWeaponAuthorityFilterInfo\(false\)' `
    'Dynamic weapon twins must enter the world collision-disabled until their complete group is valid.'

Require-OrderedText 'src/physics-interaction/weapon/WeaponCollision.cpp' @(
    'driveGeneratedKeyframedBody\(',
    'generatedDynamicWeaponAuthorityFilterInfo\(true\)',
    '_dynamicAuthorityGroup\.validateSharedMotion\(world\)',
    '_dynamicAuthorityCollisionEnabled\.store\(true'
) 'World collision must enable only after the first anchor drive and shared-motion validation succeed.'

Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' `
    'generatedMaterialId\s*=\s*[\s\S]{0,100}registerGeneratedBodyMaterial\(world\)' `
    'Weapon and held-hand shared-motion members must use ROCK''s established generated-collider friction material.'

Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' `
    'g_rockConfig\.rockDebugDrawDynamicWeaponColliders' `
    'Dynamic weapon twins must draw behind their own bDebugDrawDynamicWeaponColliders flag.'

Require-Text 'src/RockConfig.cpp' `
    '"bDebugDrawDynamicWeaponColliders"' `
    'The dynamic weapon visualization flag must be independently configurable.'

Require-OrderedText 'src/physics-interaction/native/GeneratedKeyframedBodyDrive.cpp' @(
    'BethesdaPhysicsBodyGroup::Member& body',
    'tryGetBodyArrayWorldTransform\(world, body\.getBodyId\(\), outTransform\)',
    'BodyFrameSource::BodyTransform'
) 'Shared-motion anchor limiting must use the selected BODY frame, never the aggregate motion COM.'

Require-Text 'src/physics-interaction/debug/DebugBodyOverlay.cpp' `
    'BodyOverlayRole::Target \|\| role == BodyOverlayRole::DynamicWeapon' `
    'Every dynamic weapon twin must render from its distinct BODY transform instead of their shared COM.'

Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' `
    'visualBodyWorld\s*=\s*makeGeneratedBodyArrayWorldTransform\(sourceWorld, center\)' `
    'Shared-motion bookkeeping must derive member locals from normal BODY frames rather than the keyframed weapon drive encoding.'

Require-OrderedText 'src/physics-interaction/native/GeneratedKeyframedBodyDrive.cpp' @(
    'writeDynamicTargetRotation\(',
    'BethesdaPhysicsBodyGroup::Member&',
    'niRowsToHavokColumns\(targetRotation\)'
) 'Shared-motion hard-keyframe rotation must convert its normal BODY frame only at the native boundary.'

Require-Text 'src/physics-interaction/native/GeneratedKeyframedBodyDrive.cpp' `
    'Teleporting only the selected anchor can mutate its fixture relative' `
    'Shared-motion recovery must never teleport only the anchor member.'

Require-Text 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' `
    'belongsToWorld\(world\).*setBodyTransformDeferred' `
    'Shared-motion member mutation must reject body IDs from another hknpWorld.'

Require-Text 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' `
    'bool BethesdaPhysicsBodyGroup::validateSharedMotion' `
    'The group must expose a reusable runtime shared-motion/back-pointer invariant check.'

Require-OrderedText 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' @(
    'snapshot\.motionIndex != _sharedMotionIndex',
    'snapshot\.collisionObject != _collisionObjects\[index\]'
) 'Shared-motion validation must require one motion ID and the expected collision-object selector for every member.'

Reject-Text 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' `
    'motionPropertiesId[\s\S]{0,160}BethesdaMotionType::Dynamic' `
    'FO4VR motion-property profile IDs must not be compared with BethesdaMotionType enum values.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'rawRightHandWorld[\s\S]{0,500}_rightFiringHandCanonicalWeaponLocal' `
    'Native right-hand collision intent must be reconstructed from the raw wrist and stable authored hold frame.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'dynamicWeaponProxyPairsDrifted' `
    'The collision-layer watchdog must restore reverse-row drift for dynamic weapon world contacts.'

Require-OrderedText 'src/physics-interaction/core/PhysicsInteraction.cpp' @(
    'World stale or null',
    '_weaponCollision\.abandonHavokStateAfterWorldLoss\(\)'
) 'World-loss teardown must abandon shared-motion handles without dereferencing the stale Havok world.'

Require-OrderedText 'src/physics-interaction/weapon/TwoHandedGrip.cpp' @(
    'applyCollisionResolvedWeaponAuthority\(',
    'isManualOwnershipActive\(\)',
    'clearCollisionResolvedWeaponAuthority\(\);'
) 'Final weapon authority must clear stale native-hand ownership before composing a manual grip mode.'

if ($failures.Count -gt 0) {
    Write-Host 'Dynamic weapon collision authority source boundary failed:'
    foreach ($failure in $failures) { Write-Host " - $failure" }
    exit 1
}

Write-Host 'Dynamic weapon collision authority source boundary passed.'
