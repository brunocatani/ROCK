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

function Require-Order {
    param([string]$Path, [string[]]$Patterns, [string]$Message)
    $text = Read-Source $Path
    $offset = 0
    foreach ($pattern in $Patterns) {
        $match = [regex]::Match(
            $text.Substring($offset),
            $pattern,
            [System.Text.RegularExpressions.RegexOptions]::Singleline)
        if (-not $match.Success) {
            $failures.Add($Message)
            return
        }
        $offset += $match.Index + $match.Length
    }
}

$runtimeHeader = 'src/physics-interaction/weapon/DynamicWeaponCollision.h'
$runtimeSource = 'src/physics-interaction/weapon/DynamicWeaponCollision.cpp'
$runtimePolicy = 'src/physics-interaction/weapon/DynamicWeaponCollisionPolicy.h'
$weaponAuthority = 'src/physics-interaction/weapon/TwoHandedGrip.cpp'
$interaction = 'src/physics-interaction/core/PhysicsInteraction.cpp'
$contacts = 'src/physics-interaction/core/PhysicsInteractionContacts.inl'
$layers = 'src/physics-interaction/collision/CollisionLayerPolicy.h'

# The experiment must remain opt-in and use a new world-only matrix row. The
# existing layer-44 weapon hulls continue to own gameplay contact evidence.
Require-Pattern 'src/RockConfig.h' `
    'rockWeaponCollisionDynamicBoxEnabled\s*=\s*false' `
    'The dynamic weapon box must default disabled in compiled configuration.'
foreach ($ini in @('data/config/ROCK.ini', 'data/mod/ROCK_Config/ROCK.ini')) {
    Require-Pattern $ini `
        '(?m)^bWeaponCollisionDynamicBoxEnabled\s*=\s*false\s*$' `
        "$ini must ship the experimental dynamic box disabled."
}
Require-Pattern $layers `
    'ROCK_LAYER_DYNAMIC_WEAPON_PROXY\s*=\s*51' `
    'The dynamic weapon proxy must retain its dedicated layer-51 row.'
Require-Pattern $layers `
    'buildRockDynamicWeaponProxyExpectedMask\(\)[\s\S]*isWorldSurfaceLayer\(layer\)[\s\S]*return mask' `
    'The dynamic weapon proxy row must be authored exclusively from world-surface layers.'
Require-Pattern $layers `
    'applyRockGeneratedLayerPolicies[\s\S]*applyRockDynamicWeaponProxyLayerPolicy\(matrix\)' `
    'Layer 51 must be registered with the other generated collision rows.'

# Stage one is exactly one approximate convex box, not one dynamic operation
# per member of the shared layer-44 multi-hull system that failed previously.
Require-Pattern $runtimeHeader `
    'BethesdaPhysicsBody\s+_body' `
    'The dynamic weapon runtime must own one explicit body.'
Reject-Pattern $runtimeHeader `
    'std::array\s*<\s*BethesdaPhysicsBody|std::vector\s*<\s*BethesdaPhysicsBody' `
    'The stage-one runtime must not expand into a dynamic body bank.'
Require-Pattern $runtimeSource `
    'makeBoxCornerPointsHavok[\s\S]*std::vector<RE::NiPoint3> pointCloud\(corners\.begin\(\), corners\.end\(\)\)[\s\S]*buildConvexShapeFromLocalHavokPoints' `
    'The single box must be built from the bounded eight-corner point cloud.'
Require-Pattern $runtimeSource `
    'BethesdaMotionType::Dynamic[\s\S]*ROCK_DynamicWeaponBox' `
    'The stage-one proxy must be a real dynamic Bethesda body.'

# One-way publication is the core anti-feedback invariant: all native/ROCK
# weapon writers publish collision-free intent, then one bypass publication
# applies the solved pose without observing itself.
Require-Pattern 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'notifyVisualIntentObserver\s*&&\s*_weaponVisualIntentObserver[\s\S]*_weaponVisualIntentObserver\([\s\S]*applyWeaponCollisionResolvedAuthority[\s\S]*applyWeaponVisualAuthority\([\s\S]*false\s*\)' `
    'Weapon visual authority must separate collision-free intent observation from the solved bypass publication.'
Require-Order $interaction @(
    '_twoHandedGrip\.beginWeaponCollisionPresentationFrame\(',
    '_dynamicWeaponCollision\.beginFrame\(',
    '_twoHandedGrip\.update\(',
    '_twoHandedGrip\.applyGunstockAlignment\(',
    '_dynamicWeaponCollision\.finishFrame\(',
    'applyWeaponCollisionResolvedAuthority\(',
    '_weaponCollision\.updateBodiesFromCurrentSourceTransforms\('
) 'Previous-frame collision hand authority must clear before dynamic weapon intent starts, with post-solve resolution remaining before layer-44 hull transforms.'
Require-Pattern $runtimePolicy `
    'selectAttachedHands\([\s\S]*partCarry[\s\S]*firingGripOccupied[\s\S]*leftPartGripActive[\s\S]*rightPartGripActive[\s\S]*firingHandIsActuallyLeft' `
    'Collision hand coupling must select the native/manual firing hand and active part-grip hands without moving a free hand.'
Require-Pattern $runtimePolicy `
    'reframeAttachedHand\([\s\S]*invertTransform\(requestedWeaponWorld\)[\s\S]*requestedHandWorld[\s\S]*resolvedWeaponWorld[\s\S]*handWeaponLocal' `
    'Attached IK hands must preserve their exact pre-collision weapon-local relation under the resolved weapon pose.'
Require-Order $weaponAuthority @(
    'void TwoHandedGrip::beginWeaponCollisionPresentationFrame\(',
    'clearWeaponCollisionHandAuthority\(true\)',
    'clearWeaponCollisionHandAuthority\(false\)',
    'applyWeaponCollisionResolvedAuthority\(',
    'selectAttachedHands\(',
    'tryGetRootFlattenedHandBoneTransform\(',
    'reframeAttachedHand\(',
    'applyExternalHandWorldTransform\(',
    '_weaponCollisionHandAuthorityLive',
    'applyWeaponVisualAuthority\('
) 'Collision correction must clear the prior render frame before intent, retain attached hands through the current render interval, and publish the exact weapon pose last.'
Require-Pattern $weaponAuthority `
    'Retain the high-priority result through rendering[\s\S]*next PhysicsInteraction frame clears this tag before any[\s\S]*controller/grip intent is sampled' `
    'The source must document the render-lifetime and next-intent anti-feedback invariant proven by the two-hand authority trace.'
Reject-Pattern $weaponAuthority `
    'bool TwoHandedGrip::applyWeaponCollisionResolvedAuthority\([\s\S]*?clearExternalHandWorldTransform\([\s\S]*?bool TwoHandedGrip::applyFiringHandLockedVisual' `
    'Post-solve collision publication must not clear its hand tag in the same method; that synchronously restores lower-priority two-hand targets before rendering.'
Require-Pattern $weaponAuthority `
    'WEAPON_COLLISION_HAND_PRIORITY\s*=\s*110[\s\S]*GRIP_HAND_POSE_PRIORITY\s*=\s*100|GRIP_HAND_POSE_PRIORITY\s*=\s*100[\s\S]*WEAPON_COLLISION_HAND_PRIORITY\s*=\s*110' `
    'The collision hand authority must outrank normal grip targets only through the final presentation interval.'
Require-Pattern $weaponAuthority `
    'void TwoHandedGrip::reset\(\)[\s\S]{0,600}WEAPON_COLLISION_HAND_TAG[\s\S]{0,500}Hand::Left[\s\S]{0,500}WEAPON_COLLISION_HAND_TAG[\s\S]{0,500}Hand::Right[\s\S]{0,300}_weaponCollisionHandAuthorityLive\s*=\s*\{\}' `
    'Lifecycle reset must defensively clear collision hand authority and its per-hand live state.'

# Visual correction is admitted only by a genuine proxy/world callback. The
# legacy key-3 point path remains intact while the proxy-specific key-2 path
# treats a validated processed manifold as contact evidence. Merely seeing a
# solver residual is not sufficient evidence that a wall caused it.
Require-Order $contacts @(
    'isProxyBodyIdAtomic\(bodyIdA\)',
    'tryReadFilterInfo\(',
    'isWorldSurfaceLayer\(otherLayer\)',
    'ensureRawContactPoint\(\)',
    'recordWorldSurfaceContactCallback\(',
    'shouldSkipContactSignalBeforeLayerRead\('
) 'A raw proxy/world contact point must be captured before the normal gameplay-contact prefilter discards layer 51.'
Require-Order $contacts @(
    'kManifoldProcessedEventType\s*=\s*static_cast<RE::hknpEventType::Enum>\(2\)',
    's_manifoldProcessedEventBridge',
    'handleManifoldProcessedEvent\(world,\s*contactEventData\)'
) 'The verified key-2 signal must own a distinct retained bridge and callback route.'
Require-Order $contacts @(
    'handleManifoldProcessedEvent\(',
    'recordSize\s*!=\s*kExpectedRecordSize\s*\|\|\s*eventKey\s*!=\s*kManifoldProcessedEventKey',
    'isProxyBodyIdAtomic\(bodyIdA\)',
    'tryReadFilterInfo\(',
    'recordWorldSurfaceManifoldProcessedCallback\('
) 'Processed-manifold admission must validate the native record and exact proxy/world pair before publication.'
Require-Pattern $runtimeSource `
    '\(snapshot\.contactActive\s*\|\|\s*_surfaceCouplingActive\)\s*&&[\s\S]*!snapshot\.teleported' `
    'Visual correction must require active contact or bounded release recovery and reject teleport samples.'
Require-Pattern $runtimeSource `
    'advanceSurfaceCoupledTarget\([\s\S]*_previousRawProxyBodyTarget[\s\S]*rawRequestedBodyTarget[\s\S]*snapshot\.requestedProxyBodyWorld[\s\S]*snapshot\.liveProxyBodyWorld' `
    'Established contact must discard accumulated absolute pose debt while preserving current controller deltas.'
Require-Pattern $runtimePolicy `
    'advanceSurfaceCoupledTarget\([\s\S]*makeBoundedContactAnchor\([\s\S]*invertTransform\(previousRawProxyBodyWorld\)[\s\S]*currentRawProxyBodyWorld' `
    'Surface coupling must advance from the solver anchor by only the newest collision-free intent delta.'
Require-Pattern $runtimeSource `
    'rawIntentMoving[\s\S]*advanceSurfaceCoupledTarget\([\s\S]*snapshot\.liveProxyBodyWorld,[\s\S]*snapshot\.liveProxyBodyWorld,[\s\S]*0\.0f,[\s\S]*0\.0f[\s\S]*_surfaceCouplingIdleSeconds\s*=\s*0\.0f' `
    'Released surface coupling must preserve controller deltas until motion pauses instead of immediately driving back into the obstacle.'
Require-Pattern $runtimeSource `
    'correctionWithinSafetyEnvelope[\s\S]*rockWeaponCollisionDynamicMaxVisualCorrectionGameUnits[\s\S]*kMaxVisualCorrectionRotationDegrees' `
    'Physics-resolved visual correction must retain a finite fail-closed safety envelope.'

# The proxy participates in the same callback-clock drive and deterministic
# live/stale-world cleanup contract as the existing generated bodies.
Require-Order $interaction @(
    '_weaponCollision\.flushPendingPhysicsDrive\(world, timing\);',
    '_dynamicWeaponCollision\.flushPendingPhysicsDrive\(world, timing\);',
    '_dynamicHandCollision\.flushPendingPhysicsDrive\(world, timing\);'
) 'The dynamic weapon body must drive inside the generated pre-solve callback.'
Require-Order $interaction @(
    '_completedPhysicsSolveSequence\.fetch_add\(',
    '_dynamicWeaponCollision\.samplePostSolve\(',
    '_dynamicHandCollision\.samplePostSolveDeviations\('
) 'The dynamic weapon correction snapshot must be sampled on the post-solve callback clock.'
Require-Pattern $runtimeSource `
    'retireProxyLocked[\s\S]*_body\.retireDeferred\(' `
    'Live-world dynamic weapon teardown must use deferred body retirement.'
Require-Pattern $runtimeSource `
    'liveOwnerMatches[\s\S]*bhkWorld\s*==\s*_createdBhkWorld[\s\S]*_body\.retireDeferred\(bhkWorld\)' `
    'Native retirement must require proof that the supplied world still owns the proxy.'
Reject-Pattern $runtimeSource `
    'bhkWorld\s*\?\s*bhkWorld\s*:\s*_createdBhkWorld' `
    'Teardown must not fall back to a cached world after live-world ownership is uncertain.'
Require-Pattern $runtimeSource `
    'abandonHavokStateAfterWorldLoss[\s\S]*_body\.reset\(\)' `
    'Stale-world cleanup must abandon wrapper state without native removal.'
Reject-Pattern $runtimeSource `
    '_body\.destroy\(' `
    'The dynamic weapon body must never use immediate live-world destruction.'

# FO4VR 0x1417A3A90 is initializeAsKeyFramed. It zeros motion-cinfo inverse
# mass, so it must never contaminate the shared velocity-driven body wrapper.
Reject-Pattern 'src/physics-interaction/native/HavokOffsets.h' `
    'kFunc_MotionCinfo_DeriveFromBodyCinfos' `
    'The keyframed initializer must not remain mislabeled as dynamic mass derivation.'
Reject-Pattern 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' `
    'deriveMotionCinfo\s*\(' `
    'Dynamic weapon bodies must retain the native motion-cinfo constructor inverse mass.'
Require-Pattern 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' `
    '0x1417A3A90 is initializeAsKeyFramed[\s\S]{0,500}motionCinfoCtor\(motionCinfo\);' `
    'The generated-body wrapper must document and preserve the dynamic-safe constructor profile.'
Require-Pattern 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' `
    'kGeneratedBodyRuntimeFlags\s*=\s*0x0802''0000[\s\S]{0,250}kRebuildBodyCollisionState\s*=\s*0[\s\S]*setFilterInfo\(world,\s*bodyId,\s*filterInfo,\s*1\);[\s\S]{0,1000}enableFlags\(world,\s*bodyId\.value,\s*kGeneratedBodyRuntimeFlags,\s*kRebuildBodyCollisionState\);' `
    'Generated weapon bodies must publish filter and modifier eligibility with one native collision-state rebuild.'
Require-Pattern $runtimeSource `
    'kRaiseManifoldProcessedEvents\s*=\s*0x40u[\s\S]*enableBodyFlags\([\s\S]{0,200}kRaiseManifoldProcessedEvents,[\s\S]{0,100}kRebuildBodyCollisionState\)[\s\S]*flaggedBody\.body->flags\s*&\s*kRaiseManifoldProcessedEvents' `
    'Only the dynamic weapon proxy must opt into key-2 processed-manifold events through the verified mode-0 body flag path.'
Reject-Pattern 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' `
    'kGeneratedBodyRuntimeFlags\s*=\s*0x0*4020000' `
    'Processed-manifold event opt-in must not leak into the shared generated-body defaults.'

# The dormant overlay flag is now the primary in-game shape/contact diagnostic.
# A surviving runtime mismatch must distinguish callback admission, snapshot
# admission, and immediate visual-authority readback without hot-path log spam.
Require-Pattern $runtimeHeader `
    '_proxyPairCallbackSequenceAtomic[\s\S]*_worldSurfaceCallbackSequenceAtomic[\s\S]*_rawPointCallbackSequenceAtomic[\s\S]*_processedManifoldCallbackSequenceAtomic[\s\S]*_contactSequenceAtomic' `
    'Dynamic weapon diagnostics must retain separate callback-stage counters.'
Require-Pattern $runtimeSource `
    'rockDebugDrawDynamicWeaponColliders[\s\S]*ROCK_LOG_SAMPLE_INFO\([\s\S]*DWC pipeline:[\s\S]*snapshot\(read/valid/identity/contact/teleport\)' `
    'Dynamic weapon pipeline diagnostics must be debug-gated and rate-limited.'
Require-Pattern $interaction `
    'applyWeaponCollisionResolvedAuthority[\s\S]*immediateTranslationError[\s\S]*immediateRotationError[\s\S]*DWC visual publication' `
    'Dynamic weapon visual publication must expose immediate node readback evidence.'
Require-Pattern 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' `
    'rockDebugDrawDynamicWeaponColliders[\s\S]*proxyBodyIdForDebug\(\)[\s\S]*DWC BOX[\s\S]*callbacks pair/world/raw/manifold/admit[\s\S]*snapshot read/valid/id/contact/tele' `
    'The dedicated debug flag must draw the proxy body and its contact/correction telemetry.'

if ($failures.Count -gt 0) {
    Write-Host 'Dynamic weapon collision source boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Dynamic weapon collision source boundary passed.'
