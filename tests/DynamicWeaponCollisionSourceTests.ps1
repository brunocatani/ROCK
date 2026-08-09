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
$weaponCollisionHeader = 'src/physics-interaction/weapon/WeaponCollision.h'
$weaponCollisionSource = 'src/physics-interaction/weapon/WeaponCollision.cpp'
$compoundBuilder = 'src/physics-interaction/native/HavokCompoundShapeBuilder.cpp'
$weaponAuthority = 'src/physics-interaction/weapon/TwoHandedGrip.cpp'
$interaction = 'src/physics-interaction/core/PhysicsInteraction.cpp'
$contacts = 'src/physics-interaction/core/PhysicsInteractionContacts.inl'
$layers = 'src/physics-interaction/collision/CollisionLayerPolicy.h'

# The experiment must remain opt-in and use a new world-only matrix row. The
# existing layer-44 weapon hulls continue to own gameplay contact evidence.
Require-Pattern 'src/RockConfig.h' `
    'rockWeaponCollisionDynamicBoxEnabled\s*=\s*false' `
    'Dynamic weapon collision must default disabled in compiled configuration.'
foreach ($ini in @('data/config/ROCK.ini', 'data/mod/ROCK_Config/ROCK.ini')) {
    Require-Pattern $ini `
        '(?m)^bWeaponCollisionDynamicBoxEnabled\s*=\s*false\s*$' `
        "$ini must ship experimental dynamic weapon collision disabled."
    Require-Pattern $ini `
        '(?m)^fWeaponCollisionDynamicInverseInertiaMultiplier\s*=\s*1\.2\s*$' `
        "$ini must ship the qualified dynamic-weapon rotational compliance multiplier."
}
Require-Pattern 'src/RockConfig.h' `
    'rockWeaponCollisionDynamicInverseInertiaMultiplier\s*=\s*1\.2f' `
    'The compiled dynamic-weapon rotational compliance default must match the qualified runtime value.'
Require-Pattern 'src/RockConfig.cpp' `
    'fWeaponCollisionDynamicInverseInertiaMultiplier[\s\S]*kDefaultWeaponCollisionDynamicInverseInertiaMultiplier[\s\S]*0\.25f[\s\S]*4\.0f' `
    'The dynamic-weapon rotational compliance setting must load through a finite positive range.'
Require-Pattern $layers `
    'ROCK_LAYER_DYNAMIC_WEAPON_PROXY\s*=\s*51' `
    'The dynamic weapon proxy must retain its dedicated layer-51 row.'
Require-Pattern $layers `
    'buildRockDynamicWeaponProxyExpectedMask\(\)[\s\S]*isWorldSurfaceLayer\(layer\)[\s\S]*return mask' `
    'The dynamic weapon proxy row must be authored exclusively from world-surface layers.'
Require-Pattern $layers `
    'applyRockGeneratedLayerPolicies[\s\S]*applyRockDynamicWeaponProxyLayerPolicy\(matrix\)' `
    'Layer 51 must be registered with the other generated collision rows.'

# World contact remains exactly one dynamic body whose shape is one fixed-child
# compound assembled from the shared layer-44 hull geometry. A second tiny body
# is permitted only as the noncolliding keyframed constraint authority.
Require-Pattern $runtimeHeader `
    'BethesdaPhysicsBody\s+_body' `
    'The dynamic weapon runtime must own one explicit world-contact body.'
Require-Pattern $runtimeHeader `
    'BethesdaPhysicsBody\s+_authorityProxy[\s\S]*ActiveConstraint\s+_authorityConstraint' `
    'The dynamic weapon runtime must own one noncolliding authority body and one finite constraint.'
Reject-Pattern $runtimeHeader `
    'std::array\s*<\s*BethesdaPhysicsBody|std::vector\s*<\s*BethesdaPhysicsBody' `
    'The one-body compound runtime must not expand into a body bank.'
Require-Pattern $runtimeSource `
    'getCompoundGeometrySnapshot\(compoundGeometry\)[\s\S]*makeCompoundChildFrame\([\s\S]*makeCompoundChildPointHavok\([\s\S]*buildConvexShapeFromLocalHavokPoints\([\s\S]*buildStaticCompoundShape\(compoundChildren\)' `
    'The dynamic body must be built as one fixed compound from generation-bound weapon-local convex children.'
Reject-Pattern $runtimeSource `
    'makeBoxCornerPointsHavok|ROCK_DynamicWeaponBox' `
    'The superseded eight-corner box shape path must not survive beside the compound geometry provider.'
Require-Pattern $runtimeSource `
    'buildStaticCompoundShape\(compoundChildren\)[\s\S]*BethesdaMotionType::Dynamic[\s\S]*ROCK_DynamicWeaponCompound' `
    'The fixed compound must remain the shape of one real dynamic Bethesda body.'
Require-Pattern $weaponCollisionHeader `
    'CompoundGeometrySnapshotFailure[\s\S]*CompoundGeometryChildSnapshot[\s\S]*pointsWeaponLocal[\s\S]*CompoundGeometrySnapshot[\s\S]*getCompoundGeometrySnapshot' `
    'WeaponCollision must expose an owned, generation-bound weapon-local geometry snapshot.'
Require-Pattern $weaponCollisionSource `
    'getCompoundGeometrySnapshot\([\s\S]*getCurrentWeaponGenerationKey\(\)[\s\S]*getWeaponBodyCount\(\)[\s\S]*generatedLocalPointsGame[\s\S]*pointCloudCanBuildHull[\s\S]*sourceBodyCount\s*!=\s*expectedBodyCount[\s\S]*GenerationChanged[\s\S]*outSnapshot\.valid\s*=\s*true' `
    'The compound snapshot must reject incomplete, degenerate, or generation-changing layer-44 geometry.'
Require-Pattern $runtimeSource `
    'class OwnedShapeBatch[\s\S]*~OwnedShapeBatch\(\)[\s\S]*havok_ref_count::release\(shape\)[\s\S]*childShapeReferences\.take\(childShape\)' `
    'Temporary child convex references must have deterministic RAII cleanup on every build exit.'
Require-Pattern $compoundBuilder `
    'setShape\([\s\S]*constructStaticCompound\([\s\S]*releaseTemporaryInstanceShapeReferences\(instances\)' `
    'The native compound builder must retain child instances and balance temporary references.'
Require-Pattern $runtimeSource `
    'buildProxyShape\(\)[\s\S]*noContactFilterInfo\(\)[\s\S]*BethesdaMotionType::Keyframed[\s\S]*ROCK_WeaponGripAuthorityProxy[\s\S]*hasNoContactFilterInfo' `
    'The grip authority must be a verified noncolliding keyframed proxy.'
Require-Pattern $runtimeSource `
    'makeGripAuthorityTarget\(requestedWeaponWorld\)[\s\S]*makeContactBodyInGripAuthoritySpace\([\s\S]*geometry\.centerWeaponLocal,[\s\S]*scale\)[\s\S]*createGrabConstraint\([\s\S]*_authorityProxy\.getBodyId\(\)[\s\S]*_body\.getBodyId\(\)[\s\S]*initialAuthorityTarget[\s\S]*requestedWeaponWorld\.translate[\s\S]*desiredBodyTransformAuthoritySpace' `
    'The finite constraint must place its hidden authority origin at the firing grip while preserving the offset contact-body frame.'
Reject-Pattern $runtimeSource `
    'objectInGeneratedProxyLocalSpace\(initialAuthorityTarget,\s*initialContactTarget\)|identityRelation' `
    'The grip authority relation must not reintroduce mixed-frame rotation or the old collider-center identity relation.'
Require-Pattern $runtimePolicy `
    'makeContactBodyInGripAuthoritySpace\([\s\S]*makeIdentityTransform<RE::NiTransform>\(\)[\s\S]*centerWeaponLocal\.x\s*\*\s*scale[\s\S]*centerWeaponLocal\.y\s*\*\s*scale[\s\S]*centerWeaponLocal\.z\s*\*\s*scale' `
    'The generated-body relation must preserve identity rotation and contain only the scaled grip-to-center translation.'
Require-Pattern $runtimeSource `
    'getEquippedWeaponClassification\(\)[\s\S]*sanitizeWeaponMass\(weaponIdentity\.weightGame\)[\s\S]*_body\.setMass\(bodyMass\)' `
    'The generated compound contact body must use sanitized equipped-weapon mass rather than the wrapper default.'
Require-Pattern $runtimePolicy `
    'makeBoundingBoxMassProperties\([\s\S]*massOverThree[\s\S]*inertiaX[\s\S]*inertiaY[\s\S]*inertiaZ[\s\S]*result\.inverseInertia[\s\S]*result\.inverseMass' `
    'The compound body must retain the qualified bounding-envelope principal inertia and inverse mass.'
Require-Pattern $runtimeSource `
    '_body\.setMass\(bodyMass\)[\s\S]*applyWeaponEnvelopeMassProperties\([\s\S]*frame\.hknpWorld[\s\S]*_body\.getBodyId\(\)[\s\S]*geometry[\s\S]*scale[\s\S]*inertiaEnvelopePadding[\s\S]*bodyMass' `
    'The generated compound must replace the wrapper default tensor after assigning authored weapon mass.'
Require-Pattern $runtimeSource `
    'applyWeaponEnvelopeMassProperties\([\s\S]*makeBoundingBoxMassProperties\([\s\S]*normalizeInverseInertiaAxesForGrab\([\s\S]*snapshotBody\(world,\s*bodyId\)[\s\S]*MOTION_PACKED_INERTIA_OFFSET[\s\S]*rebuildMotionMassProperties\(world,\s*initialMotion\.motionIndex\)[\s\S]*snapshotBody\(world,\s*bodyId\)[\s\S]*rebuiltPacked\[0\]\s*=\s*desiredPackedInertia\[0\][\s\S]*rebuiltPacked\[3\]\s*=\s*desiredPackedMass[\s\S]*Dynamic weapon compound envelope mass properties:' `
    'The runtime must rebuild and then reapply the bounding-envelope tensor and authored mass through the verified hknp motion path.'
Require-Pattern $runtimeSource `
    'rockWeaponCollisionDynamicInverseInertiaMultiplier[\s\S]*envelopeMassProperties\.inverseInertia\.x\s*\*\s*inverseInertiaMultiplier[\s\S]*envelopeMassProperties\.inverseInertia\.y\s*\*\s*inverseInertiaMultiplier[\s\S]*envelopeMassProperties\.inverseInertia\.z\s*\*\s*inverseInertiaMultiplier[\s\S]*multiplier=\{:\.3f\}' `
    'Dynamic weapon tuning must scale only the qualified envelope inverse inertia and expose the applied multiplier.'

# One-way publication is the core anti-feedback invariant: all native/ROCK
# weapon writers publish collision-free intent, then one bypass publication
# applies the solved pose without observing itself.
Require-Pattern 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'notifyVisualIntentObserver\s*&&\s*_weaponVisualIntentObserver[\s\S]*_weaponVisualIntentObserver\([\s\S]*applyWeaponCollisionResolvedAuthority[\s\S]*applyWeaponVisualAuthority\([\s\S]*false\s*\)' `
    'Weapon visual authority must separate collision-free intent observation from the solved bypass publication.'
Require-Order $interaction @(
    '_twoHandedGrip\.beginWeaponCollisionPresentationFrame\(',
    'previousWeaponCollisionPresentationWasLive\(',
    '_dynamicWeaponCollision\.beginFrame\(',
    '_twoHandedGrip\.update\(',
    '_twoHandedGrip\.applyGunstockAlignment\(',
    '_dynamicWeaponCollision\.finishFrame\(',
    'applyWeaponCollisionResolvedAuthority\(',
    '_weaponCollision\.updateBodiesFromCurrentSourceTransforms\('
) 'Previous-frame collision hand presentation must be witnessed and cleared before dynamic intent capture, with post-solve resolution remaining before layer-44 hull transforms.'
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
    'Retain the high-priority result through rendering[\s\S]*retained witness[\s\S]*unaffected hand driver[\s\S]*FRIK''s current root was produced[\s\S]*cannot be sampled' `
    'The source must document why post-FRIK tag clearing requires driver-reconstructed input isolation.'
Reject-Pattern $weaponAuthority `
    'bool TwoHandedGrip::applyWeaponCollisionResolvedAuthority\([\s\S]*?clearExternalHandWorldTransform\([\s\S]*?bool TwoHandedGrip::applyFiringHandLockedVisual' `
    'Post-solve collision publication must not clear its hand tag in the same method; that synchronously restores lower-priority two-hand targets before rendering.'
Require-Pattern $weaponAuthority `
    'WEAPON_COLLISION_HAND_PRIORITY\s*=\s*110[\s\S]*GRIP_HAND_POSE_PRIORITY\s*=\s*100|GRIP_HAND_POSE_PRIORITY\s*=\s*100[\s\S]*WEAPON_COLLISION_HAND_PRIORITY\s*=\s*110' `
    'The collision hand authority must outrank normal grip targets only through the final presentation interval.'
Require-Pattern $weaponAuthority `
    'void TwoHandedGrip::reset\(\)[\s\S]{0,600}WEAPON_COLLISION_HAND_TAG[\s\S]{0,500}Hand::Left[\s\S]{0,500}WEAPON_COLLISION_HAND_TAG[\s\S]{0,500}Hand::Right[\s\S]{0,300}_weaponCollisionHandAuthorityLive\s*=\s*\{\}[\s\S]{0,200}_weaponCollisionHandPresentationFromPreviousFrame\s*=\s*\{\}' `
    'Lifecycle reset must defensively clear collision hand authority and its previous-presentation witness.'
Require-Pattern $weaponAuthority `
    '_weaponCollisionHandPresentationFromPreviousFrame\s*=\s*[\r\n\s]*_weaponCollisionHandAuthorityLive' `
    'The presentation boundary must capture the retained collision-hand witness before clearing its tags.'
Require-Pattern $weaponAuthority `
    'resolveCollisionIsolatedMode\([\s\S]*_weaponCollisionHandPresentationFromPreviousFrame\[handIndex\][\s\S]*_scopeDriverFrameAuthorityActive' `
    'A retained collision hand must force physical-driver reconstruction instead of feeding the collision-corrected root back into the solver.'
Require-Pattern $interaction `
    'suppressDefaultNativeWeaponIntent\s*=\s*[\r\n\s]*_twoHandedGrip\.previousWeaponCollisionPresentationWasLive\(\)[\s\S]*_dynamicWeaponCollision\.beginFrame\([\s\S]*suppressDefaultNativeWeaponIntent' `
    'A collision-contaminated native weapon pose must not remain as the fallback frame intent when isolated driver reconstruction fails.'
Require-Pattern $runtimeSource `
    '_frameHasIntent\s*=[\s\S]*_frameAcceptingIntent\s*&&[\s\S]*!suppressDefaultNativeIntent\s*&&[\s\S]*isFiniteTransform\(weaponNode->world\)' `
    'Dynamic collision must accept the native weapon pose by default only when no prior collision presentation can contaminate it.'
Require-Pattern $weaponAuthority `
    'publishCollisionIsolatedRightNativeWeaponIntent\([\s\S]*_weaponCollisionHandPresentationFromPreviousFrame\[1\][\s\S]*_firingHandIsLeft[\s\S]*ownsWeaponTransform\(\)[\s\S]*weaponNode->parent\s*!=\s*rightHand[\s\S]*tryGetSolverHandTransform\(false,[\s\S]*composeTransforms\([\s\S]*physicalRightHandWorld,[\s\S]*weaponNode->local[\s\S]*_weaponVisualIntentObserver\(' `
    'Native right-hand carry must preserve the current weapon-local animation on a collision-isolated physical hand basis.'
Require-Pattern $weaponAuthority `
    'refreshRightNativeCanonicalFrame\([\s\S]*_weaponCollisionHandPresentationFromPreviousFrame\[1\][\s\S]*isManualOwnershipActive\(\)' `
    'Previous collision presentation must never poison the passive native right-hand canonical calibration.'

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
    'snapshot\.contactActive\s*&&[\s\S]*!snapshot\.teleported' `
    'Visual correction must require an actual active contact and reject teleport recovery samples.'
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
Require-Pattern $runtimeSource `
    'makeGripAuthorityTarget\(_frameRequestedWeaponWorld\)[\s\S]*queueGeneratedKeyframedBodyTarget\([\s\S]*_authorityDriveState[\s\S]*driveGeneratedKeyframedBody\([\s\S]*_authorityProxy[\s\S]*_authorityDriveState[\s\S]*makeContactBodyTargetFromGripAuthority' `
    'Pre-solve authority must drive the hidden keyframed grip proxy, not the colliding compound body.'
Require-Pattern $runtimeSource `
    'updateWeaponGripConstraintContactTau\([\s\S]*rockGrabLinearTau[\s\S]*rockGrabLooseWeaponSharedConstraintLinearTauMultiplier[\s\S]*rockGrabAngularTau[\s\S]*rockGrabLooseWeaponSharedConstraintAngularTauMultiplier[\s\S]*rockGrabTauMin[\s\S]*rockGrabLooseWeaponSharedConstraintCollisionTauMultiplier[\s\S]*advanceToward\([\s\S]*linearMotor->tau[\s\S]*advanceToward\([\s\S]*angularMotor->tau[\s\S]*_contactGraceSolves\s*>\s*0' `
    'Active weapon/world contact must soften both grip motors through the established loose-weapon tau policy.'
Reject-Pattern $runtimeSource `
    'updateWeaponGripConstraintContactTau\([\s\S]{0,400}(requestedTarget|proportionalRecoveryVelocity|constantRecoveryVelocity|maxForce)\s*=' `
    'Contact authority adaptation must not mutate targets, recovery velocities, or force limits.'
foreach ($path in @('src/RockConfig.h', 'src/RockConfig.cpp', 'data/config/ROCK.ini', 'data/mod/ROCK_Config/ROCK.ini')) {
    Reject-Pattern $path `
        'WeaponCollisionDynamicContactPressMaxVelocityHavok' `
        "$path must not retain the unused direct-velocity weapon contact cap after the constraint architecture replacement."
}
Reject-Pattern $runtimeSource `
    '\.dynamicVelocity\s*=\s*true' `
    'The compound contact body must not retain the center-driven dynamic-velocity authority path.'
Require-Pattern $runtimeSource `
    'samplePostSolve\([\s\S]*tryResolveLiveBodyWorldTransform\(world,\s*_body\.getBodyId\(\)' `
    'Post-solve publication must sample the solver-owned compound contact body.'
Require-Order $interaction @(
    '_completedPhysicsSolveSequence\.fetch_add\(',
    '_dynamicWeaponCollision\.samplePostSolve\(',
    '_dynamicHandCollision\.samplePostSolveDeviations\('
) 'The dynamic weapon correction snapshot must be sampled on the post-solve callback clock.'
Require-Order $runtimeSource @(
    'void DynamicWeaponCollisionRuntime::retireProxyLocked\(',
    'destroyGrabConstraint\(_createdWorld,\s*_authorityConstraint\)',
    '_body\.retireDeferred\(bhkWorld\)',
    '_authorityProxy\.retireDeferred\(bhkWorld\)'
) 'Live-world teardown must remove the constraint before deferred retirement of both generated bodies.'
Require-Pattern $runtimeSource `
    'liveOwnerMatches[\s\S]*bhkWorld\s*==\s*_createdBhkWorld[\s\S]*destroyGrabConstraint\(_createdWorld,[\s\S]*_body\.retireDeferred\(bhkWorld\)[\s\S]*_authorityProxy\.retireDeferred\(bhkWorld\)' `
    'Native constraint/body retirement must require proof that the supplied world still owns the generated state.'
Reject-Pattern $runtimeSource `
    'bhkWorld\s*\?\s*bhkWorld\s*:\s*_createdBhkWorld' `
    'Teardown must not fall back to a cached world after live-world ownership is uncertain.'
Require-Pattern $runtimeSource `
    'abandonHavokStateAfterWorldLoss[\s\S]*destroyGrabConstraint\(nullptr,\s*_authorityConstraint\)[\s\S]*_body\.reset\(\)[\s\S]*_authorityProxy\.reset\(\)' `
    'Stale-world cleanup must retire constraint payloads and abandon both wrappers without native removal.'
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
    'Generated dynamic bodies must not pass through the keyframed initializer mislabeled as mass derivation.'
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
Require-Pattern $runtimeSource `
    'signedTranslationStepTowardContactError\([\s\S]*_physicsPreviousRequestedTarget[\s\S]*_physicsRequestedTarget[\s\S]*liveBodyWorld[\s\S]*DWC motor trace:[\s\S]*intentStep=[\s\S]*signedPress=[\s\S]*contactError=[\s\S]*authority\(read/error\)=[\s\S]*tau=[\s\S]*recovery=[\s\S]*force=' `
    'Sustained-contact diagnostics must distinguish continued press from retreat and expose authority tracking plus live motor state.'
Require-Pattern $interaction `
    'applyWeaponCollisionResolvedAuthority[\s\S]*immediateTranslationError[\s\S]*immediateRotationError[\s\S]*DWC visual publication' `
    'Dynamic weapon visual publication must expose immediate node readback evidence.'
Require-Pattern 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' `
    'rockDebugDrawDynamicWeaponColliders[\s\S]*proxyBodyIdForDebug\(\)[\s\S]*DWC ACTIVE[\s\S]*addScreenTextLine\(20\.0f,\s*90\.0f[\s\S]*DWC COMPOUND[\s\S]*authorityBody[\s\S]*children=%u points=%llu[\s\S]*gripPivot[\s\S]*callbacks pair/world/raw/manifold/admit[\s\S]*snapshot read/valid/id/contact/tele' `
    'The dedicated debug flag must draw the compound and expose geometry, authority, pivot, callback, and snapshot telemetry.'

if ($failures.Count -gt 0) {
    Write-Host 'Dynamic weapon collision source boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Dynamic weapon collision source boundary passed.'
