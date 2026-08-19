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
$interaction = 'src/physics-interaction/core/PhysicsInteraction.cpp'
$contacts = 'src/physics-interaction/core/PhysicsInteractionContacts.inl'
$layers = 'src/physics-interaction/collision/CollisionLayerPolicy.h'

# The validated dynamic compound and hand-interaction graph ship enabled.
# Layer-44 weapon hulls continue to own ordinary gameplay contact evidence.
Require-Pattern 'src/RockConfig.h' `
    'rockWeaponCollisionDynamicBoxEnabled\s*=\s*true' `
    'Dynamic weapon collision must default enabled in compiled configuration.'
foreach ($ini in @('data/config/ROCK.ini', 'data/mod/ROCK_Config/ROCK.ini')) {
    Require-Pattern $ini `
        '(?m)^bWeaponCollisionDynamicBoxEnabled\s*=\s*true\s*$' `
        "$ini must ship dynamic weapon collision enabled."
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
    'isDynamicWeaponProxyObstacleLayer\(std::uint32_t layer\)[\s\S]*isWorldSurfaceLayer\(layer\)[\s\S]*isDynamicWorldCarLayer\(layer\)[\s\S]*buildRockDynamicWeaponProxyExpectedMask\([\s\S]*rightHandInteractionEnabled[\s\S]*ROCK_LAYER_DYNAMIC_RIGHT_HAND_PROXY[\s\S]*leftHandInteractionEnabled[\s\S]*ROCK_LAYER_DYNAMIC_LEFT_HAND_PROXY' `
    'The dynamic weapon proxy row must retain world/car obstacles and gate each dynamic hand independently.'
Require-Pattern $layers `
    'buildRockDynamicWorldCarExpectedMask[\s\S]*withLayer\(mask, ROCK_LAYER_DYNAMIC_HAND_PROXY\)[\s\S]*withLayer\(mask, ROCK_LAYER_DYNAMIC_LEFT_HAND_PROXY\)[\s\S]*withLayer\(mask, ROCK_LAYER_DYNAMIC_WEAPON_PROXY\)[\s\S]*withLayer\(mask, FO4_LAYER_CHARCONTROLLER\)' `
    'Tagged car rows must symmetrically admit both hands and the dynamic weapon without exposing generated gameplay colliders.'
Require-Pattern $layers `
    'applyRockGeneratedLayerPolicies[\s\S]*applyRockDynamicWeaponProxyLayerPolicy\([\s\S]*dynamicWeaponRightHandInteractionEnabled[\s\S]*dynamicWeaponLeftHandInteractionEnabled' `
    'Layer 51 must be registered with side-specific free-hand collision eligibility.'
Require-Pattern 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'synchronizeDynamicWeaponHandCollisionRoles[\s\S]*weaponCollisionAttachedHands\(\)[\s\S]*!attachedHands\.right[\s\S]*!attachedHands\.left[\s\S]*registerCollisionLayer\(world\)' `
    'Only hands not attached to the weapon may physically push its dynamic proxy.'
Require-Pattern 'src/physics-interaction/weapon/TwoHandedGrip.h' `
    'weaponCollisionAttachedHands\(\)[\s\S]*selectAttachedHands\([\s\S]*isFiringGripOccupied\(\)[\s\S]*partGrip\(true\)\.active[\s\S]*partGrip\(false\)\.active' `
    'Collision filtering and resolved hand coupling must share one attachment-role policy.'

# World contact remains exactly one dynamic body whose child instances follow
# the shared layer-44 hull sources. A second tiny body is permitted only as the
# noncolliding keyframed constraint authority.
Require-Pattern $runtimeHeader `
    'BethesdaPhysicsBody\s+_body' `
    'The dynamic weapon runtime must own one explicit world-contact body.'
Require-Pattern $runtimeHeader `
    'BethesdaPhysicsBody\s+_authorityProxy[\s\S]*ActiveConstraint\s+_authorityConstraint' `
    'The dynamic weapon runtime must own one noncolliding authority body and one finite constraint.'
Require-Pattern $runtimeHeader `
    'DynamicCompoundShape\s+_compoundShape[\s\S]*_pendingCompoundChildTransforms[\s\S]*_queuedCompoundPoseSequence[\s\S]*_consumedCompoundPoseSequence' `
    'The one-body runtime must retain native dynamic-compound ownership and a bounded main-to-physics pose queue.'
Reject-Pattern $runtimeHeader `
    'std::array\s*<\s*BethesdaPhysicsBody|std::vector\s*<\s*BethesdaPhysicsBody' `
    'The one-body compound runtime must not expand into a body bank.'
Require-Pattern $runtimeSource `
    'getCompoundGeometrySnapshot\(compoundGeometry\)[\s\S]*sourceChild\.shape[\s\S]*sourceChild\.shapeInWeapon[\s\S]*pendingCompoundShape\.create\(compoundChildren\)' `
    'The dynamic body must reuse the exact generation-bound layer-44 child shapes and their current source poses.'
Reject-Pattern $runtimeSource `
    'makeBoxCornerPointsHavok|ROCK_DynamicWeaponBox' `
    'The superseded eight-corner box shape path must not survive beside the compound geometry provider.'
Require-Pattern $runtimeSource `
    'pendingCompoundShape\.create\(compoundChildren\)[\s\S]*_compoundShape\s*=\s*std::move\(pendingCompoundShape\)[\s\S]*BethesdaMotionType::Dynamic[\s\S]*ROCK_DynamicWeaponCompound' `
    'The live compound must remain the shape of one real dynamic Bethesda body.'
Reject-Pattern $runtimeSource `
    'buildStaticCompoundShape\(compoundChildren\)|buildConvexShapeFromLocalHavokPoints' `
    'The dynamic weapon runtime must not retain its superseded fixed snapshot/rebuilt-convex path.'
Require-Pattern $runtimeSource `
    'queueCompoundChildTransforms\([\s\S]*getCompoundChildPoseSnapshot\([\s\S]*makeCompoundChildTransform\([\s\S]*_queuedCompoundPoseSequence' `
    'Every accepted game frame must queue the current source-node poses without rebuilding child geometry.'
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

Require-Order $interaction @(
    '_twoHandedGrip\.beginWeaponCollisionPresentationFrame\(',
    'previousWeaponCollisionPresentationWasLive\(',
    '_dynamicWeaponCollision\.beginFrame\(',
    '_twoHandedGrip\.update\(',
    '_twoHandedGrip\.applyGunstockAlignment\(',
    '_dynamicWeaponCollision\.finishFrame\(',
    'applyWeaponCollisionResolvedAuthority\(',
    'finishWeaponCollisionPresentationFrame\(',
    '_weaponCollision\.updateBodiesFromCurrentSourceTransforms\('
) 'Collision hand presentation must be witnessed before isolated intent capture and released after retained contact presentation ends.'
Require-Pattern $runtimePolicy `
    'selectAttachedHands\([\s\S]*partCarry[\s\S]*firingGripOccupied[\s\S]*leftPartGripActive[\s\S]*rightPartGripActive[\s\S]*supportHandPresentationAllowed[\s\S]*firingHandIsActuallyLeft[\s\S]*!supportHandPresentationAllowed' `
    'Collision hand coupling must select attached hands while allowing native reload to exclude the physical support hand.'
Require-Pattern $runtimePolicy `
    'reframeAttachedHand\([\s\S]*invertTransform\(requestedWeaponWorld\)[\s\S]*requestedHandWorld[\s\S]*resolvedWeaponWorld[\s\S]*handWeaponLocal' `
    'Attached IK hands must preserve their exact pre-collision weapon-local relation under the resolved weapon pose.'
Require-Pattern $interaction `
    'finishWeaponCollisionPresentationFrame\([\r\n\s]*dynamicWeaponFrame\.publishVisualAuthority\)' `
    'The collision hand claim must be released when retained contact presentation ends, including proxy-active free space.'
Require-Pattern $interaction `
    'applyWeaponCollisionResolvedAuthority\([\s\S]{0,250}dynamicWeaponFrame\.requestedWeaponWorld[\s\S]{0,250}dynamicWeaponFrame\.resolvedWeaponWorld' `
    'The collision-free intent and physics-resolved pose must cross the presentation boundary together.'
Require-Pattern $interaction `
    'suppressDefaultNativeWeaponIntent\s*=\s*[\r\n\s]*_twoHandedGrip\.previousWeaponCollisionPresentationWasLive\(\)[\s\S]*_dynamicWeaponCollision\.beginFrame\([\s\S]*suppressDefaultNativeWeaponIntent' `
    'A collision-contaminated native weapon pose must not remain as the fallback frame intent when isolated driver reconstruction fails.'
Require-Pattern $runtimeSource `
    '_frameHasIntent\s*=[\s\S]*_frameAcceptingIntent\s*&&[\s\S]*!suppressDefaultNativeIntent\s*&&[\s\S]*isFiniteTransform\(weaponNode->world\)' `
    'Dynamic collision must accept the native weapon pose by default only when no prior collision presentation can contaminate it.'
# Contact callbacks identify positive-point solved manifolds; the current
# post-solve body snapshot owns the actual collision-resolved pose.
Require-Order $contacts @(
    'isProxyBodyIdAtomic\(bodyIdA\)',
    'tryReadFilterInfo\(',
    'isDynamicWeaponProxySolverObstacleLayer\(otherLayer\)',
    'ensureRawContactPoint\(\)',
    'recordObstacleContactCallback\(',
    'shouldSkipContactSignalBeforeLayerRead\('
) 'A raw proxy/obstacle contact point must be captured before the normal gameplay-contact prefilter discards layer 51.'
Require-Order $contacts @(
    'kManifoldProcessedEventType\s*=\s*static_cast<RE::hknpEventType::Enum>\(2\)',
    's_manifoldProcessedEventBridge',
    'handleManifoldProcessedEvent\(world,\s*contactEventData\)'
) 'The verified key-2 signal must own a distinct retained bridge and callback route.'
Require-Order $contacts @(
    'handleManifoldProcessedEvent\(',
    'recordSize\s*!=\s*kExpectedRecordSize\s*\|\|\s*eventKey\s*!=\s*kManifoldProcessedEventKey',
    'manifoldPointCount\s*=',
    'isProxyBodyIdAtomic\(bodyIdA\)',
    'tryReadFilterInfo\(',
    'recordObstacleManifoldProcessedCallback\('
) 'Processed-manifold admission must retain the verified point count and validate the exact proxy/obstacle pair.'
Require-Pattern $runtimePolicy `
    'hasSolvedProcessedManifoldContact\([\s\S]*pointCount\s*>\s*0[\s\S]*pointCount\s*<=\s*kMaximumProcessedManifoldContactPoints' `
    'Only FO4VR key-2 records with one to four solved manifold points may become contact evidence.'
Require-Pattern $runtimeSource `
    '_processedManifoldCallbackSequenceAtomic\.fetch_add\(1[\s\S]*hasSolvedProcessedManifoldContact\([\s\S]*manifoldPointCount\)[\s\S]*_contactSequenceAtomic\.fetch_add\(1' `
    'Point-free and terminal processed-manifold records must remain diagnostic and never refresh the contact witness.'
Require-Pattern $runtimeSource `
    '_rawPointCallbackSequenceAtomic\.fetch_add\(1[\s\S]*_contactWorldAtomic\.store[\s\S]*_contactSequenceAtomic\.fetch_add\(1' `
    'Positive raw world and dynamic-hand contacts must refresh the shared interaction witness.'
Require-Pattern $runtimeSource `
    '_processedManifoldCallbackSequenceAtomic\.fetch_add\(1[\s\S]*hasSolvedProcessedManifoldContact\([\s\S]*_contactWorldAtomic\.store[\s\S]*_contactSequenceAtomic\.fetch_add\(1' `
    'Positive processed world and dynamic-hand manifolds must refresh the shared interaction witness.'
Reject-Pattern $runtimeSource `
    'hasSolvedProcessedManifoldContact\([\s\S]{0,300}isDynamicWeaponProxyObstacleLayer\(otherLayer\)[\s\S]{0,300}_contactWorldAtomic\.store' `
    'A positive dynamic-hand manifold must not be discarded before weapon presentation admission.'
Require-Pattern $runtimePolicy `
    'kProcessedManifoldContactRetentionSeconds\s*=\s*0\.35f[\s\S]*advanceProcessedManifoldContactRetention\([\s\S]*teleported[\s\S]*positivePointWitness[\s\S]*retainedSeconds\s*-\s*elapsedSeconds' `
    'Positive manifold authority must use a solver-rate-independent, teleport-safe retention window.'
Require-Pattern $runtimeSource `
    '_contactRetentionSeconds\s*=\s*[\s\S]*advanceProcessedManifoldContactRetention\([\s\S]*newMatchingContact[\s\S]*_physicsDriveTeleported[\s\S]*driveDeltaSeconds\(timing\)[\s\S]*snapshot\.contactActive\s*=\s*_contactRetentionSeconds\s*>\s*0\.0f' `
    'Post-solve authority must bridge positive-contact callback bursts using physics time.'
Reject-Pattern $runtimeSource `
    'kContactGraceSolves|_contactGraceSolves' `
    'Dynamic weapon contact authority must not expire through a solver-rate-dependent tick count.'
Require-Pattern $runtimeSource `
    'snapshotIdentityCurrent\s*&&[\s\S]{0,100}!snapshot\.teleported' `
    'Every current non-teleport post-solve sample must be eligible to retain weapon presentation ownership.'
Require-Pattern $runtimeSource `
    'correctionFinite[\s\S]*std::isfinite\(result\.translationCorrectionGameUnits\)[\s\S]*std::isfinite\(result\.rotationCorrectionDegrees\)' `
    'Physics-resolved visual correction must reject only corrupt non-finite deltas.'
Reject-Pattern $runtimeSource `
    'correctionWithinSafetyEnvelope|kMaxVisualCorrectionRotationDegrees' `
    'A finite solver pose must not be discarded by an arbitrary visual distance or angle limit.'

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
Require-Order $runtimeSource @(
    '_compoundShape\.updateTransforms\(_pendingCompoundChildTransforms\)',
    'driveGeneratedKeyframedBody\('
) 'The native child tree and owner notifications must refresh before the authority body drives into the pre-collide step.'
Require-Pattern $runtimeSource `
    'updateWeaponGripConstraintContactTau\([\s\S]*rockGrabLinearTau[\s\S]*rockGrabLooseWeaponSharedConstraintLinearTauMultiplier[\s\S]*rockGrabAngularTau[\s\S]*rockGrabLooseWeaponSharedConstraintAngularTauMultiplier[\s\S]*rockGrabTauMin[\s\S]*rockGrabLooseWeaponSharedConstraintCollisionTauMultiplier[\s\S]*advanceToward\([\s\S]*linearMotor->tau[\s\S]*advanceToward\([\s\S]*angularMotor->tau[\s\S]*_contactRetentionSeconds\s*>\s*0\.0f' `
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
Require-Pattern $runtimePolicy `
    'advanceFreeSpaceDivergenceRecovery\([\s\S]*!contactActive[\s\S]*translationGapGameUnits\s*>\s*divergenceThresholdGameUnits[\s\S]*dwellSeconds\s*>=\s*requiredDwell' `
    'Runaway body recovery must require bounded free-space divergence dwell and remain disabled during retained contact.'
Require-Pattern $runtimeSource `
    'advanceFreeSpaceDivergenceRecovery\([\s\S]*_contactRetentionSeconds\s*>\s*0\.0f[\s\S]*rockWeaponCollisionDynamicDivergenceTeleportGameUnits[\s\S]*rockWeaponCollisionDynamicDivergenceTeleportDwellSeconds[\s\S]*placeGeneratedKeyframedBodyImmediately\([\s\S]*_body,[\s\S]*_physicsRequestedTarget[\s\S]*_physicsDriveTeleported\s*=\s*driveResult\.teleported\s*\|\|\s*contactBodyRecovered' `
    'The contact body must recover only from sustained free-space runaway, then invalidate the recovered presentation sample.'
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
    'The dynamic weapon proxy must opt into key-2 processed-manifold events through the verified mode-0 body flag path.'
Reject-Pattern 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' `
    'kGeneratedBodyRuntimeFlags\s*=\s*0x0*4020000' `
    'Processed-manifold event opt-in must not leak into the shared generated-body defaults.'

# The dormant overlay flag is now the primary in-game shape/contact diagnostic.
# A surviving runtime mismatch must distinguish callback admission, snapshot
# admission, and immediate visual-authority readback without hot-path log spam.
Require-Pattern $runtimeHeader `
    '_proxyPairCallbackSequenceAtomic[\s\S]*_obstacleCallbackSequenceAtomic[\s\S]*_rawPointCallbackSequenceAtomic[\s\S]*_processedManifoldCallbackSequenceAtomic[\s\S]*_contactSequenceAtomic' `
    'Dynamic weapon diagnostics must retain separate callback-stage counters.'
Require-Pattern $runtimeSource `
    'rockDebugDrawDynamicWeaponColliders[\s\S]*ROCK_LOG_SAMPLE_INFO\([\s\S]*DWC pipeline:[\s\S]*snapshot\(read/valid/identity/contact/teleport\)' `
    'Dynamic weapon pipeline diagnostics must be debug-gated and rate-limited.'
Require-Pattern $runtimeSource `
    'signedTranslationStepTowardContactError\([\s\S]*_physicsPreviousRequestedTarget[\s\S]*_physicsRequestedTarget[\s\S]*liveBodyWorld[\s\S]*DWC motor trace:[\s\S]*intentStep=[\s\S]*signedPress=[\s\S]*contactError=[\s\S]*authority\(read/error\)=[\s\S]*tau=[\s\S]*recovery=[\s\S]*force=' `
    'Sustained-contact diagnostics must distinguish continued press from retreat and expose authority tracking plus live motor state.'
Require-Pattern $runtimeSource `
    '_debugSnapshot\.contactActive\s*=\s*snapshot\.contactActive[\s\S]*const bool correctionVisible\s*=[\s\S]*decideVisualAuthority\([\s\S]*snapshot\.contactActive,[\s\S]*correctionVisible[\s\S]*result\.publishVisualAuthority\s*=\s*visualDecision\.publish[\s\S]*result\.resolvedWeaponWorld\s*=\s*visualDecision\.useResolvedWeaponWorld\s*\?[\s\S]*resolvedWeaponWorld\s*:[\s\S]*_frameRequestedWeaponWorld' `
    'Retained contact must own presentation continuously while proxy-active free space yields to normal grip/native hand authority.'
Require-Pattern $runtimeSource `
    'FRIK V2 consumes tagged hand transforms during its next skeleton[\s\S]*positive-point world/hand manifold[\s\S]*higher-priority hand claim[\s\S]*support hand from the firing-hand driver[\s\S]*free space yields' `
    'The source must document why free-space FRIK collision ownership causes support-hand wobble and must yield.'
Require-Order $runtimeSource @(
    '_rawContactOtherBodyIdAtomic\.store\(otherBodyId',
    '_rawContactPointHavokAtomic\[axis\]\.store\(',
    '_rawContactWitnessSequenceAtomic\.fetch_add\(1',
    '_contactSequenceAtomic\.fetch_add\(1'
) 'Raw contact position and normal must publish before the callback sequence release.'
Require-Pattern $runtimeSource `
    'contactEpisodeStarted[\s\S]*_contactEpisode[\s\S]*snapshotBody\([\s\S]*otherBodyId[\s\S]*rawContactPointGame[\s\S]*publishContactDiagnosticSnapshot\(diagnostic\)' `
    'Each distinct world-contact episode must retain bounded body identity and raw point evidence in the post-solve snapshot.'
Require-Pattern $runtimeSource `
    'contactEpisodeStarted\s*&&[\s\S]*rockDebugDrawDynamicWeaponColliders[\s\S]*publishContactDiagnosticSnapshot\(diagnostic\)' `
    'Extended contact evidence must publish only for a distinct episode while dynamic-collider diagnostics are enabled.'
Require-Pattern $interaction `
    'contactEpisodeStarted[\s\S]*resolveBodyToRef\([\s\S]*tryFindCurrentWeaponSurfaceNearPoint\([\s\S]*tryGetWeaponContactDebugInfo\([\s\S]*DWC contact witness:[\s\S]*DWC contact transforms:' `
    'The main-thread diagnostic must resolve the contacted reference and nearest live weapon part once per contact episode.'
Require-Pattern $interaction `
    'applyWeaponCollisionResolvedAuthority[\s\S]*immediateTranslationError[\s\S]*immediateRotationError[\s\S]*DWC visual publication' `
    'Dynamic weapon visual publication must expose immediate node readback evidence.'
Require-Pattern 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.cpp' `
    'rockDebugDrawDynamicWeaponColliders[\s\S]*proxyBodyIdForDebug\(\)[\s\S]*DWC ACTIVE[\s\S]*addScreenTextLine\(20\.0f,\s*90\.0f[\s\S]*DWC COMPOUND[\s\S]*authorityBody[\s\S]*children=%u points=%llu[\s\S]*gripPivot[\s\S]*callbacks pair/obstacle/raw/manifold/admit[\s\S]*snapshot read/valid/id/contact/tele' `
    'The dedicated debug flag must draw the compound and expose geometry, authority, pivot, callback, and snapshot telemetry.'

if ($failures.Count -gt 0) {
    Write-Host 'Dynamic weapon collision source boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Dynamic weapon collision source boundary passed.'
