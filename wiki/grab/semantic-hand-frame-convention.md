# Semantic Hand Frame Convention For Grab

Date: 2026-05-21

Project: ROCK

Source used: current local ROCK source only.

Confidence: high for current ROCK implementation; runtime visual validation is still useful when tuning offsets.

## Purpose

ROCK grab code uses a semantic hand frame derived from the game root-flattened skeleton. This frame is the source of truth for palm direction, grab pocket placement, pinch direction fallback, finger curl planes, public palm API output, and grab debug visuals.

The convention is:

```text
local +X = fingers / knuckle direction
local +Y = palm depth / back of hand
local -Y = palm face / palm normal
local +Z = across palm
```

This means palm-facing logic should normally use `-Y`, not raw hand `-Z`.

## Source Chain

The current production path is:

1. `DirectSkeletonBoneReader::capture(...)`
   - File: `src/physics-interaction/hand/DirectSkeletonBoneReader.cpp`
   - Source mode: `DebugSkeletonBoneSource::GameRootFlattenedBoneTree`
   - Resolves the current root-flattened tree through `f4vr::getFlattenedBoneTree()`.
   - Uses `f4vr::getRootNode()` as the skeleton/root identity for this source.

2. `DirectSkeletonBoneSnapshot`
   - File: `src/physics-interaction/hand/HandSkeleton.h`
   - Owns a frame-local copy of selected flattened-tree bone records.
   - Stores copied data; consumers must not retain raw tree pointers from it.

3. `buildSemanticHandFrameFromSnapshot(...)`
   - File: `src/physics-interaction/hand/HandSkeleton.h`
   - Reads one hand bone and the five finger base bones from the snapshot.
   - Builds a `SemanticHandFrame`.

4. `HandBoneCache::resolve()`
   - File: `src/physics-interaction/hand/HandSkeleton.h`
   - Captures the root-flattened snapshot once for the frame.
   - Builds both right and left `SemanticHandFrame` values from that same snapshot.
   - Stores copied frame data in `CachedSemanticHandFrameData`.

5. Frame input and API consumers
   - `src/physics-interaction/core/PhysicsInteractionFrame.inl` reads `HandBoneCache::getSemanticHandFrame(...)`.
   - `src/api/ROCKApi.cpp` reads the cached frame through `PhysicsInteraction::tryGetRootFlattenedSemanticHandFrame(...)`.

Hot-path grab code should prefer the cached frame from `HandBoneCache`. Do not repeatedly recapture the skeleton from a frame loop unless the code is explicitly diagnostic or isolated.

## What The Flat-Root Tree Provides

The root-flattened tree is exposed to ROCK as a `BSFlattenedBoneTree`. The reader copies the parts ROCK needs into `DirectSkeletonBoneSnapshot`.

Snapshot-level data:

```text
valid
inPowerArmor
mode
source
skeleton pointer identity
boneTree pointer identity
totalBoneCount
requiredResolvedCount
bones
missingRequiredBones
```

Per-bone data copied into `DirectSkeletonBoneEntry`:

```text
name
treeIndex
parentTreeIndex
drawableParentSnapshotIndex
world NiTransform
included flag
```

The important transform is `world`, an `RE::NiTransform`. For grab convention work this gives:

```text
world.translate = bone world position
world.rotate    = bone world rotation
world.scale     = bone scale
```

ROCK does not derive grab authority from authored local handspace. It derives it from the current world transforms in the root-flattened skeleton tree.

## Bones Used For The Semantic Frame

For each hand, `buildSemanticHandFrameFromSnapshot(...)` requires:

```text
Right hand root: RArm_Hand
Left hand root:  LArm_Hand

Right finger bases:
RArm_Finger11
RArm_Finger21
RArm_Finger31
RArm_Finger41
RArm_Finger51

Left finger bases:
LArm_Finger11
LArm_Finger21
LArm_Finger31
LArm_Finger41
LArm_Finger51
```

The full finger snapshot path can also read all three segments for each finger:

```text
Finger11, Finger12, Finger13
Finger21, Finger22, Finger23
Finger31, Finger32, Finger33
Finger41, Finger42, Finger43
Finger51, Finger52, Finger53
```

The semantic palm frame only needs the hand root and five base joints. Finger pose and debug code can use the longer chain snapshot when it needs thumb/index pad positions or curve landmarks.

## How The Axes Are Built

The builder starts from:

```text
hand = RArm_Hand or LArm_Hand world transform
fingerBases = five base-joint world positions
crossPalmDirection = hand local +Z rotated into world
```

It computes:

```text
fingerBaseCenterWorld = average(fingerBases)
fingerForwardWorld    = normalize(fingerBaseCenterWorld - hand.translate)
acrossPalmWorld       = normalize(project crossPalmDirection onto plane perpendicular to fingerForwardWorld)
palmDepthWorld        = normalize(cross(acrossPalmWorld, fingerForwardWorld))
acrossPalmWorld       = normalize(cross(fingerForwardWorld, palmDepthWorld))
fingerForwardWorld    = normalize(cross(palmDepthWorld, acrossPalmWorld))
palmFaceWorld         = -palmDepthWorld
```

The repeated cross products deliberately re-orthonormalize the basis. The result is stable even if the raw hand bone rotation is not a perfect palm frame.

## Matrix Storage

The semantic palm matrix stores the basis as columns:

```text
column 0 = fingerForwardWorld = local X
column 1 = palmDepthWorld     = local Y
column 2 = acrossPalmWorld    = local Z
```

So transforming a local semantic direction into world is:

```text
worldDirection =
    fingerForwardWorld * local.x +
    palmDepthWorld     * local.y +
    acrossPalmWorld    * local.z
```

Examples:

```text
( 1,  0,  0) = toward fingers
( 0,  1,  0) = toward back of hand
( 0, -1,  0) = palm face / palm normal
( 0,  0,  1) = across palm
```

The current far pointing default is semantic `(0, -1, 0)` with `bReverseFarGrabNormal = false`, so the far ray points along the palm face by default.

## Palm Anchor

`SemanticHandFrame::palmAnchorWorld` is not just the raw `RArm_Hand` or `LArm_Hand` transform.

The anchor is built by:

1. Averaging the hand position with the five finger base positions to estimate the palm center.
2. Removing any current depth component from that center so the point lies in the semantic palm plane.
3. Moving the anchor a small amount toward the palm face:

```text
palmAnchorWorld.translate = palmCenter + palmDepthWorld * (-abs(palmDepth) / 3)
```

With the default builder depth of `0.75`, this means the anchor is offset toward local `-Y`, the palm face.

The anchor rotation is the semantic palm matrix, not the raw hand bone rotation.

## Fields In `SemanticHandFrame`

```text
rawHandWorld
```

The original root-flattened hand transform from the skeleton tree.

```text
palmAnchorWorld
```

The generated semantic palm transform. This is the main transform to use for palm-space grab logic.

```text
fingerBaseCenterWorld
```

Average world position of the five base finger joints.

```text
fingerForwardWorld
```

World-space semantic +X.

```text
palmDepthWorld
```

World-space semantic +Y. This points toward the back/depth side of the hand.

```text
palmFaceWorld
```

World-space semantic -Y. This is the palm normal used by grab-facing logic.

```text
acrossPalmWorld
```

World-space semantic +Z.

```text
palmLength
```

Distance from the hand root to the averaged finger base center.

```text
valid
```

True only when all required source points were present and produced finite axes.

## Current Grab Consumers

Dynamic grab frame input uses the cached semantic frame to populate:

```text
grabAnchorWorld
palmNormalWorld
pointingWorld
pinchDirectionWorld
```

Important paths:

```text
src/physics-interaction/core/PhysicsInteractionFrame.inl
src/physics-interaction/hand/HandSkeleton.h
src/physics-interaction/grab/GrabFinger.h
src/api/ROCKApi.cpp
```

When a generated grab authority proxy exists, some grab logic uses the proxy frame for final held-object authority. That proxy follows the same semantic convention:

```text
X  = fingers
Y  = palm depth / back
-Y = palm face
Z  = across palm
```

## Finger Snapshot Data

`resolveLiveFingerSkeletonSnapshot(...)` uses the same root-flattened tree source. It returns:

```text
fingers[5]
each finger has points[3]
palmNormalWorld
palmNormalValid
valid
```

The points are world positions copied from the flattened tree:

```text
points[0] = base segment
points[1] = middle/distal-ish segment depending on FO4 bone naming
points[2] = tip/end segment
```

ROCK uses this data for finger pose, thumb/index pinch pocket debug positions, and mesh-contact solving. It should not be used to redefine the hand-space convention every frame after posing, because visual pose publication can move the apparent finger relation and cause feedback.

## What Not To Use

Do not use `HandFrame.h` legacy authored handspace for new grab work.

Legacy authored handspace means:

```text
authored X = fingers
authored Y = cross-palm
authored Z = palm thickness
```

That is not the production grab convention anymore. `HandFrame.h` remains only for old diagnostics and the isolated two-handed weapon compatibility island.

Do not use raw hand local `-Z` as palm normal. The semantic palm normal is `palmFaceWorld`, which is semantic local `-Y`.

Do not treat `FirstPersonDiagnosticOnly` skeleton source as grab authority. It exists for diagnostics.

Do not copy two-handed weapon handspace logic into dynamic object grab. That path is explicitly quarantined until it is replaced or repaired.

## Failure Behavior

The frame must fail closed.

`HandBoneCache::resolve()` clears resolved state and returns false when:

```text
the root-flattened tree is unavailable
RArm_Hand or LArm_Hand is missing
any required finger base for either semantic frame is missing
the derived frame is invalid
```

`PhysicsInteractionFrame.inl` disables hand input when the cache is not ready or the semantic frame cannot be copied.

This is intentional. A missing or incoherent semantic frame is worse than no grab input, because stale palm axes can place the pocket or normal on the wrong side of the hand.

## Config Values Interpreted In This Convention

These values are interpreted in semantic/generated palm space:

```text
fPointingVectorHandspaceX/Y/Z
fGrabPinchDetectionDirectionHandspaceX/Y/Z
fRightGrabAuthorityProxyOffsetX/Y/ZGameUnits
fLeftGrabAuthorityProxyOffsetX/Y/ZGameUnits
```

Current important default:

```text
fPointingVectorHandspaceX = 0.0
fPointingVectorHandspaceY = -1.0
fPointingVectorHandspaceZ = 0.0
bReverseFarGrabNormal = false
```

That means the far ray is semantic palm-face by default.

Legacy `fPalmNormalHandspace*` and `fRight/LeftGrabPivotAHandspace*` are not the current dynamic grab authority. Treat them as legacy/debug unless code near the caller says otherwise.

## Debugging This Convention

Useful production INI visualization keys:

```text
bDebugShowPalmVectors
bDebugShowHandAxes
bDebugDrawGrabPockets
bDebugDrawGrabAuthorityProxy
bDebugShowGrabPivots
bDebugShowGrabPocketNormal
bDebugDrawGrabContactPatch
bDebugShowRootFlattenedFingerSkeletonMarkers
bDebugShowSkeletonBoneVisualizer
bDebugDrawSkeletonBoneAxes
```

For the current convention check:

```text
palm vector / normal should point out of the palm face
far ray should point along semantic -Y when configured as (0, -1, 0)
palm pocket and authority proxy should agree on front/back
left and right hands should use the same semantic signs
```

## Tests That Guard The Convention

Relevant tests:

```text
tests/HandColliderFramePolicyTests.cpp
tests/SemanticHandFrameSourceTests.ps1
tests/GrabAuthorityProxyFrameSourceTests.ps1
tests/GrabOrientationBasisTelemetrySourceTests.ps1
```

The key protected invariants are:

```text
semantic X matches generated palm X
semantic Y matches generated palm depth/back Y
semantic palm face is negative generated Y
semantic Z matches generated across-palm Z
frame input uses HandBoneCache semantic data, not legacy HandFrame pointing
public palm API reads cached SemanticHandFrame
legacy HandFrame.h carries a warning against new grab use
```

## Rule For Future Developers

If code needs palm-space meaning for grab, start from `SemanticHandFrame` or the generated grab authority proxy frame. Do not start from raw hand bone axes, old authored handspace, or two-handed weapon support-grip code.

If a new feature needs another semantic direction, define it in this coordinate system:

```text
X  = fingers
Y  = back/depth
-Y = palm face
Z  = across palm
```

Then transform it through `transformSemanticHandFrameDirection(...)` or the equivalent generated proxy transform.
