param(
    [Parameter(Mandatory = $true)]
    [string]$Root
)

$ErrorActionPreference = "Stop"

$runtime = Join-Path $Root "src/physics-interaction/contact/SoftContactRuntime.cpp"
$math = Join-Path $Root "src/physics-interaction/contact/SoftContactMath.h"
$worldPolicy = Join-Path $Root "src/physics-interaction/contact/SoftContactWorldPolicy.h"
$physics = Join-Path $Root "src/physics-interaction/core/PhysicsInteraction.cpp"

foreach ($path in @($runtime, $math, $worldPolicy, $physics)) {
    if (-not (Test-Path -LiteralPath $path)) {
        throw "Required soft-contact source path missing: $path"
    }
}

$runtimeText = Get-Content -LiteralPath $runtime -Raw
$mathText = Get-Content -LiteralPath $math -Raw
$worldPolicyText = Get-Content -LiteralPath $worldPolicy -Raw
$physicsText = Get-Content -LiteralPath $physics -Raw

if ($runtimeText -match "HavokOffsets|BodyCollisionControl|applyRock.*LayerPolicy|SetBodyCollisionFilterInfo|setFilterInfo|setLayer|collisionMatrix|kFilter_CollisionMatrix") {
    throw "SoftContactRuntime must remain a ROCK visual solver and must not edit native Havok collision layers or offsets."
}

if ($mathText -notmatch "visual-authority half of the interaction system") {
    throw "SoftContactMath must keep the implementation note explaining native evidence plus visual hand authority."
}

if ($runtimeText -notmatch "castSelectionSphere") {
    throw "World soft contact must use the existing swept-sphere query boundary."
}

if ($runtimeText -notmatch "kWorldProbeRestQueryCooldownSeconds") {
    throw "World soft contact must keep a bounded rest-query path so contact can acquire without large controller motion."
}

if ($runtimeText -notmatch "rockSoftContactWorldContactPaddingGameUnits" -or $runtimeText -notmatch "queryRadiusPadding") {
    throw "World soft contact must keep query padding separate from actual contact padding to avoid sticky release."
}

if ($runtimeText -notmatch "weaponHandRadiusPadding" -or
    $runtimeText -notmatch "rockSoftContactWeaponHandRadiusPaddingGameUnits" -or
    $runtimeText -notmatch "rockSoftContactWeaponHandMaxCorrectionGameUnits" -or
    $mathText -notmatch "projectCompliantTrackedMagnetCorrection") {
    throw "Weapon-hand soft contact must have a separate compliant current-frame response profile from body/world contact."
}

if ($runtimeText -notmatch "candidateResponseWinsTie" -or
    $runtimeText -notmatch "responseScaleForCandidate" -or
    $mathText -notmatch "preferStrongerContactResponse") {
    throw "Soft-contact candidate arbitration must compare the effective response ROCK applies, not raw penetration alone."
}

if ($runtimeText -notmatch "weapon-hand soft contact: penetration" -or
    $runtimeText -notmatch "entry\.responseScale" -or
    $runtimeText -notmatch "entry\.correctionLength") {
    throw "Weapon-hand soft-contact tuning must expose effective response scale and correction length for diagnostics."
}

if ($runtimeText -match "rockSoftContactWorldReleaseHysteresisGameUnits|ContactState::Recovering|rockSoftContactRecoverySmoothingSpeed|rockSoftContactSmoothingSpeed|rockSoftContactWorldSmoothingSpeed") {
    throw "Soft contact must not retain recovery/hysteresis visual authority after the tracked collider is no longer penetrating."
}

if ($runtimeText -match "smoothCorrection\(") {
    throw "SoftContactRuntime must not smooth contact correction; hard-stop projection must be recomputed from the tracked collider every frame."
}

if ($runtimeText -notmatch "releasedCachedPlaneThisFrame") {
    throw "World soft contact must track the first cached-plane release frame to gate immediate stale reacquisition."
}

if ($runtimeText -notmatch "shouldAllowPostReleaseReentrySweep") {
    throw "World soft contact post-release sweeps must only reacquire when the tracked hand moves back into the surface."
}

if ($runtimeText -notmatch "withinTangentDriftLimit" -or $runtimeText -notmatch "rockSoftContactWorldCachedPlaneMaxTangentDriftGameUnits") {
    throw "World soft contact cached planes must be bounded so stale evidence does not behave like an infinite sticky plane."
}

if ($runtimeText -notmatch "loggedWorldQueryPadding" -or $runtimeText -notmatch "effectiveQueryPadding") {
    throw "World soft contact logs must report the effective query padding used by the runtime."
}

if ($runtimeText -notmatch "candidatePriority" -or $runtimeText -notmatch "ContactKind::WorldStatic") {
    throw "World soft contact must keep explicit candidate priority so static hard stops win over weaker visual contacts."
}

$ownerGateIndex = $runtimeText.IndexOf("const bool ownedByStrongerSystem")
$worldSolveIndex = if ($ownerGateIndex -ge 0) { $runtimeText.IndexOf("solveWorldStaticContact", $ownerGateIndex) } else { -1 }
if ($ownerGateIndex -lt 0 -or $worldSolveIndex -lt 0 -or $ownerGateIndex -gt $worldSolveIndex) {
    throw "Soft-contact owner suppression must run before world static contact solving."
}

if ($runtimeText -notmatch "solveNativeWorldStaticContact" -or $runtimeText -notmatch "NativeContactEvidenceSnapshot") {
    throw "World soft contact must consume fresh native hknp evidence before falling back to static-world queries."
}

if ($runtimeText -notmatch "CandidateSource::NativeWorld" -or
    $runtimeText -notmatch "candidateSourcePriority" -or
    $runtimeText -notmatch "hasAuthoritativeWorldCandidate") {
    throw "Native hand-world evidence must be an authoritative candidate source, not an equal-priority query fallback."
}

if ($runtimeText -notmatch "projectTrackedMagnetCorrection" -or $runtimeText -notmatch "handInput\.rawHandWorld") {
    throw "Soft contact must project current tracked-collider penetration from rawHandWorld instead of smoothing toward a retained surface target."
}

if ($runtimeText -notmatch "triggerHaptic") {
    throw "World soft contact haptics must route through the existing VRControllers haptic API."
}

if ($mathText -notmatch "updateHapticEdge") {
    throw "World soft contact haptics must keep edge-only policy in testable math."
}

if ($worldPolicyText -notmatch "acceptsWorldSurfaceLayer") {
    throw "World soft contact must keep a testable static-world layer policy."
}

if ($runtimeText -notmatch "applyExternalHandWorldTransform") {
    throw "SoftContactRuntime must publish visual hand authority through the existing FRIK external transform API."
}

if ($runtimeText -notmatch "clearHandForStrongerOwner" -or
    $runtimeText -notmatch "suppressesGeneratedHandContactEvidence") {
    throw "SoftContactRuntime must expose explicit stronger-owner clearing and use the shared generated-contact ownership policy."
}

if ($runtimeText -notmatch "external transform apply failed; clearing visual contact state" -or
    $runtimeText -notmatch "clearHand\(isLeft\);[\s\S]{0,160}handState\.state = ContactState::Inactive") {
    throw "SoftContactRuntime must clear stale state when FRIK rejects an external transform apply."
}

if ($physicsText -notmatch "_softContactRuntime\.update") {
    throw "PhysicsInteraction update loop must invoke SoftContactRuntime."
}

$grabUpdateIndex = $physicsText.IndexOf("updateGrabInput(frame);")
$softContactUpdateIndex = if ($grabUpdateIndex -ge 0) { $physicsText.IndexOf("_softContactRuntime.update", $grabUpdateIndex) } else { -1 }
if ($grabUpdateIndex -lt 0 -or $softContactUpdateIndex -lt 0 -or $softContactUpdateIndex -lt $grabUpdateIndex) {
    throw "Soft contact must run after normal grab input so it sees final same-frame grab ownership."
}

if ($physicsText -notmatch 'clearHandForStrongerOwner\(isLeft,\s*"normal-grab-capture"\)' -or
    $physicsText -notmatch 'clearHandForStrongerOwner\(isLeft,\s*"dynamic-pull-grab-capture"\)' -or
    $physicsText -notmatch 'clearHandForStrongerOwner\(isLeft,\s*"held-object"\)') {
    throw "PhysicsInteraction must clear soft contact before grab capture and while held-object ownership is active."
}

if ($physicsText -notmatch "_softContactRuntime\.reset") {
    throw "PhysicsInteraction lifecycle paths must reset SoftContactRuntime."
}

exit 0
