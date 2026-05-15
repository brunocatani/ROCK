# Dynamic Grab Activation Code Review - 2026-05-15

## Scope

This review covers ordinary dynamic loose-object grab activation, held-body
lifecycle, custom proxy/constraint authority, mass/force budgeting, and the
spring-era backup support functions under:

- current repo: `E:\fo4dev\PROJECT_ROCK_V2\ROCK`;
- backup reference: `E:\fo4dev\Backups\before motor change\PROJECT_ROCK_V2\ROCK`.

Explicit exclusion: missing generated weapon collider creation is still known
broken and was not changed in this pass. Loose non-equipped weapons are reviewed
only as native multipart loose objects participating in dynamic grab.

## Confirmed Current State

- Current branch was clean and matched origin before review.
- `git push` reported `Everything up-to-date`.
- `PhysicsRecursiveWrappers.cpp` is effectively unchanged from the spring-era
  backup. The recursive motion, collision, and body activation wrappers are
  still present.
- `ObjectPhysicsBodySet.cpp` is effectively unchanged from the spring-era
  backup. Seeded scans, same-ref filtering, unresolved-ref diagnostics, guarded
  native physics-system body enumeration, and accepted body-set collection are
  still present.
- Source-boundary tests passed:
  `ctest --test-dir build-tests -C Release -L source-boundary --output-on-failure -j $env:NUMBER_OF_PROCESSORS`
  ran 12/12 passing.
- Full local tests passed:
  `ctest --test-dir build-tests -C Release --output-on-failure -j $env:NUMBER_OF_PROCESSORS`
  ran 21/21 passing.
- Fast deploy build passed:
  `cmake --build build-fast --config Release --target ROCK -- /m`
  rebuilt `build-fast\Release\ROCK.dll` and copied the DLL/PDB to
  `D:\FO4\mods\ROCK\F4SE\Plugins`.

## Activation And Body-Set Findings

Current dynamic pull and close grab both seed object scanning from the selected
hit body:

- pull: `scanOptions.seedBodyId = _currentSelection.bodyId.value`;
- close grab: `scanOptions.seedBodyId = sel.bodyId.value`.

Both paths keep `requireSameResolvedRef = true`, allow unresolved bodies only
through the explicit unresolved-ref diagnostic path, and call the recursive
Fallout-owned active prep wrappers before choosing the primary body.

Accepted held bodies are now canonical for the custom authority path:

- `_heldBodyIds = preparedBodySet.acceptedBodyIds()`;
- primary fallback only applies if the accepted set is empty;
- inertia normalization receives the whole held set;
- held mass summary reads the whole held set with unique-motion dedupe;
- direct angular authority writes the whole held set with unique-motion dedupe;
- release velocity and release cleanup operate on the held set;
- per-frame held update wakes primary plus held set.

The backup support functions that previously made multipart loose objects work
are still present. The custom motor/proxy path is now consuming them more fully
than the mouse-spring path did.

## Lifecycle Findings

Active grab lifecycle capture and restore are present:

- state is captured before active prep;
- prepared bodies are marked after recursive dynamic/collision prep;
- failure restore is guarded so peer-held join failures do not restore the
  other hand's lifecycle;
- incomplete native scans route to recursive root restore;
- release restore only runs on final object release;
- body flag leases are acquired for the held set and released with
  `finalObjectRelease`, so a peer hand can keep a shared object alive.

No source bug was found in the current lifecycle restore path during this pass.

## Motor And Weight Findings

Current dynamic grab no longer uses native mouse spring production authority.
The active path is:

1. resolve live palm-anchor body;
2. convert generated palm collider frame to grab authority frame;
3. create hidden no-contact keyframed proxy;
4. create finite linear constraint against the dynamic held body;
5. disable the hkp/hknp ragdoll angular atom as authority;
6. apply direct hknp angular velocity per physics step using the native
   hard-keyframe angular velocity boundary;
7. cap angular speed by finite angular budget and long-object lever scale.

Mass and long-object handling are scoped to force/speed authority only:

- `readHeldBodyMassSummary(...)` aggregates unique held motions;
- `grab_motion_controller::solveMotorTargets(...)` mass-caps linear force;
- angular budget is derived from the finite linear budget;
- long-object lever scale reduces angular speed cap only;
- COM/MOTION are not used as grip pivot or target-frame authority.

## INI / Runtime Validation Finding

The packaged config has `bDebugDrawGrabAuthorityProxy = false`, but the active
production INI still had `bDebugDrawGrabAuthorityProxy=true` appended at the
end from prior visual debugging. That does not change grab physics, but it can
pollute in-game visual validation by drawing the hidden authority proxy.

Action taken outside the repo, in place:

- `C:\Users\SENECA\Documents\My Games\Fallout4VR\ROCK_Config\ROCK.ini`
  changed `bDebugDrawGrabAuthorityProxy=true` to
  `bDebugDrawGrabAuthorityProxy=false`.

Other active diagnostic settings remain intentionally user-tuned:

- `iLogLevel = 1`;
- `bDebugGrabFrameLogging = true`;
- `bDebugGrabTransformTelemetry = true`;
- `iDebugGrabTransformTelemetryLogIntervalFrames = 1`.

## Exclusions / Deferred Known Broken Area

Generated equipped weapon collider creation remains excluded. This review did
not change:

- `WeaponCollision.cpp`;
- `WeaponCollision.h`;
- generated weapon body creation policy;
- equipped weapon collider activation/publication.

Loose non-equipped weapons are still treated as dynamic held objects that may be
multipart, not as equipped weapon generated colliders.

## Current Gaps To Watch

- Runtime visual validation still needs to distinguish native multipart loose
  weapons from generated equipped weapon colliders when testing. The code path
  is separate, but screenshots/log notes can mix them if the object type is not
  named precisely.
- `_heldBodyIdsSnapshot` is capped at 64 bodies for atomic contact lookup while
  most direct held-body operations use the full vector. No current test object is
  known to exceed this, but very large multipart refs should be watched in logs.
- The old INI-configured pivot values still exist for legacy telemetry and
  non-dynamic helpers. Dynamic grab runtime pivot A resolves from the live palm
  anchor body and falls back to raw hand origin, not the old INI pivot.

## Result

No source-code activation bug was confirmed in this pass. The spring-era support
functions for whole-object scan, recursive activation, and cleanup are present.
The only issue fixed was the active production INI visualizer left enabled from
debugging.
