# v1 Interaction Force Grab Implementation

Date: 2026-06-15

Project: ROCK

Source used:

- Current local ROCK source in `F:\fo4dev\PROJECT_ROCK_V2\ROCK`.
- Local API roadmap and HIGGS/ROCK API map under workspace docs.

Verification method: local source, source-boundary tests, policy test build, and local Release build. No web, Ghidra MCP, FO4 Mods MCP, or runtime logs were used.

Goal:

- Add a current-version ROCK provider force-grab API without bumping `ROCK_PROVIDER_API_VERSION`.
- Keep the API fully dynamic and ROCK-native.
- Execute public writes through a bounded command queue from ROCK-owned update points.
- Route successful force grabs through the existing `Hand::grabSelectedObject` path so grab settling, dynamic constraint ownership, finger pose, haptics, claim ownership, and cleanup remain the same as normal grabs.

Implemented scope:

- `ROCK_PROVIDER_API_VERSION` remains `1`.
- `InteractionCommands`, `InteractionCommandQueue`, `ForceGrabCommand`, `ForceReleaseCommand`, and `ThrownDropCommand` are real v1 capability/feature bits.
- `requestForceGrabV1` enqueues a bounded command and returns a command id.
- `requestForceReleaseV1` enqueues a bounded force-release command and returns a command id.
- `requestThrownDropV1` enqueues a bounded thrown-drop command and returns a command id.
- `getInteractionCommandResultV1` polls queued/completed command state.
- The v1 provider function table keeps the existing `requestForceGrabV1` then `getInteractionCommandResultV1` order; release/drop entry points append after those existing entries.
- Provider glue validates owner token, granted capability, struct size/version, hand, flags, target identity, velocity shape where applicable, and queue capacity.
- Runtime execution revalidates world/physics writes, generations, hand availability, target availability, object ownership, body scan, distance, held-object match, and release state.
- Successful execution creates a temporary loose-object selection and commits through `Hand::grabSelectedObject`.
- Successful force release and thrown drop execute through `Hand::releaseGrabbedObject`, release ROCK's object claim, dispatch normal release messages/events, and clear per-hand interaction intent/candidate state.
- Force release defaults to a gentle physical drop by using the normal release path while suppressing captured controller throw velocity.
- Force release and thrown drop can apply trusted caller-supplied Havok linear/angular velocity through the existing release velocity path; trusted values are finite-checked but not clamped.

Initial limitations:

- Explicit `Left` or `Right` hand only.
- Live loose object / loose weapon reference only.
- Near/object-at-hand force grab only. Far dynamic pull is intentionally deferred until it can reuse the existing pull path cleanly.
- Busy hands reject rather than dropping existing held objects.
- Force release and thrown drop require an explicit hand and only release ROCK-held objects.
- Thrown drop rejects shared two-hand held objects because the peer hand still owns the object.
- The current v1 force-release payload exposes direct linear/angular Havok velocity only. It intentionally does not derive angular force from an application point because that would add unverified runtime assumptions beyond the current request.

Validation run:

- `pwsh -NoProfile -ExecutionPolicy Bypass -File tests\PublicApiLaunchSourceTests.ps1`
- `cmake --preset custom-tests`
- `cmake --build build-tests --config Release --target ROCKPolicyTestBinaries -- /m`
- `ctest --test-dir build-tests -C Release -L source-boundary --output-on-failure -j $env:NUMBER_OF_PROCESSORS`
- `cmake --preset custom-fast`
- `cmake --build build-fast --config Release --target ROCK -- /m`

Known follow-up:

- Add far dynamic-pull command execution through existing `lockFarSelection`, `startDynamicPull`, `updateDynamicPull`, and pull-catch commit paths.
- Runtime smoke test in FO4VR with an external consumer once a caller is available.
