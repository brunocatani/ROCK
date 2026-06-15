# Grab Release API Review Fix Plan

Date: 2026-06-15

Project: ROCK

Context:

- Current branch: `feature/ghidra-grab-motor-mapping`.
- Current API version must remain `ROCK_PROVIDER_API_VERSION == 1`.
- Latest committed release/drop API commit before fixes: `4609e45 feature/api: add queued provider release commands`.
- Existing unrelated untracked file: `regular-grab-two-hand-object-steering-wheel-research.md`.

User decisions:

- Do not bump API version.
- Preserve v1 and make the current v1 interaction command API correct.
- Provider force grab does not need a fresh press-release guard. If the user was already holding grab when API force grab succeeds, releasing that held grab input may release the object.
- Force release should default to a gentle drop when no trusted velocity/force is supplied.
- Force release should expose optional caller-controlled release force/velocity through the API.
- External thrown-drop / release force values are trusted. Do not clamp them.

Review findings to fix:

- ABI table ordering bug: `requestForceReleaseV1` and `requestThrownDropV1` were inserted before `getInteractionCommandResultV1`, moving an existing v1 function pointer. Fix by restoring `getInteractionCommandResultV1` immediately after `requestForceGrabV1` and appending new functions after it.
- Force release currently uses normal `PhysicalDrop` release velocity behavior. It should be gentle by default, not throw from stale controller history.
- Force release needs optional trusted velocity/force inputs. Use existing held-body release velocity path for arbitrary held world objects; `BethesdaPhysicsBody::applyLinearImpulse/applyPointImpulse` is for ROCK-created bodies and is not the correct general path for arbitrary held object bodies.
- Optional force/velocity request should avoid hidden compatibility paths and should be explicit in flags.
- Documentation feature-bit list is stale: `SDK/ROCK/docs/PublicApi.md` lists `ForceGrabCommand` but not `ForceReleaseCommand` / `ThrownDropCommand`.
- Tests are source-boundary only; update them to verify ABI order and the new optional force-release surface. Runtime smoke remains unrun.

Planned API/header fix:

- Keep `ROCK_PROVIDER_API_VERSION` at `1`.
- Keep existing `RockProviderForceReleaseRequestV1` and grow it only through reserved space if possible.
- Implemented `RockProviderForceReleaseFlagV1` values:
  - `ImmediateCollisionRestore = 1u << 0`
  - `RequireMatchingTarget = 1u << 1`
  - `UseVelocityHavok = 1u << 2`
- For `RockProviderForceReleaseRequestV1`, grow the current v1 struct to add:
  - `float linearVelocityHavok[3]`
  - `float angularVelocityRadiansPerSecond[3]`
- Application-point force derivation was intentionally not added. Direct linear/angular Havok velocity satisfies the trusted optional velocity requirement without introducing unverified COM/application-point runtime assumptions.
- Preserve `RockProviderThrownDropRequestV1` as explicit trusted velocity drop command.
- Restore function pointer order:
  - `requestForceGrabV1`
  - `getInteractionCommandResultV1`
  - `requestForceReleaseV1`
  - `requestThrownDropV1`

Planned runtime behavior:

- In `PhysicsInteraction::processProviderInteractionCommands`:
  - Force release with no velocity flags uses `GrabReleaseDisposition::PhysicalDrop` for normal lifecycle restore/activation, but disables captured controller release velocity for a gentle default drop.
  - Force release with `UseVelocityHavok` releases through the same cleanup path, then applies the trusted supplied velocity through `Hand::applyReleaseVelocitySnapshot`.
  - Thrown drop remains `PhysicalDrop`; without trusted velocity it captures current held release motion, and with `UseVelocityHavok` it applies trusted supplied velocity after release.
  - Keep shared two-hand thrown-drop rejection unless explicitly changed.
  - Keep force-grab release behavior as decided: no fresh release-arm guard.

Validation already run before this fix plan:

- `pwsh -NoProfile -ExecutionPolicy Bypass -File tests\PublicApiLaunchSourceTests.ps1`
- `cmake --build build-tests --config Release --target ROCKPolicyTestBinaries -- /m`
- `ctest --test-dir build-tests -C Release -L source-boundary --output-on-failure -j $env:NUMBER_OF_PROCESSORS`
- `cmake --build build-fast --config Release --target ROCK -- /m`

Validation completed after fixes:

- `pwsh -NoProfile -ExecutionPolicy Bypass -File tests\PublicApiLaunchSourceTests.ps1`
- Source/SDK provider header diff check.
- `cmake --build build-tests --config Release --target ROCKPolicyTestBinaries -- /m`
- `ctest --test-dir build-tests -C Release -L source-boundary --output-on-failure -j $env:NUMBER_OF_PROCESSORS`
- `cmake --build build-fast --config Release --target ROCK -- /m`
- Final `git status`, `git diff --check`, and intended diff review.

Open risk:

- No FO4VR runtime smoke test has been run for provider force grab/release/drop commands.
