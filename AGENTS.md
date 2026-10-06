# AGENTS.md — ROCK

Project location: `F:\fo4dev\PROJECT_ROCK_V2\main_projects\ROCK`.
Main root and project routing: [AGENTS.md](F:/fo4dev/PROJECT_ROCK_V2/AGENTS.md).
Workspace-relative paths, including `Docs/` and `libraries_and_tools/`,
resolve from `F:\fo4dev\PROJECT_ROCK_V2`.

These instructions apply to `main_projects/ROCK/` in addition to the workspace
root contract.

## Active Checkout

This is the default ROCK checkout, with the modular feature-provider APIs and
FRIK `0.79` / API `2.3` integration. Use this checkout for ROCK work and verify
its current branch before editing; follow the workspace Git policy.

ROCK deploys to `D:\FO4\mods\ROCK`, with DLL/PDB under `F4SE\Plugins`.

As of 2026-09-27, the user deleted the Parallel 1 checkout and its development
branch. Continue work previously assigned to ROCK Parallel in this regular ROCK
checkout, on its currently checked-out branch (`main` at this update), deploying
only to `D:\FO4\mods\ROCK`. Do not recreate the deleted checkout or branch.
ROCK V2 remains a separate project; do not synchronize it with regular ROCK
without explicit instruction.

Consumer changes still require the user's explicit permission, including
consumer source, bindings, SDK copies, FRIK, addons, configuration and deployed
files. Moving this work to regular ROCK does not expand that permission.

## Ownership And Architecture

ROCK is the FO4VR F4SE plugin for physics-based hand interaction, object
grabbing, hand collision, equipped-weapon interaction, and the provider API.

- ROCK is the primary data provider. Its major features expose independently
  versioned interfaces, currently V1. These versions are separate from FRIK API 2.3.
- Released provider contracts must remain compatible. Follow the workspace
  [Released ROCK provider API compatibility rule](../../AGENTS.md#released-rock-provider-api-compatibility-2026-09-23)
  for ROCK, its SDK, and consumers. Compatible additions use a new minor (1.0 to
  1.1); substantial redesigns or any incompatible change require a new major (V1
  to V2). Preserve existing table prefixes, record layouts, and behavior in a
  minor update, and negotiate minimum minor/table size for new functionality.
  Retain released majors; newer compatible minors may serve older requests
  without requiring consumer changes.
- Changing a published function without a new interface version is permitted
  only for a user-approved emergency fix to a named function and affected
  versions, preserving the ABI and intended
  contract. Private maintenance must preserve all published behavior. Shared
  implementation changes must not alter old contracts, and compatibility checks
  must cover unchanged old consumer contracts alongside the new version.
- ROCK links `libraries_and_tools/CommonLibF4VR` directly and does not consume
  F4VR-CommonFramework.
- ROCK-owned logging, runtime helpers, menu tracking, resource helpers, and
  controller input live under `src/rock_support/`.
- The exact OpenVR ABI used for raw controller input and haptics is vendored under
  `third_party/openvr/`; do not create a separate workspace dependency for it.
- Do not move PAPER, PAPER_Toolkit, SCISSORS, or addon-owned behavior into ROCK
  merely for convenience.
- Use RR's upstream `dependancies_and_dependant_mods/Fallout-4-VR-Body`
  for skeleton/provider integration reference. hFRIK is retired and remains
  untouched pending the user's manual move; do not build or deploy it.

Historical HIGGS material is not authority. ROCK's current source and verified
FO4VR behavior are the state-of-the-art reference.

## Compatibility References

When adding ROCK compatibility, consult the mods under
`F:\fo4dev\PROJECT_ROCK_V2\systems_that _need _rps_compat\`:

- `F4VR-ImmersiveFlashlight`
- `Fo4VR-True-Scopes`
- `Heisenberg-Physical-Interactions`
- `VirtualHolsters`

Holsters in that folder is not up to date. Cylon is providing information as
it comes; treat that information and the Holsters copy as reference only,
not established truth.

## Git Authority

- The workspace development repository's sole configured remote is `origin` at
  `https://github.com/brunocatani/ROCK.git`. This public repository is the active
  development source.
- A private legacy backup repository also exists (ask the user if it is ever
  needed). It is not a configured remote and receives no further development or pushes
  unless the user explicitly requests work on that legacy repository.
- The public repository received the private repository's branch and tag
  history on 2026-09-13. Future publication is a normal push to public `origin`
  under the workspace Git policy, not a private-to-public synchronization.
- Use the currently checked-out branch. Reverify live status and remote state
  before every Git operation; documented branch snapshots can become stale.

## Production Paths

- Active config:
  `%USERPROFILE%\Documents\My Games\Fallout4VR\Mods_Config\ROCK\ROCK.ini`
  (resolve the Documents known folder; never hard-code a username)
- Optional developer overrides: `ROCK_Developer.ini` in the same folder; created only by non-default developer changes.
- Auto-deploy target: `D:\FO4\mods\ROCK\F4SE\Plugins`
- Deployed artifacts: `ROCK.dll` and `ROCK.pdb`

Verify deployed DLL VersionInfo and timestamp rather than trusting a recorded
version snapshot. Production config changes require the permission and in-place
editing rules from the root contract.

## Required Build

```powershell
cmake --preset custom-fast
cmake --build build-fast --config Release --target ROCK -- /m:1 /p:CL_MPCount=2
```

Expected local outputs:

```text
build-fast/Release/ROCK.dll
build-fast/Release/ROCK.pdb
```

Do not automatically delete active build directories; preserve incremental and
PCH state. `custom-release` and `build-release` remain user-gated release flow.

## Affected-Test Workflow

The test selector is available **on demand** — run it only when the user
explicitly requests a test run. Do NOT run it automatically after commits.

```powershell
pwsh -NoProfile -File tools/Invoke-RockTests.ps1
```

- With a dirty tree it selects from staged, unstaged, and untracked ROCK files.
- With a clean tree it selects from the latest commit.
- It configures `custom-tests` as needed, builds selected tests in `Release` with
  the root compiler cap, runs CTest at most at `-j 4`, and fails closed to broader
  coverage when mapping is unsafe.
- Do not manually narrow the selection or bypass automatic full-suite escalation.

Use `-BaseRef <ref>` when validating a committed range plus worktree changes,
`-PlanOnly` to inspect selection, and `-All` only when requested or when a
cross-repository/dependency change cannot be inferred from ROCK's Git diff.

Manual full-suite fallback/diagnostics:

```powershell
cmake --preset custom-tests
cmake --build build-tests --config Release --target ROCKPolicyTestBinaries -- /m:1 /p:CL_MPCount=2
ctest --test-dir build-tests -C Release --output-on-failure -j 4
```

## Test Policy (owner, 2026-10-06)

Tests exist to catch real breakage, not to restate decisions. The 2026-10-06
triage deleted 51 of 146 test files that only mirrored policy if-chains, pinned
tuning values or log strings, or modeled engine types with stand-ins; they had
to be edited with nearly every intended behavior change and never caught a bug.

Add or keep a test only when it guards one of these:

- a released provider/API/ABI or configuration contract;
- numeric or geometric math on real engine types or real captured/measured data;
- concurrency, lifetime, ownership, or memory safety, including fail-closed
  rejection before engine writes;
- equivalence between an optimized path and its reference path;
- a specific regression reproduced from captured runtime input, or an input
  state machine with a real-world hazard (stale samples, hitches, lost or late
  native events).

Do not add tests that:

- re-list a policy or decision function's outcomes row by row;
- pin tuning constants, defaults, thresholds, reason strings, or log text;
- use toy stand-ins for engine types (a float "rotation", scalar transforms) or
  inject the math under test through test-side lambdas;
- exist only to keep a production symbol referenced: delete both instead;
- guard a finished migration, or refuse/invalidate on a hash mismatch.

Policy tests link CommonLib's real `NiPoint.cpp`/`NiMatrix3.cpp`; never
re-implement engine math in test stubs. When a behavior change breaks a test,
first decide whether the test meets the rules above; if not, delete it rather
than rewriting it to match the new decision.

## Runtime Debugging

When a runtime symptom survives one attempted fix, the next change must add a
rate-limited trace that distinguishes candidate causes in one game session.
Correct contaminated inputs or broken invariants; do not stack estimation layers
or present timers/caps as root-cause fixes.
