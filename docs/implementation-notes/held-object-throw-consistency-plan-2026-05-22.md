# Held Object Throw Consistency Plan

Date: 2026-05-22

Project: ROCK

Branch at planning time: `feature/ghidra-grab-motor-mapping`

Source used: local ROCK source inspection only.

Confidence: medium-high. The plan is based on current source behavior, but final tuning still needs in-game validation.

Verification method: static inspection of held-object release, throw velocity composition, player-space velocity compensation, config, and policy-test structure.

Affected systems: held-object grab release, release velocity history, player-space compensation, throw config, policy tests.

## Summary

Improve held-object throws by replacing the current peak-sample release velocity with a deterministic release-intent solver.

The result should make throws repeatable, targetable, and physically plausible: good holds throw from tracked hand intent, bad/lagging/colliding holds fall back toward the object's actual velocity instead of inventing force.

Use local ROCK source only. No web, Ghidra, HIGGS, or production INI edits for this pass.

## Key Changes

- Replace release velocity selection in `HandGrab.cpp` / `GrabHeldObject.h`.
- Stop using highest-magnitude history sample for hand/object/angular release velocity.
- Record only valid release samples: finite values, `0 < dt <= 0.05s`, no player-space warp.
- Clear release history on warp or hard hitch so stale tracking cannot produce a throw.
- Solve hand intent from a short recency-weighted window, default `0.085s`.
- Reject older samples moving against the newest release direction so backswing spikes do not dominate.

## Release Composition

- Primary source is filtered controller-local hand velocity.
- Object velocity becomes a fallback/quality blend, not an additive boost.
- Compute hold quality from last pivot deviation, contact softening, and held-drive authority.
- Good hold: mostly hand intent.
- Poor hold, collision, high lag, weak/native scan uncertainty: blend toward actual object velocity.
- Keep player-space velocity additive after local throw solving so moving with the player does not consume the throw cap.

## Force Boundaries

- Interpret `fGrabThrowMaxVelocityHavok` as the max relative throw component, not a cap that erases player motion.
- Keep existing angular authority/long-object angular caps.
- Limit linear tangential wrist-swing assist so wrist spin cannot launch an object sideways when the hand's linear throw intent is weak.
- Lower packaged default `fGrabThrowObjectVelocityBlend` to `0.20` unless local testing shows `0.35` is still needed.

## Config And Interfaces

- Add `fGrabThrowSampleWindowSeconds = 0.085` to packaged `ROCK.ini`, `RockConfig.h`, and `RockConfig.cpp`, clamped to `[0.02, 0.20]`.
- Keep existing throw keys.
- Update packaged comments to describe filtered release intent and relative velocity cap semantics.
- Add pure policy result telemetry fields internally: sample count, effective object blend, hold quality, filtered hand/object velocity, relative throw velocity.

## Tests

- Add `GrabReleaseVelocityPolicyTests.cpp` and register it in `ROCKPolicyTestBinaries`.
- Cover steady forward hand samples producing stable forward release velocity.
- Cover rejection of an older opposite-direction peak.
- Cover hitch/warp/non-finite samples being ignored or clearing history.
- Cover object velocity fallback when controller-derived hand velocity is unavailable.
- Cover poor hold quality blending toward object velocity.
- Cover good hold using mostly controller intent.
- Cover tangential wrist assist capped relative to linear hand intent.
- Cover relative throw cap applying before player velocity is added.
- Cover angular release still respecting authority and long-object caps.
- Add a source-boundary test rejecting release use of `maxMagnitudeVelocity(...)` for hand/object/angular throw solving.

## Validation

Run these commands after implementation:

```bat
cd ROCK && cmake --preset custom-tests && cmake --build build-tests --config Release --target ROCKPolicyTestBinaries -- /m
cd ROCK && ctest --test-dir build-tests -C Release --output-on-failure -j %NUMBER_OF_PROCESSORS%
cd ROCK && cmake --preset custom-fast && cmake --build build-fast --config Release --target ROCK -- /m
```

`custom-fast` is expected to auto-deploy `ROCK.dll` and `ROCK.pdb`.

Commit only if the throw change is complete, tests/build pass, and unrelated dirty files are not included. Suggested commit:

```text
fix/grab: stabilize held object throw velocity
```

## Assumptions

- Desired feel is consistent realism, not hidden target auto-aim.
- Minimal config is preferred: one new sample-window key plus clearer semantics for existing throw keys.
- Runtime in-game validation is still recommended after deployment, but policy tests and Release build are the required completion gate.

