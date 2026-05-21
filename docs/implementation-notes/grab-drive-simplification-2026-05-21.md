# Grab Drive Simplification - 2026-05-21

Project: ROCK

Source used: current ROCK source, current ROCK runtime logs, approved local HIGGS comparison from `F:\fo4dev\skirymvr_mods\source_codes\higgs`.

Confidence: high for source behavior, runtime validation still requires in-game testing.

## Problem

Runtime logs showed normal loose-object grabs with full linear force but weak angular force, for example `linForce=2000` and `angForce=160`. Long objects and tiny objects then corrected rotation too slowly, and small/corner contact patches could also move too much authority into a weak one-point pivot.

## Decision

- Live held-object motors now use one mass-capped force budget for both linear and angular motor force.
- Body-set classification no longer scales live motor force. It only narrows velocity and mass scope where needed.
- Removed the config keys that exposed the old angular-force division and extra angular startup fade from the packaged INI.
- Contact patches no longer replace the frozen BODY-local pivot by themselves. They remain validation, pose, support, telemetry, and release evidence.
- Weak mesh-start pivots can yield to a palm-pocket mesh point when that gives a better seated relation.

## Rationale

HIGGS uses mesh/contact logic mostly to decide where the grab point is, not to weaken the normal loose-object motor. ROCK had accumulated extra authority layers that made the result worse than the simpler model. For FO4VR/hknp, the old `linear / 12.5` angular budget was visibly too weak in logs and made rotational error settle too slowly.

The pivot change avoids letting tiny patches, corners, or origin-adjacent mesh hits become the sole frozen pivot authority. The palm-pocket path remains the preferred close-grab seat because it has been the most stable in live testing.

## Validation Plan

- Policy/source-boundary tests must pass.
- `custom-fast` Release build must pass and auto-deploy.
- In-game validation should check broom/long-handle objects, cigarette/small rods, balls, pocket watches, and corner grabs near object origins.
