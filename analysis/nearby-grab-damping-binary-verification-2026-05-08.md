# Nearby Grab Damping Binary Verification - 2026-05-08

ROCK enables nearby damping through hknp motion-properties entries instead of direct velocity writes because FO4VR already routes damping through `hknpMotionProperties` and body motion-property IDs. The alternatives were repeated per-frame velocity scaling or editing shared preset entries in place. Per-frame velocity writes fight the solver and shared-entry edits affect unrelated bodies, so the production path copies a body's current motion-properties record, raises only the verified damping fields, adds/reuses that record in the world's motion-properties library, assigns the returned ID to the nearby body, then restores the original ID when all damping leases release.

## Verified Offsets And Calls

- `hknpWorld::setBodyMotionProperties` at VR `0x14153B2F0` reads `world+0x20` body array, body stride `0x90`, body `+0x68` motion index, `world+0xE0` motion array, motion stride `0x80`, and writes `hknpMotion +0x38` when the motion-properties ID changes. It then notifies the world body-change path.
- `hknpMotionPropertiesLibrary::addEntry` at VR `0x141767A70` uses the library entry array at `library+0x28`, count/free-list state at `+0x30/+0x38`, copies exactly `0x40` bytes into the selected entry, and returns the new `uint16_t` ID through the out parameter.
- `hknpMotionProperties` reflection metadata at `0x1438799D0` has stride `0x28` member descriptors. It names `linearDamping` at record offset `+0x18` and `angularDamping` at `+0x1C`; the full reflected record size is `0x40`.
- `hknpSolverIntegrator::subIntegrateBatch` at VR `0x141A1BFF0` and `hknpSolverIntegrator_subIntegrateLastStepBatch` at VR `0x141A1E480` load the motion's properties ID, compute `propertiesBase + id * 0x40`, read `+0x18/+0x1C`, multiply each by step timing, clamp to `<= 1.0`, and apply `velocity *= 1 - dampingStep`.
- Native FO4VR function `0x14103E910` copies the current record from `world+0x5D0 -> library+0x28`, writes its two float parameters into record `+0x18/+0x1C` when non-negative, calls `hknpMotionPropertiesLibrary::addEntry`, and assigns the returned ID to the motion. This independently confirms the field offsets and library workflow.

## ROCK Runtime Policy

- Never edit shared preset entries in place.
- Raise damping with `max(original, requested)` so nearby damping cannot make an already-damped object looser.
- Cache custom records per world and full `0x40` record bytes so repeated grabs reuse existing property IDs instead of allocating a new entry every time.
- Track active leases by `(world, motionId)`. A restore only writes the original ID when the current motion still points at ROCK's damped ID; if another system changed it, ROCK leaves the newer state alone and logs the skip.

## 2026-05-08 Code Review Follow-Up

- Native mutation calls (`hknpMotionPropertiesLibrary::addEntry` and `hknpWorld::setBodyMotionProperties`) must run outside ROCK's damping lease mutex. The lease registry now reserves a `(world, motionId)` write under lock, releases the lock for native add/set, then commits or aborts the registry state and wakes waiters.
- Final restore does not depend only on the originally captured representative body. If that body no longer maps to the leased motion, ROCK scans the readable hknp body table for another body with the same motion ID before deciding whether the motion is gone.
- Failed final restore keeps the ownerless lease in the global table for retry. `tickNearbyGrabDamping` retries pending final restores whenever a world is available, so transient body churn does not permanently strand a damped motion-property ID.
- Nearby damping config values are hknp per-second coefficients. The old normalized `[0,1]` sanitizer remains only for pure velocity attenuation helpers; runtime hknp writes use `sanitizeHknpDampingCoefficient`.
