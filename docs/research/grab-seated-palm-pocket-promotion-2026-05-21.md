# Grab Seated Palm-Pocket Promotion - 2026-05-21

Project: ROCK

Source used: current ROCK source and local policy tests. No web, Ghidra, FO4 Mods MCP, or production INI changes.

Confidence: medium-high before live validation. The code paths passed local policy/source tests and the `custom-fast` Release build, but the exact feel still needs in-game testing because this changes held-object settling behavior.

Affected systems:

- `src/physics-interaction/hand/HandGrab.cpp`
- `src/physics-interaction/grab/GrabMotionController.h`
- close/far mesh-start grab convergence, seated pivot reacquire, contact-patch evidence, finger pose targets

Finding:

Weak mesh-start grabs already had a seated reacquire path, but that path mostly replaced the active point with one closest mesh point near the palm pocket. Palm-pocket captures were better because they carried richer seat/support evidence. The missing invariant was: when a weak mesh-start grab reaches the palm pocket, the frame should promote the whole seated support model, not just swap one point.

Implementation:

- Added `evaluateSeatedPalmPocketPromotion` as a pure policy.
- Promotion is limited to weak mesh starts that have reached touch range or timed out inside the pocket.
- If local pivot delta is small, ROCK promotes directly to palm-pocket mesh authority.
- If local delta is medium, ROCK advances the pivot by a bounded blend and keeps `requiresSettledVisualHandRelation` true until the relation completes.
- If local delta is large, or contact softening is active without touch-range arrival, ROCK does not move the pivot and only enriches support evidence when better samples or finger groups are available.
- Runtime promotion now rebuilds bounded palm-plane mesh support samples through the same nine-point probe policy used by contact patches.
- Completed promotion rewrites the active frame support samples, pivot B, grip point, grip normal, finger pose targets, and visual-hand relation state together.

Implication:

This should make mesh-start grabs behave more like palm-pocket grabs after the object actually reaches the hand, especially for weak one-point mesh starts. Large pivot jumps remain blocked so the fix does not reintroduce visual-hand offset snaps or sudden object pops.
