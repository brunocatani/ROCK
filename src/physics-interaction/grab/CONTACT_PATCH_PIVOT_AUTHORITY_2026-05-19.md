# Contact Patch Pivot Authority - 2026-05-19

Project/repo affected: ROCK

Source used: current ROCK source on `feature/ghidra-grab-motor-mapping`; local HIGGS source was used only to classify the broad concept as mesh/triangle-assisted grab-point authority, not as implementation authority.

Confidence level: medium-high for source behavior, runtime validation still requires in-game handling checks.

Verification method: local source inspection, focused pure policy tests, source-boundary tests, full ROCK test suite, and mandatory `custom-fast` build/deploy.

Affected systems:

- `src/physics-interaction/grab/GrabContact.h`
- `src/physics-interaction/hand/HandGrab.cpp`
- grab pivot-B capture and contact-patch evidence telemetry

Implementation rationale:

ROCK already had mesh/contact-patch evidence and runtime plumbing for a contact-patch pivot candidate, but the resolver initially kept patches evidence-only. The implemented path allows a mesh-snapped contact patch to replace the object-side BODY-local pivot only when it is owner/body coherent, deduped to broad surface support, selection coherent, close to the palm pocket, within the bounded selected-authority delta, and materially better than the current mesh pivot by both pocket distance and score.

The accepted patch authority is position-only. It moves the frozen pivot-B point, but it does not promote the patch normal into object orientation authority. The runtime preserves the canonical surface normal for stored grip evidence and marks contact-patch authority as `SurfacePatchPositionPivot` in telemetry.

Implication for future implementation:

- Do not broaden this by setting `usePatchPivot = true` directly.
- Do not let patch normals become orientation authority without a separate policy, tests, and runtime validation.
- Keep mesh snap and owner resolution mandatory for contact-patch pivot authority.
- Keep dedupe, patch quality, unique sample/triangle counts, and duplicate rejection metadata visible in capture telemetry.
- In-game validation should focus on thin rods, small props, handles, trays, pistols, and edge/corner grabs where the selected mesh pivot and palm contact patch disagree.
