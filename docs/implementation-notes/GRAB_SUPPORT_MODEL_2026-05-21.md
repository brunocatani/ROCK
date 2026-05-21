# Grab Support Model - 2026-05-21

Project: ROCK

Source used: current local ROCK source on `feature/ghidra-grab-motor-mapping`.

Confidence: medium-high for source behavior; runtime feel still needs in-game validation.

Verification method: source mapping, policy tests, source-boundary tests, local build/test commands.

## Problem

The existing contact patch tells ROCK which rendered surface point the hand touched, but a one-point patch, a corner, or a long thin same-face patch does not define roll support. Treating sample count or a narrow triangle patch as authority can make small props spin around one point and make long handles feel weak or fake.

## Implementation Direction

The new support model is a second stage after mesh/contact selection. It keeps the chosen palm or pinch point as the anchor, then consumes extra support samples from hand-meaningful directions:

- across-palm side probes;
- finger-forward/back probes;
- thumb/index probes for pinch grabs;
- existing contact patch samples.

The model classifies support as:

- `SinglePoint`;
- `SameSurface`;
- `OpposedPinch`;
- `LongHandleAxis`;
- `PalmWrap`.

Only high-confidence opposed/wrap support can author a new solver pivot. Same-face and single-point evidence can improve metadata/debugging but cannot move pivot B.

## Safety Rules

- Support hits must resolve to the held body or the selected multipart visual-mesh family.
- A support pivot must stay within a bounded shift from the selected anchor.
- Contact-patch line evidence alone cannot author a centerline pivot.
- Long-handle metadata can report a stable object long axis without moving pivot B unless opposed side support exists.
- Live held motor force is not reduced from weak support quality.
- Held-time refresh must not silently move frozen pivot B.

## Expected Runtime Effect

Small pinchable objects can freeze pivot B at an opposed thumb/index midpoint instead of one tiny triangle. Long thin objects can use object long-axis metadata and, when side probes find opposed sides, move pivot B toward the handle centerline. Bad corners remain usable translation points without pretending to provide roll authority.
