# Semantic Hand Frame Axis Unification - 2026-05-21

Project: ROCK

Source used: current local ROCK source and live-tested generated hand collider convention.

Confidence: high for the shared axis convention; runtime feel still needs in-game validation.

Verification method: source audit plus policy/source-boundary tests.

Affected systems:
- root-flattened finger pose snapshots;
- object-grab finger curl planes and local transform correction;
- equipped two-handed weapon support grip;
- public ROCK palm position/forward API;
- packaged and production grab INI comments/left proxy offset.

Decision:
- The semantic hand frame is derived from the root-flattened hand bone plus live finger bases.
- Local X is finger-forward.
- Local Y is palm depth/back.
- Local -Y is palm face.
- Local Z is across palm.
- Finger curl planes and support-grip palm targets must use this semantic frame, not raw hand local `-Z` or legacy authored handspace.

Rationale:
- Dynamic palm-pocket grab already uses the generated grab proxy basis.
- The remaining raw `-Z` and legacy handspace paths could make finger curls, thumb/index correction, weapon support grip, and API consumers disagree with the generated palm/collider authority.
- Adjacent root-flattened finger joints are a better segment-forward source than assuming each finger bone's local `+X`.
