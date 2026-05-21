# Semantic Hand Frame Axis Unification - 2026-05-21

Project: ROCK

Source used: current local ROCK source and live-tested generated hand collider convention.

Confidence: high for the shared axis convention; runtime feel still needs in-game validation.

Verification method: source audit plus policy/source-boundary tests.

Affected systems:
- root-flattened finger pose snapshots;
- per-frame cached hand input and far pointing direction;
- object-grab finger curl planes and local transform correction;
- equipped two-handed weapon support grip;
- public ROCK palm position/forward API;
- packaged and production grab INI comments/left proxy/pointing offsets.

Decision:
- The semantic hand frame is derived from the root-flattened hand bone plus live finger bases.
- Local X is finger-forward.
- Local Y is palm depth/back.
- Local -Y is palm face.
- Local Z is across palm.
- Finger curl planes and support-grip palm targets must use this semantic frame, not raw hand local `-Z` or legacy authored handspace.
- Far pointing now interprets `fPointingVectorHandspace*` in the same semantic frame; the default is `-Y` palm-face with `bReverseFarGrabNormal=false`.
- The old authored handspace helpers in `HandFrame.h` are quarantined for legacy diagnostics and the two-handed weapon compatibility island only. Do not copy that convention into new runtime hand code.

Rationale:
- Dynamic palm-pocket grab already uses the generated grab proxy basis.
- The remaining raw `-Z` and legacy handspace paths could make finger curls, thumb/index correction, weapon support grip, and API consumers disagree with the generated palm/collider authority.
- Adjacent root-flattened finger joints are a better segment-forward source than assuming each finger bone's local `+X`.
- Caching the semantic frames in `HandBoneCache` keeps frame input and public API consumers on the already refreshed root-flattened snapshot instead of recapturing the skeleton in hot paths.
