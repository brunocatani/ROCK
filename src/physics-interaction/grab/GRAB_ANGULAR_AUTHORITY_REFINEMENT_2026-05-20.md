# Grab Angular Authority Refinement - 2026-05-20

Project: ROCK
Confidence: medium-high
Verification method: local ROCK source inspection, local HIGGS source comparison, policy tests, and local build/test validation.

## Problem

Long objects and compact objects such as balls or pocket watches could still rotate too freely around the grab point, especially when contact patch acquisition produced only position authority or an untrusted normal.

The previous contact-patch pivot authority change correctly prevented poor patch normals from becoming orientation authority, but the grab motor still treated the direct FO4VR angular velocity command as if the pivot had full angular support.

## HIGGS Reference Checked

Local source only:

- `F:\fo4dev\skirymvr_mods\source_codes\higgs\include\config.h`
- `F:\fo4dev\skirymvr_mods\source_codes\higgs\src\hand.cpp`
- `F:\fo4dev\skirymvr_mods\source_codes\higgs\src\constraint.cpp`

Relevant HIGGS behavior:

- dynamic grabs use finite linear/angular motor force, not infinite keyframe rotation;
- angular force follows linear force through `grabConstraintAngularToLinearForceRatio`;
- colliding objects lerp toward lower tau values;
- non-actor dynamic grabs keep normal angular tau fixed instead of raising angular
  tau from hand/object rotation error;
- grabbed object inverse inertia is clamped by max axis ratio and by `grabbedObjectMinInertia`;
- pulled objects use high angular damping to prevent uncontrolled spin.

## ROCK Decision

ROCK now keeps held-object angular correction solver-owned through the custom constraint's ragdoll motor atom. The earlier FO4VR-native hard-keyframe angular velocity writer was removed as a production authority path so it cannot compete with the constraint target or be selected by config.

The added logic therefore does not port HIGGS 1:1. It applies HIGGS's stability principles at ROCK's solver authority layer:

- linear and angular tau stay at the configured base values except while contact
  softening is active, preventing delayed snap-rotation gain changes;
- weak pivot/contact evidence is retained for release angular velocity safety,
  not live held motor force or damping;
- compact weak-contact objects receive extra angular softness;
- weak pivots reduce twist around the pivot-to-COM axis while preserving swing;
- release angular velocity uses the same weak-pivot twist limiter;
- grabbed inertia normalization now also applies HIGGS's minimum inertia clamp.

Long-handle classification remains useful as contact-shape telemetry and for
low-support force authority, but it no longer applies twist/swing axis damping by
itself. Long-object held feel should come from the selected grip point, mass,
finite angular force, and normalized inertia tensor rather than a separate
length-based motor mode.

## Runtime Implication

Objects with trusted normals and strong contact support retain normal release authority. Position-only pivots, untrusted normals, single-point/low-support contact patches, and small compact weak-contact objects get softer release angular velocity caps, while live held motor force remains fixed and mass-capped.

Review correction:

- Low-contact angular attenuation is only applied when the contact patch actually owns/supports the active pivot. Rejected/incidental patch samples are still recorded for diagnostics and reacquisition, but they no longer weaken trusted mesh or palm-pocket pivots.

No production INI was edited by this note. The packaged defaults in `data/config/ROCK.ini` document the new tunables.
