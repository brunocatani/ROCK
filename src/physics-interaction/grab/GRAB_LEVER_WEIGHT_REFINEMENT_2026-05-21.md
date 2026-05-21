# Grab Lever And Weight Refinement - 2026-05-21

Project: ROCK
Confidence: medium-high
Sources used: local ROCK source, local HIGGS source at `F:\fo4dev\skirymvr_mods\source_codes\higgs`, policy tests, source-boundary tests, local build/test validation.

## Problem

Live testing showed the remaining held-object lever effect felt artificial: long objects could feel sluggish or step-driven, compact objects could spin too easily around the grab point, and non-palm-pocket close grabs could leave the visual hand using stale authority after the object settled.

## HIGGS Reference

Production HIGGS dynamic grabs keep normal held linear/angular motors fixed except while contact softening is active. Generic objects use a finite 2000 linear-force budget capped by Havok mass, while loose weapons get the higher 9000 weapon budget before the same mass cap. HIGGS near selection is a short swept sphere, and nearby clutter damping is a small local radius around the hand.

## ROCK Decision

ROCK now treats held object size/contact shape as evidence and release safety data, not as a live motor gain mode:

- normal held linear and angular tau stay at the configured base tau;
- position lag no longer raises tau or max force;
- generic heavy props no longer climb to loose-weapon force;
- weak, small, point-like, and long-handle support no longer reduces live angular motor force or damping;
- long-handle/weak support no longer blocks visual-hand publishing once the existing settled-relation gate is satisfied;
- close selection is clamped to HIGGS-scale FO4VR game-unit values;
- nearby grab damping keeps ROCK's broader production radius because live testing found it improved clutter stability;
- held Havok mass can slow player movement through a reversible temporary SpeedMult modifier.

## Implication

Long-object feel should now come from the selected grip point, object mass, the inertia tensor clamp, finite generic/weapon motor budgets, and Havok collision rather than from ROCK-specific length or weak-contact motor penalties. If long objects still feel wrong in live testing, the next likely cause is pivot placement or inertia/mass data, not adaptive held motor policy.
