# Weapon Domain

This module owns generated weapon collision, two-handed support, weapon semantic classification, interaction routing, and weapon visual authority. It keeps firearm/melee interaction policy together while relying on native and hand modules for low-level body operations and hand-frame inputs.

## Generated Collision Lifecycle

Equipped weapon collision is now creation-only. Each update reads the current equipped weapon identity, hashes the current first-person visible weapon roots, scans those roots for extractable TriShapes, creates generated Havok bodies from the visible geometry, publishes their metadata, and then enables their collision filters.

The lifecycle does not queue native weapon attach tasks, install equipped-instance witness hooks, wait through visual refresh stages, or keep pending/settle/probe rebuild state. If the current visual root or buildable source geometry is absent, ROCK tears down stale generated weapon bodies and clears the cached source summary. The next valid visible weapon tree creates a fresh body set directly.

OMOD/index data is used only as part of the equipped weapon generation key. It is not a geometry source and it does not authorize a native refresh path. Semantic part classification remains runtime telemetry for interaction roles and debug evidence; it does not gate whether visible, buildable weapon geometry receives collision.
