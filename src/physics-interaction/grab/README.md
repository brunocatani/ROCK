# Grab Domain

This module owns object grab preparation, constraint math, contact patch selection, finger pose/runtime policies, held-object damping, and player-space compensation. The split keeps HIGGS-style grab behavior separate from native memory access and from the top-level frame loop.

## Shared Held Objects

Two-hand grabbing of loose objects uses HIGGS' dynamic-grab pattern rather than a handoff or the equipped-weapon visual support-grip solver. Single-hand loose grabs stay on FO4VR's native mouse-spring action. When the peer hand close-grabs that same held reference, the joining hand creates a HIGGS-style angular+linear grab constraint and the original hand is promoted from native mouse spring to its own constraint on the same object body. That gives the object two physical hand anchors, so separated hand motion can pivot and rotate the body instead of only translating it.

Delayed second-hand joins do not rely only on the ordinary swept selection cache. On a grip press, ROCK can synthesize a close selection from the peer hand's current held body set, but only for that exact peer-held reference and only inside close reach. Far pulls and unrelated close selections remain exclusive.

Shared loose-object grabs keep ownership at the hand-owner level instead of counting repeated claims. A pull-to-grab promotion by the same hand is still one hand lease, while a true second-hand grab adds a separate peer lease. Object-side body flags, inertia, and active-grab lifecycle state are restored only when the last hand releases. Joining a peer-held object skips the vanilla VR grab drop call so the existing ROCK-held peer action is not disrupted.

## Far Pull Catch Handoff

HIGGS keeps the grab request alive while a pulled object is moving toward the palm, then biases the pulled grab so the visible hand does not wait for a mathematically exact contact point before it closes. ROCK follows that handoff shape while keeping FO4VR's native mouse-spring held-object path: the original grip must stay held, release cancels, and arrival arms a bounded close-commit retry instead of requiring a second button edge. While that catch intent is active, normal selection refresh is frozen to the arrived ref/body; if the ref/body no longer matches, the top-level interaction loop cancels the intent and releases the object claim instead of letting the old pull owner leak.

Near/far convergence now has two promotion paths. Exact touch still promotes immediately, but a stable object inside the hand pocket can also promote after the converge timeout. Stability is frame-based and rejects fast separating pocket drift unless there is fresh held-body collision evidence. During that window ROCK may publish a pre-touch acquisition finger pose and visual-only HIGGS reverse hand target inside a bounded acquisition visual envelope; final mesh local-transform corrections still wait until `TouchHeld` so the object-hand relation is not recaptured while the native spring is settling.
