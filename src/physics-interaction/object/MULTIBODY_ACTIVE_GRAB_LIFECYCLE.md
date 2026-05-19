# Multipart Active Grab Body Ownership

ROCK prepares active grabs by calling FO4VR's recursive `NiAVObject` physics wrappers on the selected reference root. That is the correct native boundary for multipart objects, but it means the committed held-body set must be at least as complete as the selected tree that native prep can touch. If body discovery only trusts each node's `bhkPhysicsSystem` body-id array, multipart refs such as loose weapons or Giddyup Buttercups can be partially prepared while only the selected seed body is activated, leased, damped, and restored.

The active-grab scan now has two selected-ref-only recovery rules:

- `allowOwnerNodeWorldFallbackScan` walks readable hknp body slots and admits bodies whose native collision back-pointer resolves to a node under the selected ref root. This repairs stale, missing, or unreadable per-node physics-system body arrays without crossing into other references.
- `allowSelectedObjectTreeActiveGrabLayers` lets active grab accept selected-tree supplemental layers that are not general dynamic-push targets. The first known case is `FO4_LAYER_PROPS`, which some multipart loose refs use for secondary collision bodies.

Lifecycle capture must also remember selected-tree bodies that recursive prep can touch even when they are rejected as held bodies. Rejected unsupported/noncollidable/actor-layer bodies are captured for restoration, and prepared scans compare their post-prep motion/filter state against the original record. On release, accepted loose dynamic bodies keep active collision as before, while rejected selected-tree bodies restore engine-owned filter or motion state instead of being left in a half-prepared state.
