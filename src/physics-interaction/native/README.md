# Native Boundary

This module owns ROCK's direct FO4VR Havok/Bethesda ABI calls, hard offsets, allocator use, and guarded body-slot access. It exists so hand, grab, weapon, object, debug, and core orchestration code can use named operations instead of re-reading native layouts or calling direct-index Havok writers from multiple places.
