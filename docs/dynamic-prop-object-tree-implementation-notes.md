# Dynamic Prop Object-Tree Implementation Notes

## 2026-04-27 Wrapper Verification

This pass moves active grab/pull preparation from single collision-object mutation to FO4VR's recursive NiAVObject physics commands. The reason is structural: complex props and dropped weapons often split collision across child nodes, so changing only the contacted collision object leaves part of the object tree static/keyframed and produces partial pickup behavior. HIGGS handles grabbed objects as a coherent body set; FO4VR gives ROCK a verified recursive command path that matches that system-level need without hand-rolling child conversion.

Ghidra confirmation, performed against the loaded FO4VR binary:

- `0x141DF95B0` accepts a NiAVObject root as its first parameter, stores command ID `5`, stores the motion preset at command offset `0x10`, stores activate at `0x38`, and dispatches through `0x141DFA2B0`.
- `0x141DF9940` accepts a NiAVObject root as its first parameter, stores command ID `9`, stores enable at command offset `0x10`, stores recursive at `0x38`, and dispatches through `0x141DFA2B0`.
- `0x141DFA2B0` applies the callback to the current node collision object and recurses through child nodes only when the command's recursive flag is set.
- `0x141E07300` operates on one `bhkNPCollisionObject` through its physics-system pointer and body index. It is not a subtree operation and is not used for active object-tree preparation.
- `0x141546EF0` activates a single hknp body ID after validating its body array entry.
- `0x141547320` activates a list of body IDs with additional body flag and material checks. ROCK uses single-body activation per accepted body for this pass because its body-set scanner already owns validation and dedupe.
- `0x141E08520` applies a linear impulse through a single `bhkNPCollisionObject`, resolving that collision object to its current body ID before forwarding to the hknp impulse helper.

Implementation consequence: active grab/pull calls recursive SetMotion(DYNAMIC) and EnableCollision(true) on the selected reference root before rescanning the body set. Passive bumping never calls these recursive conversion wrappers; it only wakes and pushes bodies already classified as dynamic props.

## 2026-04-28 Release And Passive Push Review

Successful grabs now keep the prepared object tree dynamic on release instead of restoring one primary body back to its original motion preset. The reason is the same as active preparation: split-collision props and dropped weapons must stay coherent as a body set. A single-body restore would leave child collisions in mixed motion states. Failed setup still restores through the recursive wrapper before ROCK commits to the grab.

Passive push uses body-ID velocity updates after scanning and deduplicating the target object's accepted dynamic motion set. ROCK deliberately does not use the older collision-object impulse helper for this path because a contacted collision object can represent only one child body, while the object-tree scan tells ROCK which bodies share a motion and prevents duplicate velocity writes.

Ghidra recheck for the code paths used by this pass:

- `0x141DF95B0` still builds command `5`, writes the motion preset and activate flag, and dispatches through `0x141DFA2B0`.
- `0x141DF9940` still builds command `9`, writes enable/recursive flags, and dispatches through `0x141DFA2B0`.
- `0x141DFA2B0` applies the command callback to the current collision object, then recurses through NiNode children only when the recursive flag at command offset `0x38` is set.
- `0x141DF56F0` accepts body ID plus linear/angular velocity vectors and routes through the deferred command path when running during the physics command window.
- `0x141546EF0` validates the hknp body entry through body-array stride `0x90` before activating the body.
