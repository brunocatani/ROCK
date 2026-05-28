# Potential Fixes Plan

## Solution Families

### 1. Use one contract for both position and rotation

- Make the authority frame fully raw hand.
- Or make the authority frame fully proxy.
- This removes the hybrid mismatch completely.

What it solves:

- No more "translation from one frame, rotation from another" problem.

Tradeoff:

- raw/raw usually loses the nice physical palm-anchor seat.
- proxy/proxy usually loses the nicer controller-driven object orientation.
- In this project, proxy/proxy is especially risky because current comments and history strongly suggest that pure proxy rotation was already worse.

### 2. Keep the hybrid frame, but blend rotation when mismatch is large

- Normal case: keep current behavior, raw-hand rotation.
- Bad case: if raw/proxy angle exceeds a threshold, blend raw-hand rotation toward proxy rotation.
- Could be:
- hard threshold
- smooth slerp
- startup-only blend
- blend only until TouchHeld

What it solves:

- Only affects the intermittent bad grabs.
- Keeps the current "best" feel for normal grabs.

Tradeoff:

- You are adding a second rotational regime.
- If overdone, it can make the hand feel less direct.
- But this is probably the safest behavioral fix for "only some grabs."

### 3. Keep hybrid orientation, but stop using cross-contract pivot conversion as the B-side source of truth

- This is the clean mathematical fix.
- Right now the risky part is not just the hybrid frame itself.
- The risky part is that ROCK uses:
- A-side point in proxy/body-A local space
- plus desired-body relation in raw-hand authority space
- to derive solver-side transform-B translation

Alternative:

- Treat the held-body semantic grip point as the primary B-side truth.
- Use `pivotBBodyLocalGame` directly as the solver’s B pivot source, or otherwise derive transform-B from one consistent B-side contract instead of from the frozen A-local proxy point.

What it solves:

- Removes the exact "A-local point under one basis feeding B under another basis" bridge that can become weak on some grabs.

Tradeoff:

- This is more invasive than a blend.
- It changes the internal solver contract, not just the startup behavior.
- But it targets the exact mismatch mechanism most directly.

### 4. Freeze pivot A in authority-frame local space, not proxy-local space

- Another clean alternative.
- Instead of freezing the key A-side local point in proxy/body-A local coordinates for the B-side math, freeze it in the same authority frame that owns the raw-hand rotation.
- Then convert to proxy-local only when writing transform A to the actual constraint.

What it solves:

- Unifies the freeze math under one orientation contract.
- Keeps the proxy body for physical body A while removing part of the cross-contract freeze coupling.

Tradeoff:

- Also invasive.
- More math churn than a blend/gate fix.
- But cleaner than keeping the mismatch at the center of the freeze relation.
