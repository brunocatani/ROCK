# Native Mouse-Spring Grab Tuning - 2026-05-08

## Finding

Normal single-hand close grabs are driven by `HeldObjectDriveMode::NativeMouseSpring`, not by ROCK's shared grab constraint. The per-frame target is queued with `_nativeGrab.queueTarget(desiredBodyWorld)` and flushed at the Havok step through `hknpBSMouseSpringAction_Update`.

The constraint knobs in section 13 (`fGrabLinearTau`, `fGrabAngularTau`, damping, recovery, adaptive motor) still matter for shared/two-hand constraint grabs, but they do not govern the native mouse-spring response used by ordinary single-hand dynamic grabs.

## Decision

ROCK now exposes native mouse-spring response scales in the INI:

- `fGrabNativeMouseSpringLinearResponseScale`
- `fGrabNativeMouseSpringAngularResponseScale`
- `fGrabNativeMouseSpringAngularClampScale`

These scale the verified native cinfo fields read from FO4VR's own mouse-spring constants. This keeps the native BODY-frame action boundary and Bethesda update smoothing intact while giving tuning authority over the observed feel problem: translation lagging behind while rotation catches up too aggressively.

## Smoothing Note

There is no separate ROCK target smoothing layer for mouse-spring grabs. The smoothing on that path is the native `hknpBSMouseSpringAction_Update` behavior plus the response values in its cinfo. Adding another target smoothing layer in ROCK would delay the object target before the native spring sees it and would make the current translation lag harder to tune.
