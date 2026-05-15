# Dynamic Grab 2000-Frame Diagnostic Pass

## Why This Exists

The current grab issue is not fully visible in startup-only logs. Several bad grabs
start clean, then drift or twitch after the constraint, proxy authority, visual node,
and player-space writer have all run for multiple frames. The diagnostic window is
therefore extended to 2000 held frames so the log captures grab startup, settle,
late visual/body drift, player movement compensation, proxy flushes, and after-solve
readback over the same hold.

This pass is telemetry only. It does not change grab pivot selection, motor tuning,
constraint targets, proxy motion, player-space compensation behavior, hand pose, or
collision activation.

## Runtime Setting

- `iDebugGrabHighDetailFrames = 2000`

When `bDebugGrabFrameLogging = true`, this setting keeps high-detail grab diagnostics
at per-frame cadence for the first N held frames. After the window expires, existing
periodic logging resumes.

## Captured Data

- `GRAB FRAME HOLD`
  - held-frame index;
  - raw hand vs constraint anchor relation;
  - body vs raw/constraint target rotation;
  - owner/hit/held/root node drift against body-derived expectations.

- `GRAB FRAME NODES`
  - held-frame index;
  - collision owner, hit node, held node, and root node identity;
  - whether each node owns the hknp collision object.

- `GRAB FRAME VISUALS`
  - held-frame index;
  - desired, body, owner, hit, held, and root direction vectors.

- `GRAB ANGULAR PROBE`
  - held-frame index;
  - raw axis row/column error;
  - motor drive error;
  - mass/body/motion summary;
  - queue/flush/failure counters.

- `HELD dynamic`
  - held-frame index;
  - dynamic drive, loose weapon flag, form ID, constraint ID;
  - pivot and target-body errors;
  - object velocity and force budget.

- `PROXY GRAB AUTHORITY`
  - first 2000 proxy flushes;
  - queued vs flushed sequence;
  - queue lag;
  - raw hand to proxy rotation/position delta;
  - proxy readback and no-contact filter state.

- `PROXY GRAB AFTER_SOLVE`
  - first 2000 after-solve samples;
  - proxy/object readback against target and live-proxy expectations;
  - grip target and live-proxy errors.

- `Held player-space`
  - first 2000 held player-space samples;
  - player-space delta, rotation delta, warp flags, and velocity.

- `Held player-space central writer`
  - first 2000 held writer passes;
  - whether the central writer is touching registered held bodies while the grab
    constraint is active.

## Expected Use

Test both hands and include at least one object that appears clean and one that
shows twitch or wrist-break behavior. The important comparison is not frame 1 only;
it is where the first non-zero divergence appears between visual node, body frame,
proxy authority, and player-space writer.
