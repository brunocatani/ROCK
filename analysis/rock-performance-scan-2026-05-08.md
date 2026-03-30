# ROCK Performance Scan - 2026-05-08

This pass measures ROCK frame cost without letting timer data affect gameplay. Debug overlay and verbose diagnostic channels were identified as production FPS risks, while hand, body, weapon, grab, selection, soft contact, haptics, and pull behavior stay enabled.

## Current Findings

- Full body colliders add generated keyframed hknp bodies and update from the live skeleton every frame.
- Hand/body/weapon generated collider drives also flush on the Havok physics-step boundary.
- Debug overlay rendering can do body extraction, shape decode/fingerprint work, GPU buffer work, line batching, and text rendering on the OpenVR submit path.
- Debug-level logging and contact identity logging can add work in frame and contact paths.
- Soft contact builds hand/body runtime shapes and can run world probes.
- Selection shape casts can create hit pressure during near/far selection updates.
- Native contact callbacks can classify body pairs and publish provider/native evidence.

## Implemented Instrumentation

- Added `src/physics-interaction/performance/PerformanceProfiler.h/.cpp`.
- Added `[Debug]` INI keys:
  - `bPerformanceProfilerEnabled = false`
  - `iPerformanceProfilerLogIntervalFrames = 300`
  - `iPerformanceProfilerWarmupFrames = 120`
  - `bPerformanceProfilerOverlayText = false`
- Timed scopes:
  - Frame update
  - Hand collider update
  - Body collider update
  - Generated collider physics flush
  - Weapon collision
  - Selection casts
  - Soft contact
  - Debug overlay publish
  - Debug overlay render
  - Contact resolve
  - Native contact callbacks
- Selection casts also count hit totals as pressure events.
- QueryPerformanceCounter is isolated to `PerformanceProfiler.cpp`; disabled scoped timers return before sampling.

## Production INI Profile

Updated packaged fallback `data/config/ROCK.ini` and active production `C:\Users\SENECA\Documents\My Games\Fallout4VR\ROCK_Config\ROCK.ini` in place:

- `iLogLevel = 2`
- Expensive debug visuals/log channels off by default:
  - Collider overlays, target colliders, hand axes, grab pivots, palm vectors
  - Hand/body/weapon debug collider drawing
  - Soft-contact debug drawing
  - Contact target identity logging
  - Grab frame logging and grab transform telemetry/text/axes
  - Root-flattened skeleton markers, skeleton visualizer, skeleton axes, skeleton bone logging/truncation logging
- Gameplay systems remain enabled:
  - ROCK main switch
  - Input remap
  - Weapon collision and static-world weapon collision
  - Soft contact hand/weapon/body/world
  - Native character-controller object filter
  - Selection highlight
  - Body bone colliders and body static-world collision
  - Hand static-world collision
  - Pull catch, haptics, and grab support systems

## Measurement Status

No live in-game profiler window has been captured in this source pass. Once `bPerformanceProfilerEnabled = true` is enabled for a test run, ROCK.log will emit per-window average/max/total milliseconds and sample/event counts for the scopes above. `bPerformanceProfilerOverlayText = true` can show the most recent window in the debug overlay without enabling collider visualization.

## Follow-Up Decision Gate

Runtime optimization should be chosen from profiler output:

- Body collider update/flush high: reduce repeated skeleton lookup/allocation and add target-change skips.
- Soft contact high: reuse frame skeleton/body snapshots and fixed storage where possible.
- Selection casts high: reduce per-cast containers and inspect near/far hit pressure.
- Native contact callback high: prefilter body pairs earlier and reduce shared-state pressure.
- Debug overlay render high when enabled: keep diagnostic-only and optimize shape cache/fingerprint work separately.
