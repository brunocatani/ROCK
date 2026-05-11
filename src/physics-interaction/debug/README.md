# Debug Domain

This module owns overlay rendering, debug axes, skeleton visualization, and world-origin diagnostics. Debug code still reads some renderer/camera/native layouts for visualization, but those offsets stay isolated here so production interaction logic does not depend on debug-only native assumptions.
