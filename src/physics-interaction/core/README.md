# Core Runtime

This module owns the `PhysicsInteraction` lifecycle, frame orchestration, hooks, and provider-facing runtime state. It intentionally delegates native Havok operations, hand/grab behavior, weapon behavior, and debug rendering to domain modules so the main update loop remains the coordinator rather than the place where every subsystem reimplements its own boundary logic.
