#pragma once

// PhysicsHooks.h -- Engine hook installations for ROCK physics.
// Extracted from PhysicsInteraction.cpp for single-responsibility.
// Each hook is a one-time trampoline write persisting for DLL lifetime.

namespace frik::rock
{
	void installCCRadiusHook();
	void installBumpHook();
	void installNativeGrabHook();
	void installRefreshManifoldHook();
}
