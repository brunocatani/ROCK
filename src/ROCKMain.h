#pragma once

// ROCKMain.h -- Public interface for the ROCK F4SE plugin entry point.
//
// WHY: PhysicsInteraction needs to dispatch F4SE messages through ROCK's own
// messaging interface (replacing the monolith's g_frik.broadcastMessage()).
// This header exposes the minimal set of functions that other ROCK source files
// need from the entry point module.
//
// This header is intentionally lean -- it only declares functions that cross
// the ROCKMain.cpp / PhysicsInteraction.cpp boundary. All other entry point
// state is private to ROCKMain.cpp's anonymous namespace.

namespace F4SE { class MessagingInterface; }

namespace rock
{
    /// Get ROCK's F4SE messaging interface for dispatching physics events.
    ///
    /// Returns nullptr if F4SEPlugin_Load has not completed yet.
    /// PhysicsInteraction uses this to broadcast touch/grab/release events
    /// to external mods that registered as ROCK listeners.
    ///
    /// Usage (replaces g_frik.broadcastMessage()):
    ///   if (auto* m = rock::getROCKMessaging()) {
    ///       m->Dispatch(msgType, &data, sizeof(data), nullptr);
    ///   }
    const F4SE::MessagingInterface* getROCKMessaging();
}
