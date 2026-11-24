// SPDX-License-Identifier: MIT
//
// This interface header and its companion DevBenchAPI.cpp are MIT-licensed (see
// DevBenchAPI.LICENSE.txt) so ANY script-extender plugin — including
// proprietary/closed-source — may vendor them to talk to devbench, independent of the
// devbench plugin's GPL-3.0. They are self-contained: drop both files into your plugin
// (or consume the devbench-api vcpkg port) and build — no other devbench source is
// needed.
#pragma once

#include <cstdint>

// NO SCRIPT-EXTENDER HEADER HERE, DELIBERATELY.
//
// Two reasons, and the second is a hard build invariant:
//
// 1. This ABI is game-neutral. Every declaration below is plain C++ — the message id,
//    the function-pointer types and the vtable. Nothing in it is Skyrim-shaped, and
//    pulling <SKSE/SKSE.h> in only ever made it *look* Skyrim-only. The one piece that
//    genuinely needs an extender is GetDevBenchInterface001(), which dispatches a
//    message; that lives in DevBenchAPI.cpp, which picks SKSE or F4SE at compile time.
//
// 2. devbench's own src/core/ MUST NOT include a script-extender header (see
//    docs/MULTIGAME.md — the unit-test target, which links neither extender, is what
//    keeps that honest). The host side of this ABI lives in the core, so it includes
//    this file; if this file pulled in SKSE the core could not.
//
// devbench cross-plugin API — lets another SKSE or F4SE plugin register MCP/REST tools
// and emit events into the running devbench host. Usage: once your plugin has loaded
// (SKSE kPostLoad / F4SE kPostLoad), request the interface via a messaging dispatch,
// then call through the versioned abstract interface below.
//
// The call direction is reversed from a typical query API: you hand devbench a handler
// that it calls back when a tool is invoked. So payloads are JSON strings and the
// handler is a plain C function pointer + void* ctx — never std::function or other C++
// objects across the DLL boundary.
namespace DevBenchAPI
{
	constexpr const auto DevBenchPluginName = "devbench";

	// Host-owned writer a handler uses to return its JSON result. Call exactly once
	// before returning; the host owns the buffer (so result-string ownership never
	// crosses the DLL boundary).
	using WriteFn = void (*)(void* a_sink, const char* a_resultJson);

	// A tool handler. Invoked on devbench's server (listener) thread — marshal to the
	// main game thread yourself if you touch game state. `a_argsJson` is the tool's
	// arguments object as a JSON string; return your result via a_write(a_sink, json).
	// MUST be a plain C function / captureless lambda.
	using ToolFn = void (*)(void* a_ctx, const char* a_argsJson, void* a_sink, WriteFn a_write);

	// Message used to fetch the interface (a fixed, randomly chosen id).
	struct DevBenchMessage
	{
		enum : std::uint32_t
		{
			kMessage_GetInterface = 0x9a3f1c08
		};
		void* (*GetApiFunction)(unsigned int a_revisionNumber) = nullptr;
	};

	struct IDevBenchInterface001;
	// WHEN TO CALL THIS:
	//   Skyrim  — from your kDataLoaded handler (devbench sets itself up at kPostLoad).
	//   Fallout — from your kPostPostLoad handler. NOT kPostLoad: devbench does its own
	//             setup at kPostLoad, and F4SE runs every plugin's kPostLoad handler in
	//             plugin LOAD ORDER, so a consumer sorting before devbench would ask
	//             before the provider exists and get nullptr. kPostPostLoad is dispatched
	//             immediately after kPostLoad and exists precisely for this second phase.
	//
	// Returns nullptr if devbench is absent — normal, not an error: devbench is a
	// development dependency and must never become a load-order requirement. The result
	// is cached only on success, so a nullptr result is safe to retry at a later message.
	//
	// Defined in DevBenchAPI.cpp, which is the only extender-aware part of this ABI:
	// it compiles against SKSE or F4SE (auto-detected, overridable — see that file).
	IDevBenchInterface001* GetDevBenchInterface001();

	struct IDevBenchInterface001
	{
		// devbench build number: VERSION_MAJOR*10000 + MINOR*100 + PATCH.
		virtual unsigned int GetBuildNumber() = 0;

		// Register a tool, exposed over both MCP (/mcp) and REST (/api/tool/<name>).
		// a_descriptorJson: { "description": str, "inputSchema": obj, "readOnly": bool }.
		// Returns false if a tool of this name already existed (it is still replaced).
		virtual bool RegisterTool(const char* a_name, const char* a_descriptorJson,
			ToolFn a_handler, void* a_ctx) = 0;

		// Publish an event (MCP notification + REST /api/events?since=N).
		virtual void EmitEvent(const char* a_topic, const char* a_payloadJson) = 0;

		// Register a handler for a custom menu, dispatched through the base `menu` tool as
		// action='invoke', name='<menuName>'. Thin alias for RegisterToolExtension("menu", …) — kept
		// for the 1.4.0 ABI. Same handler contract as RegisterTool. Returns false if it replaced an
		// existing handler. ABI: vtable slot exists only on hosts that ship it — call only when
		// GetBuildNumber() >= 10400 (devbench 1.4.0).
		virtual bool RegisterMenuHandler(const char* a_menuName, const char* a_descriptorJson,
			ToolFn a_handler, void* a_ctx) = 0;

		// Register a handler that EXTENDS a built-in base tool, keyed by a string, so a mod adds a
		// sub-capability WITHOUT a top-level tool (the agent-facing surface stays small). The base
		// tool routes to it: `menu invoke name='<key>'`, `inspect kind='<key>'`. Same handler
		// contract as RegisterTool (args JSON in — the full base-tool args — result JSON out, runs on
		// the listener thread; marshal to the main thread yourself). a_descriptorJson
		// { "description", … } is surfaced by the base tool's discovery path (`menu describe`,
		// `inspect kind=extensions`). Opted-in base tools: `menu`, `inspect`. Returns false if it
		// replaced an existing (baseTool, key) entry.
		//
		// Declare the arguments your handler reads beyond the base tool's own as
		// "inputSchema": { "type": "object", "properties": { … } } — the same shape RegisterTool
		// takes. devbench folds them into the base tool's schema, so they appear in tools/list and
		// pass its unknown-argument check. Without one, devbench cannot know your arguments and
		// stops checking that base tool's argument names while you are registered.
		//
		// ABI: appended after RegisterMenuHandler — call only when GetBuildNumber() >= 10500
		// (devbench 1.5.0).
		virtual bool RegisterToolExtension(const char* a_baseTool, const char* a_key,
			const char* a_descriptorJson, ToolFn a_handler, void* a_ctx) = 0;

		// Set the game's global time scale — the same control as `game setTimeScale`, so a consumer
		// can speed up or slow down a run (e.g. 3.0 to finish a benchmark sooner). a_scale is
		// 0.1..3.0; a_leaseMs is how long it stays in effect before the previous scale is restored
		// (0 = 60000), so an interrupted consumer still leaves the game at normal speed.
		// NON-BLOCKING and callable from ANY thread (including the main thread): the change is
		// applied on a later main-thread frame; there is no RunAndWait. Returns false when the scale
		// is out of range, or when a recording/capture is in flight and would be made incomparable.
		// Platforms without time-scale support return false (currently Fallout 4).
		// a_owner identifies the lease for `game getTimeScale` (nullptr = an anonymous owner).
		//
		// ABI: appended after RegisterToolExtension — call only when GetBuildNumber() >= 12000
		// (devbench 1.20.0).
		virtual bool SetTimeScale(float a_scale, std::uint32_t a_leaseMs, const char* a_owner) = 0;

		// The global time multiplier the game is running at right now (what the engine has actually
		// reached, not a value a request left pending). 1.0 when nothing has changed it.
		// Returns NaN on platforms without time-scale support (currently Fallout 4).
		//
		// ABI: appended after SetTimeScale — call only when GetBuildNumber() >= 12000
		// (devbench 1.20.0).
		virtual float GetTimeScale() = 0;

		// Posts a message that every tool reply carries (a `notices` array) until it is cleared or
		// a_ttlMs elapses (0 = until cleared), so an agent or a headless script sees a critical
		// condition on its next call. A repeated a_key replaces the notice. a_severity is "info",
		// "warning" or "critical" (anything else reads as "info"). Returns false for an empty key or text.
		//
		// ABI: appended after GetTimeScale — call only when GetBuildNumber() >= 13400
		// (devbench 1.34.0).
		virtual bool PostNotice(const char* a_key, const char* a_source, const char* a_severity, const char* a_text, std::uint32_t a_ttlMs) = 0;

		// Removes the notice with a_key. Returns false if there was none.
		//
		// ABI: appended after PostNotice — call only when GetBuildNumber() >= 13400.
		virtual bool ClearNotice(const char* a_key) = 0;
	};

	// ---- Main-thread service: separate from IDevBenchInterface001 ON PURPOSE ----
	//
	// Run a callback once on the game's main thread, between frames, on the same queue
	// devbench's own tools use. Why it exists: on Fallout 4 VR, F4SE's AddTask and
	// AddUITask run tasks on WORKER threads (measured 2026-10-28, FO4VR 1.2.72: 43 of 43
	// AddUITask tasks off the main thread), so a mod that "marshals to the main thread"
	// through them does not get there, and D3D or game-state work done that way races the
	// engine. devbench drains its own queue right after the frame is drawn and submitted.
	//
	// Discovered by its OWN message, not by a new IDevBenchInterface001 slot or revision:
	// a devbench without it ignores the message and leaves `api` null, so the answer is
	// "not supported" rather than a call through a vtable slot that is not there. A
	// revision cannot carry it, because GetApi(n) has always returned the 001 interface
	// for any n >= 1, and a build number cannot either, because this fork reports
	// upstream's version. On Fallout, where the message route can miss (see
	// DevBench_GetApiFunction), the DLL also exports DevBench_GetMainThreadApi.
	// DevBenchAPI.cpp's GetDevBenchMainThread() tries both.
	using MainThreadFn = void (*)(void* a_ctx);

	struct DevBenchMainThreadApi
	{
		// 1 = the fields below. A later version only appends.
		std::uint32_t version = 1;

		// Queue a_fn(a_ctx) to run once on the main thread and return at once. Callable
		// from any thread; from the main thread itself, a_fn runs on a later frame.
		// false = not queued (the host has no queue at all); a_fn will not be called.
		// a_fn must not throw, and a_ctx must stay valid until a_fn has run.
		bool (*RunOnMainThread)(MainThreadFn a_fn, void* a_ctx) = nullptr;

		// Whether devbench has ESTABLISHED, by measurement, that this queue runs on the
		// game's main thread on this runtime. true on Fallout 4 VR 1.2.72. false means
		// "not established": on another Fallout runtime the queue falls back to F4SE's
		// AddTask; on Skyrim it is SKSE's task queue, which nothing here has measured.
		bool (*ConfinedToMainThread)() = nullptr;
	};

	// Message used to fetch DevBenchMainThreadApi (a fixed, randomly chosen id). Same
	// dispatch as DevBenchMessage, to the same receiver.
	struct DevBenchMainThreadMessage
	{
		enum : std::uint32_t
		{
			kMessage_GetMainThread = 0x5d27e0b1
		};
		const DevBenchMainThreadApi* api = nullptr;  // stays null on a devbench without it
	};

	// Consumer helper (DevBenchAPI.cpp): the main-thread service, or nullptr when this
	// devbench does not offer it. Cached after the first success.
	const DevBenchMainThreadApi* GetDevBenchMainThread();
}

extern DevBenchAPI::IDevBenchInterface001* g_devBenchInterface;
