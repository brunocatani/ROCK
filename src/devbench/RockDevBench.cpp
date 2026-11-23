#include "devbench/RockDevBench.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <format>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "RockConfig.h"
#include "devbench/vendor/DevBenchAPI.h"
#include "physics-interaction/performance/PerformanceProfiler.h"

namespace rock::devbench
{
    namespace
    {
        using json = nlohmann::json;

        // Bumped whenever a generic action changes its arguments or the shape of its answer; health reports it.
        constexpr int kToolContract = 1;

        constexpr auto kToolName = "rock";

        // devbench's own stall watchdog is 5000ms; answering well before it keeps "ROCK did not answer" apart from
        // "devbench is stalled".
        constexpr auto kGameThreadTimeout = std::chrono::milliseconds(2000);

        // FrameBegin ran this recently, so a queued action runs at the next one. Later than this, FRIK's frame has
        // stopped (main menu, loading) and the action goes to devbench's main-thread queue instead.
        constexpr auto kFrameStaleAfter = std::chrono::milliseconds(250);

        /**
         * Serializes an answer for the host. Invalid UTF-8 is replaced rather than thrown, so one stray byte in a
         * setting's text can't turn an answer into a failure.
         */
        std::string dump(const json& value) { return value.dump(-1, ' ', false, json::error_handler_t::replace); }

        /**
         * A failed call. It carries no data keys, so a caller can't read a zero out of it and take it for a value.
         */
        std::string errorJson(const std::string& message)
        {
            return dump(json{ { "ok", false }, { "error", message } });
        }

        /**
         * A scalar argument as text: a string as it is, a number or boolean in its JSON form, so set takes value=1.5
         * as well as value="1.5". Empty when the argument is absent or null.
         */
        std::string argString(const json& args, const char* name)
        {
            const auto it = args.find(name);
            if (it == args.end() || it->is_null())
                return {};
            if (it->is_string())
                return it->get<std::string>();
            if (it->is_number() || it->is_boolean())
                return it->dump();
            throw std::invalid_argument(std::format("argument '{}' must be a string, number or boolean", name));
        }

        bool argBool(const json& args, const char* name)
        {
            const auto it = args.find(name);
            return it != args.end() && it->is_boolean() && it->get<bool>();
        }

        json argument(const char* type, const char* description)
        {
            return { { "type", type }, { "description", description } };
        }

        const char* ownerName(PreviewOwner owner) noexcept
        {
            return owner == PreviewOwner::DevBench ? "devbench" : "modsconfig";
        }

        const char* fileName(config::Group group) noexcept
        {
            return group == config::Group::Developer ? "ROCK_Developer.toml" : "ROCK.toml";
        }

        std::int64_t steadyMilliseconds() noexcept
        {
            return std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch())
                .count();
        }

        /**
         * The TOML table and key a config, set or clear call names. Both are required: ROCK has many tables and no
         * main one to default to.
         */
        std::pair<std::string, std::string> sectionAndKey(const json& args, const char* action)
        {
            auto section = argString(args, "section");
            auto key = argString(args, "key");
            if (section.empty() || key.empty()) {
                throw std::invalid_argument(std::format(
                    "action '{}' needs 'section' (the TOML table, as \"Grabbing\" or \"FiringHand.Left\") and 'key'",
                    action));
            }
            return { std::move(section), std::move(key) };
        }

        enum class RunOn
        {
            // Queued for the game thread; the caller waits up to kGameThreadTimeout.
            GameThread,
            // Answered on devbench's listener thread: reads only atomics and data guarded for any thread.
            Listener,
        };

        struct Action
        {
            const char* name;
            const char* description;
            json arguments;
            json (*handler)(const json& args);
            RunOn runOn;
        };

        std::atomic<bool> s_registered{ false };
        std::atomic<bool> s_armed{ false };
        DevBenchAPI::IDevBenchInterface001* s_interface = nullptr;
        unsigned int s_hostBuild = 0;

        // The steady-clock milliseconds of the last FrameBegin, 0 before the first.
        std::atomic<std::int64_t> s_lastFrameBeginMs{ 0 };

        /**
         * A game-thread action waiting for its frame. The promise is shared, never on the waiting thread's stack:
         * on timeout that thread returns while the command is still queued.
         */
        struct Command
        {
            std::function<std::string()> fn;
            std::shared_ptr<std::promise<std::string>> result;
        };

        std::mutex s_commandsLock;
        std::vector<Command> s_commands;
        std::atomic<bool> s_hasCommands{ false };

        void execute(const Command& command)
        {
            std::string result;
            try {
                result = command.fn();
            } catch (const std::exception& error) {
                result = errorJson(std::format("the action threw on the game thread: {}", error.what()));
            }
            try {
                command.result->set_value(std::move(result));
            } catch (const std::future_error&) {}
        }

        /**
         * devbench's main-thread queue callback: runs the command and frees it. It must not throw, and execute()
         * catches everything.
         */
        void executeQueued(void* context)
        {
            const std::unique_ptr<Command> command(static_cast<Command*>(context));
            execute(*command);
        }

        std::span<const Action> actions();

        json health(const json&)
        {
            json names = json::array();
            for (const auto& action : actions())
                names.push_back(action.name);
            return {
                { "plugin", "ROCK" },
                { "version", std::string(Version::NAME) },
                { "contract", kToolContract },
                { "tool", kToolName },
                { "devbench", s_hostBuild },
                { "armed", s_armed.load(std::memory_order_relaxed) },
                { "actions", names },
                // Generic actions of F4VR-CommonFramework mods that ROCK's tool does not have.
                { "unsupported", json::array({ "state", "input", "snapshot" }) },
            };
        }

        /**
         * Game thread: the option as the next apply reads it, with the file's value beneath it.
         */
        json readConfig(const json& args)
        {
            const auto [section, key] = sectionAndKey(args, "config");
            const auto setting = g_rockConfig.settingValue(section, key);
            if (!setting)
                throw std::invalid_argument(std::format("ROCK has no option [{}] {}", section, key));
            return {
                { "section", section },
                { "key", key },
                { "value", setting->value },
                { "file", fileName(setting->group) },
                { "fileValue", setting->fileValue },
                { "overriddenBy", setting->previewOwner ? json(ownerName(*setting->previewOwner)) : json(nullptr) },
            };
        }

        /**
         * Game thread, before this frame's settings apply: the value lays over the file's from this frame on.
         */
        json setConfig(const json& args)
        {
            const auto [section, key] = sectionAndKey(args, "set");
            if (!args.contains("value"))
                throw std::invalid_argument("action 'set' needs 'value'");
            const auto value = argString(args, "value");
            if (!g_rockConfig.setPreviewValue(PreviewOwner::DevBench, section, key, value)) {
                throw std::invalid_argument(
                    std::format("[{}] {} = '{}': no such option, or not a value of it; nothing is set", section, key,
                        value));
            }
            const auto setting = g_rockConfig.settingValue(section, key);
            return {
                { "section", section },
                { "key", key },
                { "value", setting ? json(setting->value) : json(nullptr) },
                { "file", setting ? json(fileName(setting->group)) : json(nullptr) },
                { "note", "session override, applied from this frame; the file is unchanged" },
            };
        }

        /**
         * Game thread: drops only devbench's own values; Mods Config's live preview stays.
         */
        json clearConfig(const json& args)
        {
            if (argBool(args, "all")) {
                const auto cleared = g_rockConfig.clearPreviewValues(PreviewOwner::DevBench);
                return { { "cleared", cleared },
                    { "note", "every session override set made is dropped; the files' values apply from this frame" } };
            }
            const auto [section, key] = sectionAndKey(args, "clear");
            const bool hadOverride = g_rockConfig.discardPreviewValue(PreviewOwner::DevBench, section, key);
            const auto setting = g_rockConfig.settingValue(section, key);
            return {
                { "section", section },
                { "key", key },
                { "hadOverride", hadOverride },
                { "overridden", setting && setting->previewOwner.has_value() },
                { "note", "the file's value applies from this frame, unless 'overridden': Mods Config previews it too" },
            };
        }

        json listOverrides(const json&)
        {
            json overrides = json::array();
            for (const auto& entry : g_rockConfig.previewValues()) {
                overrides.push_back({ { "section", entry.table }, { "key", entry.key }, { "value", entry.value },
                    { "owner", ownerName(entry.owner) }, { "applies", entry.applies } });
            }
            return { { "overrides", overrides } };
        }

        /**
         * The window as a table: scopes by total time, then counters and values.
         */
        std::string perfText(const performance_profiler::DevBenchWindow& window)
        {
            auto scopes = window.scopes;
            std::ranges::sort(scopes, std::ranges::greater{}, &performance_profiler::WindowScope::totalMs);
            const double frames = static_cast<double>((std::max)(window.frames, std::uint64_t{ 1 }));
            std::string text = std::format("perf: {:.1f}s window, {} frames, times in ms\n", window.windowMs / 1000.0,
                window.frames);
            text += std::format("{:<44} {:>9} {:>9} {:>9} {:>10} {:>9} {:>8}\n", "scope", "n", "avg", "max", "total",
                "/frame", "events");
            for (const auto& scope : scopes) {
                const double avg = scope.samples ? scope.totalMs / static_cast<double>(scope.samples) : 0.0;
                text += std::format("{:<44} {:>9} {:>9.3f} {:>9.3f} {:>10.2f} {:>9.3f} {:>8}\n", scope.name,
                    scope.samples, avg, scope.maxMs, scope.totalMs, scope.totalMs / frames, scope.events);
            }
            if (!window.counters.empty()) {
                text += "counters\n";
                for (const auto& counter : window.counters)
                    text += std::format("  {:<42} {:>9}\n", counter.name, counter.count);
            }
            if (!window.values.empty()) {
                text += std::format("{:<44} {:>9} {:>11} {:>11}\n", "values", "n", "avg", "max");
                for (const auto& value : window.values) {
                    const double avg = static_cast<double>(value.total) / static_cast<double>(value.samples);
                    text += std::format("  {:<42} {:>9} {:>11.1f} {:>11}\n", value.name, value.samples, avg, value.max);
                }
            }
            return text;
        }

        /**
         * Listener thread: the profiler's accumulators are atomics, so reading never waits for or stalls a frame.
         * ROCK's scopes do not nest, so the tree format answers flat.
         */
        json readPerf(const json& args)
        {
            const bool reset = argBool(args, "reset");
            auto format = argString(args, "format");
            if (format.empty())
                format = "tree";
            if (format != "tree" && format != "flat" && format != "text")
                throw std::invalid_argument(std::format("unknown format '{}' (tree, flat, text)", format));

            const auto window = performance_profiler::readDevBenchWindow(reset);
            json answer = {
                { "reset", reset },
                { "windowMs", window.windowMs },
                { "frames", window.frames },
                { "collecting", window.collecting },
                { "logging", window.logging },
            };
            if (window.logging) {
                answer["note"] = "the profiler log is on: a scope's max may predate the last reset by up to one "
                                 "log interval";
            }
            if (format == "text") {
                answer["format"] = "text";
                answer["text"] = perfText(window);
                return answer;
            }
            answer["format"] = "flat";

            const double windowMs = window.windowMs;
            const double frames = static_cast<double>(window.frames);
            json sites = json::object();
            for (const auto& scope : window.scopes) {
                json site = {
                    { "n", scope.samples },
                    { "totalMs", scope.totalMs },
                    { "avgMs", scope.samples ? scope.totalMs / static_cast<double>(scope.samples) : 0.0 },
                    { "maxMs", scope.maxMs },
                    { "busyPct", windowMs > 0 ? scope.totalMs / windowMs * 100.0 : 0.0 },
                };
                if (frames > 0) {
                    site["callsPerFrame"] = static_cast<double>(scope.samples) / frames;
                    site["msPerFrame"] = scope.totalMs / frames;
                }
                if (scope.events)
                    site["events"] = scope.events;
                sites[scope.name] = std::move(site);
            }
            json counters = json::object();
            for (const auto& counter : window.counters)
                counters[counter.name] = counter.count;
            json values = json::object();
            for (const auto& value : window.values) {
                values[value.name] = { { "n", value.samples },
                    { "avg", static_cast<double>(value.total) / static_cast<double>(value.samples) },
                    { "max", value.max } };
            }
            answer["sites"] = std::move(sites);
            answer["counters"] = std::move(counters);
            answer["values"] = std::move(values);
            return answer;
        }

        std::span<const Action> actions()
        {
            static const std::vector<Action> table = [] {
                const auto section =
                    argument("string", "config/set/clear: the TOML table, as \"Grabbing\" or \"FiringHand.Left\"");
                const auto key = argument("string", "config/set/clear: the option's key in that table");
                return std::vector<Action>{
                    { "health",
                        "which ROCK and devbench build this is, the tool's contract, whether it is armed, and the "
                        "actions. Answered without the game thread, so it replies while the game is stalled",
                        json::object(), &health, RunOn::Listener },
                    { "config",
                        "one option as ROCK reads it from this frame: the last session value set by devbench or "
                        "Mods Config's live preview, otherwise the file's; with the file it lives in and the file's "
                        "value",
                        { { "section", section }, { "key", key } }, &readConfig, RunOn::GameThread },
                    { "set",
                        "override one option for this session without writing the file, in the text its file takes "
                        "(1/0, yes/no and \"x, y, z\" are accepted); it applies from this frame. A value its type "
                        "cannot hold is an error and sets nothing",
                        { { "section", section }, { "key", key },
                            { "value", argument("string", "set: the new value") } },
                        &setConfig, RunOn::GameThread },
                    { "clear",
                        "drop one session override that set made, or all of them with all=true, so the file's value "
                        "applies again. Mods Config's live preview of an option stays",
                        { { "section", section }, { "key", key },
                            { "all", argument("boolean", "clear: drop every session override set made") } },
                        &clearConfig, RunOn::GameThread },
                    { "overrides",
                        "the session values in effect, each with its owner ('devbench' for set, 'modsconfig' for Mods "
                        "Config's live preview) and whether it is the one that applies",
                        json::object(), &listOverrides, RunOn::GameThread },
                    { "perf",
                        "time spent in each of ROCK's profiler scopes since the last reset, all threads together "
                        "(scopes do not nest): n, total, avg and max ms, busy% of the window, calls and ms per "
                        "frame; plus the profiler's counters and value metrics. Collecting starts when the tool "
                        "registers, after the profiler's warmup frames, so reset, hold the condition, then read. "
                        "Answered without the game thread",
                        { { "reset", argument("boolean", "perf: start a new window after reading this one") },
                            { "format",
                                argument("string",
                                    "perf: flat (JSON keyed by scope name; tree is answered flat) or text (a table "
                                    "in 'text', the quickest to read)") } },
                        &readPerf, RunOn::Listener },
                };
            }();
            return table;
        }

        /**
         * What devbench lists for the tool: a description that names ROCK's features and every action, and one flat
         * input schema holding every action's arguments.
         */
        std::string buildDescriptor()
        {
            std::string text =
                std::format("ROCK v{}: physical hands for Fallout 4 VR - grabbing, pulling, throwing, hand, body and "
                            "weapon collision, two-handed weapons, shoulder stash, eating, grenades, settings and "
                            "profiler.\n\nThe required 'action' argument chooses the operation. Generic actions, "
                            "as in F4VR-CommonFramework mods:",
                    Version::NAME);
            json names = json::array();
            json properties = json::object();
            std::string listenerActions;
            for (const auto& action : actions()) {
                text += std::format("\n- {}: {}", action.name, action.description);
                names.push_back(action.name);
                for (const auto& [name, schema] : action.arguments.items()) {
                    if (!properties.contains(name))
                        properties[name] = schema;
                }
                if (action.runOn == RunOn::Listener)
                    listenerActions += std::format("{}{}", listenerActions.empty() ? "" : ", ", action.name);
            }
            text += std::format("\n{} answer at once, without the game thread. Every other action runs on the game "
                                "thread at ROCK's next frame, and fails after 2s if the game thread does not get there.",
                listenerActions);
            properties["action"] = { { "type", "string" }, { "enum", names }, { "description", "the operation to run" } };
            return dump(json{
                { "description", text },
                { "readOnly", false },
                { "inputSchema",
                    { { "type", "object" }, { "properties", properties }, { "required", json::array({ "action" }) } } },
            });
        }

        /**
         * Runs an action's handler and turns what it returns or throws into the answer.
         */
        std::string runHandler(const Action& action, const json& args)
        {
            try {
                auto result = action.handler(args);
                if (!result.contains("ok"))
                    result["ok"] = true;
                return dump(result);
            } catch (const std::exception& error) {
                return errorJson(error.what());
            }
        }

        /**
         * Listener thread: hands the command to the game thread and waits for its answer. ROCK's FrameBegin runs it
         * while FRIK's frame runs; otherwise devbench's main-thread queue does. F4SE's task queue is never used: on
         * Fallout 4 VR it runs tasks on worker threads.
         */
        std::string runOnGameThread(std::function<std::string()> fn)
        {
            auto promise = std::make_shared<std::promise<std::string>>();
            auto future = promise->get_future();
            Command command{ std::move(fn), std::move(promise) };

            const auto lastFrame = s_lastFrameBeginMs.load(std::memory_order_acquire);
            if (lastFrame != 0 && steadyMilliseconds() - lastFrame < kFrameStaleAfter.count()) {
                std::scoped_lock lock(s_commandsLock);
                s_commands.push_back(std::move(command));
                s_hasCommands.store(true, std::memory_order_release);
            } else if (const auto* mainThread = DevBenchAPI::GetDevBenchMainThread()) {
                auto queued = std::make_unique<Command>(std::move(command));
                if (!mainThread->RunOnMainThread(&executeQueued, queued.get()))
                    return errorJson("ROCK's frame is not running, and devbench did not queue the action");
                // The queue owns it now; executeQueued frees it.
                (void)queued.release();
            } else {
                return errorJson("ROCK's frame is not running, and this devbench has no main-thread queue");
            }

            if (future.wait_for(kGameThreadTimeout) != std::future_status::ready) {
                return errorJson(std::format(
                    "the game thread did not run the action within {}ms: it is stalled or paused, or ROCK's frame "
                    "stopped",
                    kGameThreadTimeout.count()));
            }
            return future.get();
        }

        std::string invoke(const char* argsJson)
        {
            json args;
            try {
                args = argsJson && *argsJson ? json::parse(argsJson) : json::object();
            } catch (const std::exception& error) {
                return errorJson(std::format("bad arguments JSON: {}", error.what()));
            }
            if (!args.is_object())
                return errorJson("the arguments must be a JSON object");

            std::string name;
            try {
                name = argString(args, "action");
            } catch (const std::exception& error) {
                return errorJson(error.what());
            }
            std::string known;
            for (const auto& action : actions())
                known += std::format("{}{}", known.empty() ? "" : "|", action.name);
            if (name.empty())
                return errorJson(std::format("missing 'action' ({})", known));

            const auto all = actions();
            const auto action = std::ranges::find_if(all, [&](const Action& entry) { return name == entry.name; });
            if (action == all.end())
                return errorJson(std::format("unknown action '{}' ({})", name, known));
            if (action->runOn == RunOn::Listener)
                return runHandler(*action, args);
            return runOnGameThread([action = *action, args] { return runHandler(action, args); });
        }

        /**
         * What devbench calls, on its listener thread. Nothing may be thrown back across the C ABI.
         */
        void toolHandler(void*, const char* argsJson, void* sink, DevBenchAPI::WriteFn write)
        {
            std::string result;
            try {
                result = invoke(argsJson);
            } catch (const std::exception& error) {
                result = errorJson(std::format("the tool failed: {}", error.what()));
            } catch (...) {
                result = errorJson("the tool failed with an unknown exception");
            }
            write(sink, result.c_str());
        }

        /**
         * Something can read what costs per-frame work, so switch it on for the rest of the session: the profiler
         * collects what the perf action reads.
         */
        void arm()
        {
            if (!s_armed.exchange(true, std::memory_order_relaxed))
                performance_profiler::setDevBenchCollection(true);
        }
    }

    /**
     * Registers the "rock" tool when devbench is installed, and arms it. Game thread, once the game has loaded;
     * devbench sets itself up at kPostLoad, so it answers by then whatever the load order.
     */
    void registerTool()
    {
        if (s_registered.load(std::memory_order_acquire))
            return;
        s_interface = DevBenchAPI::GetDevBenchInterface001();
        if (!s_interface) {
            logger::info("ROCK: devbench is not installed; the '{}' devbench tool is not registered.", kToolName);
            return;
        }
        // Only the slots every devbench has are used (GetBuildNumber, RegisterTool, EmitEvent); a later slot must be
        // gated on this build number.
        s_hostBuild = s_interface->GetBuildNumber();
        const auto descriptor = buildDescriptor();
        if (!s_interface->RegisterTool(kToolName, descriptor.c_str(), &toolHandler, nullptr)) {
            logger::warn("ROCK: registering the '{}' devbench tool replaced an existing tool of that name.", kToolName);
        }
        s_registered.store(true, std::memory_order_release);
        logger::info("ROCK: devbench build {}: registered the '{}' tool.", s_hostBuild, kToolName);
        arm();
    }

    /**
     * FrameBegin, before the settings apply: notes that ROCK's frame runs, then runs the actions queued since the
     * last one. One relaxed load while devbench is absent or nothing is queued.
     */
    void onFrameBegin()
    {
        if (!s_registered.load(std::memory_order_relaxed))
            return;
        s_lastFrameBeginMs.store(steadyMilliseconds(), std::memory_order_release);
        if (!s_hasCommands.load(std::memory_order_acquire))
            return;
        std::vector<Command> commands;
        {
            std::scoped_lock lock(s_commandsLock);
            commands.swap(s_commands);
            s_hasCommands.store(false, std::memory_order_release);
        }
        for (const auto& command : commands)
            execute(command);
    }

    bool isArmed() noexcept { return s_armed.load(std::memory_order_relaxed); }
}
