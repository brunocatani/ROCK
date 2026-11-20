// A guard of its own, not #pragma once: a mod can have two copies of this file, its own and its framework's
#ifndef MODS_CONFIG_FO4_API_V1_H
#define MODS_CONFIG_FO4_API_V1_H

#include <Windows.h>
#include <algorithm>
#include <cstdint>
#include <functional>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

// ----------------------------------------------------------------------------------------
// Mods Config FO4 VR's native API, for the DLL of a mod that has a config spec, or a config UI of its own.
// Copy this whole file into your project AS IS.
//
// Two layers. What passes between the DLLs is two interfaces of virtual functions: ModsConfigFO4ApiV1, which Mods
// Config VR implements, and IClient, which a client implements and registers once. The menu asks the client by name
// for its functions, states and options. Their arguments are plain: whole numbers, float, bool, enums, UTF-8 texts
// that end with a zero, and pointers. No std:: type crosses: each DLL has its own heap, and its own layout of them.
// What the host passes is valid during the call, so copy what you keep. What you pass the host copies before the
// call returns.
// ModsConfigFO4Client, at the end of this file, is an IClient over your lambdas, compiled into your DLL. Use it, and
// the interfaces only for what it does not wrap. Every call, either way, is on the game's main thread.
//
// A client method that throws or faults cuts the client off until the game is restarted, and its mod works as one
// with no client.
//
// EXAMPLE USAGE:
//
// {
//     using namespace mods_config::api;
//     auto& modsConfig = ModsConfigFO4Client::get();
//
//     // at any time, also before it connects
//     modsConfig.addFunction("calibrate", [](const Params& params) {
//         return FunctionResult{ .done = true, .message = std::format("Done: {:.2f} m.", measure(params.getInt("samples", 10))) };
//     });
//     modsConfig.addState("calibrating", [] { return std::optional<Value>(isCalibrating()); });
//     modsConfig.addState("pipboyType", [] { return std::optional<Value>(pipboyType()); }, [](const Value& value) { setPipboyType(std::get<int>(value)); });
//     modsConfig.subscribeToChanges([](const Change& change) { previewOrRestore(change.path, change.value, change.discard); });
//     modsConfig.subscribeToSaves([](const Saved&) { endPreviewsAndReload(); });
//     // each frame: light the zone of a group of the config spec with "id": "grabZone" while the ray is on its rows
//     const bool highlight = modsConfig.isPointedAt("grabZone");
//
//     // from F4SE's kPostLoad on, when every plugin's DLL is loaded: 0 while Mods Config FO4 VR is there
//     const int err = modsConfig.connect("MyMod");
// }

namespace mods_config::api
{
#define MODS_CONFIG_FO4_CALL __cdecl

    // The version of this header: what V1 has. A method is only added at the end of an interface, and a struct only
    // grows at its end, so a host of a later version takes a client of this one.
    inline constexpr std::uint32_t MODS_CONFIG_FO4_API_VERSION = 1;

    // the host's DLL, and the function it exports that gives the API
    inline constexpr auto MODS_CONFIG_FO4_DLL = "ModsConfigFO4VR.dll";
    inline constexpr auto MODS_CONFIG_FO4_GET_API = "ModsConfigFO4API_GetApiV1";

    // Mods Config FO4 VR's F4SE plugin name, the sender of its F4SE messages, which a plugin registers a listener for from
    // kPostLoad on, as F4SE takes a listener of a sender that is loaded
    inline constexpr auto MODS_CONFIG_FO4_F4SE_NAME = "ModsConfigFO4VR";

    // The type of the F4SE message Mods Config FO4 VR sends after a Save of a config spec wrote something, to each F4SE
    // plugin of its `f4sePluginRequirements`. Its data is the config spec's `modName`, UTF-8 text that ends with a
    // zero, and is valid during the listener's call. It does not say what changed: a client's `onSaved` does.
    inline constexpr std::uint32_t MODS_CONFIG_FO4_SAVED_MESSAGE = 1;

    /**
     * The type of a value, as a setting of a config spec has its `valueType`.
     */
    enum class ValueType : std::uint32_t
    {
        Bool,
        Int, // signed 32-bit
        Float, // 32-bit
        String
    };

    /**
     * A value: its type, and the field of its type. A String is its text.
     */
    struct SettingValue
    {
        ValueType type = ValueType::String;
        const char* text = nullptr;
        bool boolValue = false;
        std::int32_t intValue = 0;
        float floatValue = 0;
    };

    /**
     * The format of a file of the client's config spec.
     */
    enum class FileFormat : std::uint32_t
    {
        Ini,
        Toml
    };

    /**
     * A file of the client's config spec: its name in the config spec's `files`, its format, and its path, absolute.
     */
    struct FileView
    {
        const char* name;
        FileFormat format;
        const char* path;
    };

    /**
     * Where a setting's value is kept. In a file of the config spec: the file, and the names that lead to the value, its
     * section and its key in an INI file, and in a TOML or a JSON file the tables it is in and its key. In the game, with
     * no file: a global's form, or a script property's form, its script, null for the form's first, and its name. A
     * form is its plugin and its id in it, as `MyMod.esp|800`. In a state of the client, with no file and no form: the
     * state's name, and the client applies the value as it is told it.
     */
    struct StorageView
    {
        const FileView* file;
        const char* const* path;
        std::uint32_t pathCount;
        const char* form;
        const char* scriptName;
        const char* propertyName;
        const char* clientState;
    };

    /**
     * A setting of the client's config spec.
     */
    struct SettingView
    {
        const char* id; // unique in the config spec
        ValueType valueType;

        // where its value is kept, for a change of it, and no file as the options of a choice are asked
        StorageView storage;
    };

    /**
     * What a change of a setting is: a Preview of a valid pending value, or a Discard back to the value the editing
     * session started with.
     */
    enum class ChangePhase : std::uint32_t
    {
        Preview,
        Discard
    };

    /**
     * A change of a `live` setting of the client's config spec, while it waits for a Save, which the client shows in the
     * game: the value it was sent last for the setting, or the setting's baseline, and the new one, in the setting's
     * valueType. A setting kept in the game is in it already: the menu writes it before it tells the client, which can
     * do what more it needs. One kept in a state of the client is applied by the client. A Discard comes as the player
     * sets the value back, discards it, or leaves the mod with it. A Save sends none: its SaveBatch has the setting, and
     * the file or the game has the value then.
     */
    struct SettingChange
    {
        const SettingView* setting;
        SettingValue previousValue;
        SettingValue value;
        ChangePhase phase;
    };

    /**
     * A setting a Save wrote: its value before the Save, and the value written, in the setting's valueType.
     */
    struct SavedSetting
    {
        const SettingView* setting;
        SettingValue baselineValue;
        SettingValue value;
    };

    /**
     * What a Save of the client's config spec wrote, which the client is told once, after the action of each setting
     * ran: every setting it wrote, of any `applies`, in the order of the config spec, the files it wrote, and how many
     * Saves of the config spec wrote something in this run of the game, this one with them. A `live` setting the client
     * shows a preview of is in it, and its preview ends: the file or the game has the value. A setting kept in a state of
     * the client is written by this alone, and the client applies it.
     */
    struct SaveBatch
    {
        const SavedSetting* settings;
        std::uint32_t settingCount;
        const FileView* writtenFiles;
        std::uint32_t fileCount;
        std::uint64_t revision;
    };

    /**
     * A parameter of a function: its name, and its value in its own JSON type in the config spec.
     */
    struct NamedValue
    {
        const char* name;
        SettingValue value;
    };

    /**
     * Whether the menu is closed, open, or open and minimized.
     */
    enum class MenuState : std::uint32_t
    {
        Closed,
        Open,
        Minimized
    };

    /**
     * What happened in the menu: it opened or closed, which every client is told, the client's mod or a page of its
     * config spec that has an `id` was shown or left, and the ray came on an item of its config spec or left it
     * (PointedAt). A minimized menu is still open. A client of an older header can be told a type it does not know,
     * which it leaves alone.
     */
    enum class MenuEventType : std::uint32_t
    {
        MenuOpened,
        MenuClosed,
        ModShown,
        ModLeft,
        PageShown,
        PageLeft,
        PointedAt
    };

    struct MenuEvent
    {
        MenuEventType type;
        const char* pageId; // the page's `id` in the config spec, for PageShown and PageLeft, and null for the others

        // For PointedAt, what the ray is on: the id of its row, then the id of each group the row is in, innermost
        // first, as the config spec has them. A setting's id, given or made; a section's, an activation sphere's and
        // a button's own `id`, and a sphere's section with none. An item with no id is left out, and none is nothing.
        // Read them only for PointedAt: a menu of an older header has no such fields.
        const char* const* pointedIds;
        std::uint32_t pointedCount;
    };

    /**
     * A call of a function of the client, which the host passes it: its parameters, and how the function gives the
     * message of its result.
     */
    class IFunctionCall
    {
    public:
        virtual std::uint32_t paramCount() = 0;
        virtual const NamedValue* param(std::uint32_t index) = 0;

        // What the help line shows of the result: after the info mark, or after the invalid mark when the function
        // returns false. A text can be a `$` token of the game's language.
        virtual void setMessage(const char* text) = 0;

    protected:
        ~IFunctionCall() = default;
    };

    /**
     * Where the client gives the options of a choice, which the host passes it.
     */
    class IOptions
    {
    public:
        // Adds an option, after those added before. Its value is of the setting's valueType, and an option of another
        // type is left out. A null text shows the value. A text can be a `$` token of the game's language.
        virtual void add(const SettingValue* value, const char* text) = 0;

    protected:
        ~IOptions() = default;
    };

    /**
     * What a client implements, which the menu asks by name. It is registered once, and lives for the whole run of the
     * game. Each method answers at once.
     */
    class IClient
    {
    public:
        // whether it has a function of a name, which a `CallNativeFunction` of its config spec runs: a button of one it
        // does not have is disabled
        virtual bool hasFunction(const char* name) = 0;

        // runs a function, with the button's params: true when it did what it does. Work that takes longer goes on after
        virtual bool runFunction(const char* name, IFunctionCall* call) = 0;

        // answers a state, which a `clientState` condition asks once a frame while the mod is shown, and a setting kept
        // in it as the mod is selected, in value: false while it is not available. It changes nothing. A String's text
        // stays valid until a state is asked next
        virtual bool answerState(const char* name, SettingValue* value) = 0;

        // gives the options of a choice with `listFromClientCallback`, as the mod is selected: false for one it has none of
        virtual bool giveOptions(const SettingView* setting, IOptions* options) = 0;

        // What happens in the menu, as it happens. PointedAt is told as what the ray is on changes, so the mod shows in
        // the world what the item sets: it stays across the gaps between rows and while the row's editor or Adjust in
        // world is open, and is none as the ray leaves the rows, the menu leaves the mod, closes, or minimizes for
        // another UI
        virtual void onEvent(const MenuEvent* event) = 0;

        // a change of a `live` setting, which the client shows in the game until it is saved or
        // discarded
        virtual void onChange(const SettingChange* change) = 0;

        // what a Save of its config spec wrote, once a Save
        virtual void onSaved(const SaveBatch* batch) = 0;

    protected:
        ~IClient() = default;
    };

    /**
     * Mods Config FO4 VR's side of the API, which its DLL exports.
     */
    class ModsConfigFO4ApiV1
    {
    public:
        /**
         * The version of the host's API: what V1 has. Check it before you call what a later version added.
         */
        virtual std::uint32_t getVersion() const = 0;

        /**
         * Mods Config FO4 VR's own version, as "1.0.0".
         */
        virtual const char* getModVersion() const = 0;

        /**
         * Adds an entry of the client to the quick launch section of the menu's front page: its text, its help, which
         * the help line shows, the text of its button, null for Open, and the function of the client a press runs, which
         * opens the mod's own UI. A client can add an entry for each of its functions. A text can be a `$` token of the
         * game's language. False for a client that is not registered, and for a function that has an entry already,
         * which keeps the first.
         */
        virtual bool addQuickLaunch(IClient* client, const char* text, const char* help, const char* buttonText, const char* functionName) const = 0;

        /**
         * Registers a client, once, under its mod's name, which is matched to the config spec whose `modName` is that
         * name without regard to case, with the version of its header. False for a name that has a client already, which
         * keeps the first.
         */
        virtual bool registerClient(const char* name, std::uint32_t version, IClient* client) const = 0;

        /**
         * Whether the menu is closed, open, or minimized. Any DLL can ask, with or without a client.
         */
        virtual MenuState getMenuState() const = 0;

        /**
         * Opens the menu at a mod, and at a page of its config spec by its `id`: at the mod's front page with no page,
         * and at the menu's front page with no mod. A minimized menu is shown, and one that shows a mod with unsaved
         * changes asks first. False for a mod or a page that is not there, with nothing opened.
         */
        virtual bool openMenu(const char* modName, const char* pageId) const = 0;

        /**
         * Minimizes the menu, as a function of the client opens the mod's own UI. Its editing session stays.
         */
        virtual void minimizeMenu() const = 0;

        /**
         * Shows the menu again where it was, as the mod's own UI closes, and reads the shown mod's values again: only
         * while it is minimized, so a menu the player showed or closed is left as it is.
         */
        virtual void restoreMenu() const = 0;

        /**
         * Reads the values of the client's mod again while it is shown, after the mod changed them: a clean setting
         * takes its new value, and a pending one stays the player's.
         */
        virtual void reloadValues(IClient* client) const = 0;

        /**
         * Sets the pending value of a setting of the client's config spec by its id, in its valueType, as the player's
         * edit in the menu: a mod's own UI hands its edits back with it, and the menu's Save writes them. A color, a
         * transform and a binding are the text of their TOML table, or their line in an INI file. A value that is the
         * baseline makes the setting clean, and a `live` one is told back with `onChange`, as a player's edit is. False,
         * with nothing changed and why in the menu's log, unless the menu, shown or minimized, shows the client's mod, from
         * inside the client's `onChange` or `onSaved`, and for a value it refuses: of an id it has no setting of, of a
         * read-only setting, of another type, or no value of the setting, as one outside its limits.
         */
        virtual bool setPendingValue(IClient* client, const char* settingId, const SettingValue* value) const = 0;

        /**
         * Finds Mods Config FO4 VR's DLL and gets its API. Returns 0 when the API is there, and else:
         * 1 for a game with no Mods Config FO4 VR, 2 for one that does not export the API, 3 for one that gives none,
         * and 4 for an API older than minVersion.
         */
        [[nodiscard]] static int initialize(const std::uint32_t minVersion = MODS_CONFIG_FO4_API_VERSION)
        {
            if (inst) {
                return 0;
            }
            const auto dll = GetModuleHandleA(MODS_CONFIG_FO4_DLL);
            if (!dll) {
                return 1;
            }
            const auto getApi = reinterpret_cast<const ModsConfigFO4ApiV1*(MODS_CONFIG_FO4_CALL*)()>(GetProcAddress(dll, MODS_CONFIG_FO4_GET_API));
            if (!getApi) {
                return 2;
            }
            const auto api = getApi();
            if (!api) {
                return 3;
            }
            if (api->getVersion() < minVersion) {
                return 4;
            }
            inst = api;
            return 0;
        }

        /**
         * The API, after initialize returned 0.
         */
        inline static const ModsConfigFO4ApiV1* inst = nullptr;

    protected:
        ~ModsConfigFO4ApiV1() = default;
    };

    // ----------------------------------------------------------------------------------------
    // The C++ side of the API, compiled into the client's DLL: nothing of it crosses to the host.

    /**
     * A value that passes to and from the menu: a parameter's, an option's or a state's.
     */
    using Value = std::variant<bool, int, float, std::string>;

    /**
     * The named parameters a button of the config spec runs a function with, each in its own JSON type.
     */
    class Params
    {
    public:
        explicit Params(IFunctionCall* call)
            : _call(call)
        {}

        /**
         * A parameter by its name, in its own type. None for one the button does not give.
         */
        std::optional<Value> get(const std::string_view name) const
        {
            for (std::uint32_t i = 0; i < _call->paramCount(); ++i) {
                const auto* param = _call->param(i);
                if (!param || !param->name || name != param->name) {
                    continue;
                }
                switch (param->value.type) {
                case ValueType::Bool:
                    return param->value.boolValue;
                case ValueType::Int:
                    return static_cast<int>(param->value.intValue);
                case ValueType::Float:
                    return param->value.floatValue;
                case ValueType::String:
                    break;
                }
                return std::string(param->value.text ? param->value.text : "");
            }
            return std::nullopt;
        }

        bool getBool(const std::string_view name, const bool fallback = false) const
        {
            const auto value = get(name);
            const auto* flag = value ? std::get_if<bool>(&*value) : nullptr;
            return flag ? *flag : fallback;
        }

        /**
         * A whole number, or the fallback for one that is not given or no number. A number is cut to its whole part.
         */
        int getInt(const std::string_view name, const int fallback = 0) const
        {
            const auto value = get(name);
            if (const auto* number = value ? std::get_if<int>(&*value) : nullptr) {
                return *number;
            }
            const auto* real = value ? std::get_if<float>(&*value) : nullptr;
            return real ? static_cast<int>(*real) : fallback;
        }

        /**
         * A number, a whole one too, or the fallback for one that is not given or no number.
         */
        float getFloat(const std::string_view name, const float fallback = 0.0f) const
        {
            const auto value = get(name);
            if (const auto* real = value ? std::get_if<float>(&*value) : nullptr) {
                return *real;
            }
            const auto* number = value ? std::get_if<int>(&*value) : nullptr;
            return number ? static_cast<float>(*number) : fallback;
        }

        std::string getString(const std::string_view name, std::string fallback = {}) const
        {
            const auto value = get(name);
            const auto* text = value ? std::get_if<std::string>(&*value) : nullptr;
            return text ? *text : std::move(fallback);
        }

    private:
        IFunctionCall* _call;
    };

    /**
     * What a function did: whether it did what it does, and the message the help line shows of it, which can be none.
     */
    struct FunctionResult
    {
        bool done = true;
        std::string message;
    };

    /**
     * An option of a choice: its value, in the setting's type, and what the player sees, which is the value with none.
     */
    struct Option
    {
        Value value;
        std::string text;
    };

    /**
     * An entry of the quick launch section of the menu's front page, which runs a function of the mod.
     */
    struct QuickLaunch
    {
        std::string text;
        std::string help;

        // the text on its button: empty for Open
        std::string buttonText;
    };

    /**
     * A change of a `live` setting of the config spec, as the C++ client hands it to the mod: the setting's id, where it
     * is kept, the value the mod was sent last for the setting and the new one, and whether it is a Discard back to the
     * value the editing session started with. A setting kept in a file has its file's name, format and absolute path,
     * and the names that lead to the value in it. One kept in the game has no file, and its form, as `MyMod.esp|800`,
     * with the script and the name of a property: the game has the value already. One kept in a state of the mod has
     * the state's name alone, and the state's setter has applied the value. A setting a Save wrote is one too, with its
     * value before the Save and the value written. Its texts are valid during the handler: copy what you keep.
     * `handedOver` marks the change a value the mod's own setPendingValue is handing over makes in that call, a
     * preview, or a Discard for its baseline: the mod has the value already, so a handler only records it.
     */
    struct Change
    {
        std::string_view id;
        std::string_view fileName;
        FileFormat format = FileFormat::Toml;
        std::string_view filePath;
        std::vector<std::string_view> path;
        std::string_view form;
        std::string_view scriptName;
        std::string_view propertyName;
        std::string_view clientState;
        Value previous;
        Value value;
        bool discard = false;
        bool handedOver = false;
    };

    /**
     * A file a Save wrote: its name in the config spec's `files`, its format and its absolute path.
     */
    struct SavedFile
    {
        std::string_view name;
        FileFormat format = FileFormat::Toml;
        std::string_view path;
    };

    /**
     * What a Save of the config spec wrote, as the C++ client hands it to the mod: each setting it wrote, of any
     * `applies`, in the order of the config spec, the files it wrote, and how many Saves of the config spec wrote
     * something in this run of the game, this one with them. A `live` setting the mod shows a preview of is in it, and
     * its preview ends: the file or the game has the value. Its texts are valid during the handler: copy what you keep.
     */
    struct Saved
    {
        std::vector<Change> settings;
        std::vector<SavedFile> files;
        std::uint64_t revision = 0;
    };

    using ClientFunction = std::function<FunctionResult(const Params& params)>;
    using ClientState = std::function<std::optional<Value>()>;
    using StateSetter = std::function<void(const Value& value)>;
    using ClientOptions = std::function<std::vector<Option>()>;
    using MenuEventHandler = std::function<void(MenuEventType event, std::string_view pageId)>;
    using PointingHandler = std::function<void(const std::vector<std::string>& ids)>;
    using ChangeHandler = std::function<void(const Change& change)>;
    using SaveHandler = std::function<void(const Saved& saved)>;

    /**
     * The client of the mod's DLL, one in the DLL: an IClient over the mod's lambdas. It connects under the mod's name,
     * which is matched to the config spec whose `modName` it is, and answers the menu with what the mod adds, at any
     * time: its functions, states, options of choices, quick launch entries and handlers of the menu's events. While
     * Mods Config FO4 VR is not installed nothing is called.
     */
    class ModsConfigFO4Client final : public IClient
    {
    public:
        static ModsConfigFO4Client& get()
        {
            static ModsConfigFO4Client client;
            return client;
        }

        ModsConfigFO4Client(const ModsConfigFO4Client&) = delete;
        ModsConfigFO4Client& operator=(const ModsConfigFO4Client&) = delete;

        /**
         * Connects to the menu under the mod's name, once, from F4SE's kPostLoad on, with the quick launch entries added
         * before. With the minVersion of 1 it connects to any menu of V1, so a client of a newer header
         * keeps working with an older menu: what a later version added is used after a check of `apiVersion()`, which
         * this class does too. A mod that does not work at all with an older menu passes the version it needs, and a
         * menu whose API is older is not connected to. Returns 0 when it is connected, initialize's code while the API is
         * not there or is older, and 5 when the menu refused the name, which has a client already: its log says why.
         */
        int connect(const std::string& name, const std::uint32_t minVersion = 1)
        {
            if (_api) {
                return 0;
            }
            if (const int err = ModsConfigFO4ApiV1::initialize(minVersion); err != 0) {
                return err;
            }
            if (!ModsConfigFO4ApiV1::inst->registerClient(name.c_str(), MODS_CONFIG_FO4_API_VERSION, this)) {
                return 5;
            }
            _api = ModsConfigFO4ApiV1::inst;
            _name = name;
            for (const auto& [function, entry] : _quickLaunch) {
                sendQuickLaunch(function, entry);
            }
            return 0;
        }

        // the API, for what this class does not wrap: null until it is connected
        const ModsConfigFO4ApiV1* api() const
        {
            return _api;
        }

        bool isConnected() const
        {
            return _api != nullptr;
        }

        /**
         * The version of the connected menu's API, and 0 until it is connected: a mod uses what a later version added
         * only while it is that version or later, as `if (client.apiVersion() >= 2)`.
         */
        std::uint32_t apiVersion() const
        {
            return _api ? _api->getVersion() : 0;
        }

        /**
         * Adds a function, which a `CallNativeFunction` of the config spec runs by its name, with its params. It returns
         * at once. False for a name it has already, which keeps the first.
         */
        bool addFunction(std::string name, ClientFunction function)
        {
            return _functions.try_emplace(std::move(name), std::move(function)).second;
        }

        /**
         * Adds a state, which a `clientState` condition of the config spec asks by its name, once a frame while the mod
         * is shown: it answers at once and changes nothing. None while it is not available. A setting of the config spec
         * can be kept in it, which the menu reads as the mod is selected and gives the setter, which applies the value in
         * the mod: each preview of a `live` one, its Discard back to the baseline, and the value a Save wrote. It cannot
         * refuse a value, and is called again with one it has. A state with no setter is a `readOnly` setting's.
         */
        bool addState(std::string name, ClientState state, StateSetter setter = {})
        {
            const auto added = _states.try_emplace(name, std::move(state)).second;
            if (added && setter) {
                _stateSetters.emplace(std::move(name), std::move(setter));
            }
            return added;
        }

        /**
         * Adds the options of a choice with `listFromClientCallback`, by the setting's id, which the menu asks as the mod
         * is selected.
         */
        bool addOptions(std::string settingId, ClientOptions options)
        {
            return _options.try_emplace(std::move(settingId), std::move(options)).second;
        }

        /**
         * Adds an entry of the quick launch section of the menu's front page and the function a press on it runs, by
         * its name, which a `CallNativeFunction` of the config spec can run too. The function's message is told in the
         * help line, in red when it failed. A function that opens a UI of the mod's own minimizes the menu, and the mod
         * restores it as the UI closes. False for a name it has a function of already, with nothing added.
         */
        bool addQuickLaunch(std::string name, QuickLaunch entry, ClientFunction function)
        {
            if (!addFunction(name, std::move(function))) {
                return false;
            }
            if (_api) {
                return sendQuickLaunch(name, entry);
            }
            _quickLaunch.emplace_back(std::move(name), std::move(entry));
            return true;
        }

        /**
         * Minimizes the menu, with its editing session kept, as the mod opens a UI of its own. Nothing while it is not
         * connected.
         */
        void minimizeMenu() const
        {
            if (_api) {
                _api->minimizeMenu();
            }
        }

        /**
         * Shows the menu again, with the shown mod's values read again. Nothing while it is not connected, or while the
         * menu is not minimized, as the player showed or closed it.
         */
        void restoreMenu() const
        {
            if (_api) {
                _api->restoreMenu();
            }
        }

        /**
         * Reads the mod's values again while it is shown in the menu, after the mod changed them.
         */
        void reloadValues()
        {
            if (_api) {
                _api->reloadValues(this);
            }
        }

        /**
         * Sets the pending value of a setting of the config spec by its id, as the player's edit in the menu: a UI of
         * the mod's own hands its edits back with it, and saves nothing itself. A `live` one comes back to the change
         * handlers during this call, as a preview, or a Discard for its baseline, marked `handedOver`: the mod applies
         * the value itself before it hands it over. False while it is not connected, from inside a handler of the
         * changes or the Saves, and for a value the menu refuses, whose log says why.
         */
        bool setPendingValue(const std::string_view id, const Value& value)
        {
            if (!_api) {
                return false;
            }
            const std::string settingId(id);
            std::string text;
            const auto setting = toSettingValue(value, text);

            // the id the change of this call comes back with, while the call lasts
            _handingOverId = settingId;
            const bool taken = _api->setPendingValue(this, settingId.c_str(), &setting);
            _handingOverId.clear();
            return taken;
        }

        /**
         * Opens the menu at the mod, at a page of its config spec by its `id`, or at its front page with none. False
         * while it is not connected, and for a page the config spec does not have.
         */
        bool openMenu(const std::string_view pageId = {}) const
        {
            if (!_api) {
                return false;
            }
            const std::string page(pageId);
            return _api->openMenu(_name.c_str(), page.empty() ? nullptr : page.c_str());
        }

        /**
         * Calls a handler with each event of the menu: it opened and closed, and the mod and a page of its config spec
         * that has an `id` were shown and left.
         */
        void subscribeToMenuEvents(MenuEventHandler handler)
        {
            _menuEventHandlers.push_back(std::move(handler));
        }

        /**
         * Calls a handler as what the ray is on in the menu changes: the id of the row of the config spec it is on, then
         * the id of each group the row is in, innermost first, and none as the ray leaves the mod's rows. The menu-event
         * handlers are not called with it.
         */
        void subscribeToPointing(PointingHandler handler)
        {
            _pointingHandlers.push_back(std::move(handler));
        }

        /**
         * Whether the ray is on an item of the config spec by its id, or on a row inside it, as `isPointedAt("bodyGrab")`
         * for an activation sphere: a mod that draws each frame asks it there.
         */
        bool isPointedAt(const std::string_view id) const
        {
            return std::ranges::find(_pointed, id) != _pointed.end();
        }

        /**
         * What the ray is on: the id of its row, then of each group the row is in, innermost first. Empty for none.
         */
        const std::vector<std::string>& pointedIds() const
        {
            return _pointed;
        }

        /**
         * Calls a handler with each change of a `live` setting of the config spec: a preview of a pending value, and a
         * discard back to the value the session started with.
         */
        void subscribeToChanges(ChangeHandler handler)
        {
            _changeHandlers.push_back(std::move(handler));
        }

        /**
         * Calls a handler with what each Save of the config spec wrote, once a Save, after the action of each setting ran.
         */
        void subscribeToSaves(SaveHandler handler)
        {
            _saveHandlers.push_back(std::move(handler));
        }

        bool isMenuOpen() const
        {
            return _menuOpen;
        }

        /**
         * The `id` of the page of the config spec that is shown, and none while none with an `id` is.
         */
        std::optional<std::string_view> shownPage() const
        {
            return _shownPage ? std::optional<std::string_view>(*_shownPage) : std::nullopt;
        }

    private:
        ModsConfigFO4Client() = default;
        ~ModsConfigFO4Client() = default;

        /**
         * Gives the connected menu a quick launch entry of a function.
         */
        bool sendQuickLaunch(const std::string& function, const QuickLaunch& entry) const
        {
            return _api->addQuickLaunch(const_cast<ModsConfigFO4Client*>(this),
                entry.text.c_str(),
                entry.help.c_str(),
                entry.buttonText.empty() ? nullptr : entry.buttonText.c_str(),
                function.c_str());
        }

        /**
         * A value as the API passes it, whose text points into the text given: it is valid while that is.
         */
        static SettingValue toSettingValue(const Value& value, std::string& text)
        {
            SettingValue setting{ .type = ValueType::String };
            if (const auto* flag = std::get_if<bool>(&value)) {
                setting.type = ValueType::Bool;
                setting.boolValue = *flag;
            } else if (const auto* number = std::get_if<int>(&value)) {
                setting.type = ValueType::Int;
                setting.intValue = *number;
            } else if (const auto* real = std::get_if<float>(&value)) {
                setting.type = ValueType::Float;
                setting.floatValue = *real;
            } else {
                text = std::get<std::string>(value);
                setting.text = text.c_str();
            }
            return setting;
        }

        /**
         * A value the API passes as the C++ client's: a String is its text, which is copied.
         */
        static Value fromSettingValue(const SettingValue& value)
        {
            switch (value.type) {
            case ValueType::Bool:
                return value.boolValue;
            case ValueType::Int:
                return static_cast<int>(value.intValue);
            case ValueType::Float:
                return value.floatValue;
            case ValueType::String:
                break;
            }
            return std::string(value.text ? value.text : "");
        }

        bool hasFunction(const char* name) override
        {
            return _functions.contains(std::string_view(name));
        }

        bool runFunction(const char* name, IFunctionCall* call) override
        {
            const auto found = _functions.find(std::string_view(name));
            if (found == _functions.end()) {
                return false;
            }
            const auto result = found->second(Params(call));
            if (!result.message.empty()) {
                call->setMessage(result.message.c_str());
            }
            return result.done;
        }

        bool answerState(const char* name, SettingValue* value) override
        {
            const auto found = _states.find(std::string_view(name));
            const auto answer = found != _states.end() ? found->second() : std::nullopt;
            if (!answer) {
                return false;
            }
            *value = toSettingValue(*answer, _stateText);
            return true;
        }

        bool giveOptions(const SettingView* setting, IOptions* options) override
        {
            const auto found = _options.find(std::string_view(setting->id));
            if (found == _options.end()) {
                return false;
            }
            for (const auto& option : found->second()) {
                std::string text;
                const auto value = toSettingValue(option.value, text);
                options->add(&value, option.text.empty() ? nullptr : option.text.c_str());
            }
            return true;
        }

        /**
         * Keeps what an event of the menu tells, whether it is open, the page that is shown and what the ray is on, and
         * calls the handlers: PointedAt the pointing handlers, and any other the menu-event handlers.
         */
        void onEvent(const MenuEvent* event) override
        {
            if (event->type == MenuEventType::PointedAt) {
                _pointed.clear();
                for (std::uint32_t i = 0; i < event->pointedCount; ++i) {
                    _pointed.emplace_back(event->pointedIds[i] ? event->pointedIds[i] : "");
                }
                for (const auto& handler : _pointingHandlers) {
                    handler(_pointed);
                }
                return;
            }
            switch (event->type) {
            case MenuEventType::MenuOpened:
                _menuOpen = true;
                break;
            case MenuEventType::MenuClosed:
                _menuOpen = false;
                break;
            case MenuEventType::PageShown:
                _shownPage = event->pageId ? event->pageId : "";
                break;
            case MenuEventType::PageLeft:
            case MenuEventType::ModLeft:
                _shownPage.reset();
                break;
            case MenuEventType::ModShown:
            case MenuEventType::PointedAt:
                break;
            }
            const std::string_view page = event->pageId ? event->pageId : "";
            for (const auto& handler : _menuEventHandlers) {
                handler(event->type, page);
            }
        }

        /**
         * A change of a setting as the C++ client has it, whose texts point into the setting's view.
         */
        static Change toChange(const SettingView& setting, const SettingValue& previous, const SettingValue& value, const bool discard)
        {
            const auto& storage = setting.storage;
            const auto* file = storage.file;
            Change change{ .id = setting.id ? setting.id : "",
                .fileName = file && file->name ? file->name : "",
                .format = file ? file->format : FileFormat::Toml,
                .filePath = file && file->path ? file->path : "",
                .path = {},
                .form = storage.form ? storage.form : "",
                .scriptName = storage.scriptName ? storage.scriptName : "",
                .propertyName = storage.propertyName ? storage.propertyName : "",
                .clientState = storage.clientState ? storage.clientState : "",
                .previous = fromSettingValue(previous),
                .value = fromSettingValue(value),
                .discard = discard };
            for (std::uint32_t i = 0; i < storage.pathCount; ++i) {
                change.path.emplace_back(storage.path[i] ? storage.path[i] : "");
            }
            return change;
        }

        /**
         * Calls the setter of the state a setting is kept in with the value of its change, which a Discard has the
         * baseline as. Nothing for a setting kept elsewhere, and for a state with no setter.
         */
        void applyState(const Change& change) const
        {
            if (change.clientState.empty()) {
                return;
            }
            if (const auto setter = _stateSetters.find(change.clientState); setter != _stateSetters.end()) {
                setter->second(change.value);
            }
        }

        /**
         * Calls the handlers of the changes with a change, as the C++ client has it, after the setter of a state. The
         * change of the setting setPendingValue is handing over is marked `handedOver`.
         */
        void onChange(const SettingChange* change) override
        {
            auto mine = toChange(*change->setting, change->previousValue, change->value, change->phase == ChangePhase::Discard);
            mine.handedOver = !_handingOverId.empty() && mine.id == _handingOverId;
            applyState(mine);
            for (const auto& handler : _changeHandlers) {
                handler(mine);
            }
        }

        /**
         * Calls the handlers of the Saves with what a Save wrote, as the C++ client has it, after the setter of each
         * state it wrote.
         */
        void onSaved(const SaveBatch* batch) override
        {
            Saved saved{ .revision = batch->revision };
            for (std::uint32_t i = 0; i < batch->settingCount; ++i) {
                const auto& setting = batch->settings[i];
                saved.settings.push_back(toChange(*setting.setting, setting.baselineValue, setting.value, false));
                applyState(saved.settings.back());
            }
            for (std::uint32_t i = 0; i < batch->fileCount; ++i) {
                const auto& file = batch->writtenFiles[i];
                saved.files.push_back({ .name = file.name ? file.name : "", .format = file.format, .path = file.path ? file.path : "" });
            }
            for (const auto& handler : _saveHandlers) {
                handler(saved);
            }
        }

        const ModsConfigFO4ApiV1* _api = nullptr;

        // the name it connected under, the mod's
        std::string _name;

        // what the mod added, by name
        std::map<std::string, ClientFunction, std::less<>> _functions;
        std::map<std::string, ClientState, std::less<>> _states;
        std::map<std::string, StateSetter, std::less<>> _stateSetters;
        std::map<std::string, ClientOptions, std::less<>> _options;

        // the quick launch entries added before it connected, by their functions
        std::vector<std::pair<std::string, QuickLaunch>> _quickLaunch;

        std::vector<MenuEventHandler> _menuEventHandlers;
        std::vector<PointingHandler> _pointingHandlers;
        std::vector<ChangeHandler> _changeHandlers;
        std::vector<SaveHandler> _saveHandlers;

        // the id of the setting setPendingValue hands over while it runs, empty otherwise
        std::string _handingOverId;

        // the text of a string state, which stays until a state is asked next: the menu copies it after the answer
        std::string _stateText;

        // what the menu's events told: whether it is open, and the page of the config spec that is shown
        bool _menuOpen = false;
        std::optional<std::string> _shownPage;

        // what the ray is on, as PointedAt told it last
        std::vector<std::string> _pointed;
    };
}

#endif
