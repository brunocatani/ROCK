#include "config/SettingMetadata.h"
#include "setup-wizard/SetupWizardPolicy.h"
#include "setup-wizard/SteamVrBindingPolicy.h"

#include <cstdio>
#include <fstream>
#include <map>
#include <string>

namespace
{
    namespace wizard = rock::setup_wizard_policy;
    namespace sv = rock::steamvr_binding_policy;

    int g_failures = 0;

    void expect(const char* label, bool value)
    {
        if (!value) {
            std::printf("FAILED: %s\n", label);
            ++g_failures;
        }
    }

    // "Table.Key" -> value, from the shipped example file (every key at its default).
    [[nodiscard]] std::map<std::string, std::string> exampleDefaults()
    {
        std::map<std::string, std::string> values;
        std::ifstream file(std::string(ROCK_CONFIG_REFERENCE_DIR) + "/ROCK_example.toml");
        std::string line, table;
        while (std::getline(file, line)) {
            if (!line.empty() && line.back() == '\r') line.pop_back();
            if (line.size() > 2 && line.front() == '[') {
                table = line.substr(1, line.find(']') - 1);
                continue;
            }
            const auto equals = line.find(" = ");
            if (line.empty() || line.front() == '#' || equals == std::string::npos) continue;
            auto value = line.substr(equals + 3);
            if (value.size() >= 2 && value.front() == '"' && value.back() == '"') value = value.substr(1, value.size() - 2);
            values[table + "." + line.substr(0, equals)] = value;
        }
        return values;
    }

    // Each option is a player setting the catalog knows, with distinct choices that
    // include the value ROCK ships, on a page that exists.
    void optionsComeFromTheCatalog()
    {
        const auto defaults = exampleDefaults();
        expect("example file read", !defaults.empty());
        for (const auto& option : wizard::kOptions) {
            const rock::config::SettingMetadata* found = nullptr;
            for (const auto& setting : rock::config::kSettingMetadata)
                if (setting.table == option.section && setting.key == option.key) found = &setting;
            const std::string name = std::string(option.section) + "." + std::string(option.key);
            if (!found) std::printf("not in the catalog: %s\n", name.c_str());
            expect("option names a setting", found != nullptr);
            expect("option is a player setting", found && found->group == rock::api::configuration::Group::Consumer);
            expect("option is on a page", option.page < wizard::kPages.size());
            expect("option has two choices or more", option.choiceCount() >= 2);
            for (std::size_t i = 0; i < option.choiceCount(); ++i)
                for (std::size_t j = i + 1; j < option.choiceCount(); ++j)
                    expect("choices differ", option.choices[i].value != option.choices[j].value);
            const auto shipped = defaults.find(name);
            if (shipped == defaults.end() || !option.find(shipped->second))
                std::printf("default of %s is not a choice\n", name.c_str());
            expect("the default is a choice", shipped != defaults.end() && option.find(shipped->second));
        }
        for (std::size_t page = 0; page < wizard::kPages.size(); ++page) {
            bool used = false;
            for (const auto& option : wizard::kOptions) used |= option.page == page;
            expect("every page has an option", used);
        }
    }

    // ROCK's binding presents Index controllers and wands as Touch: the setting tells them apart.
    void detection()
    {
        using sv::Headset;
        using sv::Status;
        const auto offer = Status::Offer, rock = Status::RockActive;
        expect("Index", sv::detectHeadset(true, "knuckles", offer, offer) == Headset::Index);
        expect("wands", sv::detectHeadset(true, "vive_controller", offer, offer) == Headset::ViveWands);
        expect("Touch", sv::detectHeadset(true, "oculus_touch", offer, offer) == Headset::NoBindingNeeded);
        expect("Index under ROCK's binding", sv::detectHeadset(true, "oculus_touch", rock, offer) == Headset::Index);
        expect("wands under ROCK's binding", sv::detectHeadset(true, "oculus_touch", offer, rock) == Headset::ViveWands);
        expect("Steam Frame", sv::detectHeadset(true, "frame_controller", offer, offer) == Headset::NoBindingNeeded);
        expect("something else", sv::detectHeadset(true, "holographic_controller", offer, offer) == Headset::Other);
        expect("nothing on", !sv::detectHeadset(true, "", offer, offer));
        expect("OpenComposite", !sv::detectHeadset(false, "knuckles", offer, offer));
    }

    // Finishing installs for the player's choice, or the detected controllers while ROCK
    // still offers its binding, and only over SteamVR's default.
    void finishingPlansTheControllers()
    {
        using sv::Headset;
        using sv::Status;
        using wizard::BindingAction;
        const auto offer = Status::Offer;
        auto plan = wizard::planControllers(Headset::Index, Headset::Index, offer, offer, true);
        expect("chosen Index installs", plan.action == BindingAction::Install && plan.controller == sv::Controller::Index);
        plan = wizard::planControllers(std::nullopt, Headset::ViveWands, offer, offer, true);
        expect("no choice installs for the detected wands",
            plan.action == BindingAction::Install && plan.controller == sv::Controller::ViveWand);
        plan = wizard::planControllers(std::nullopt, Headset::Index, offer, offer, false);
        expect("no choice and the offer off installs nothing", plan.action == BindingAction::None);
        plan = wizard::planControllers(Headset::Index, std::nullopt, offer, offer, false);
        expect("a choice installs even with the offer off", plan.action == BindingAction::Install);
        plan = wizard::planControllers(Headset::Index, Headset::Index, Status::PlayersOwn, offer, true);
        expect("the player's own binding stays", plan.action == BindingAction::None);
        plan = wizard::planControllers(Headset::Index, Headset::Index, Status::RockActive, offer, true);
        expect("ROCK's binding already set", plan.action == BindingAction::None);
        plan = wizard::planControllers(Headset::NoBindingNeeded, Headset::Index, offer, offer, true);
        expect("choosing other controllers over offered ones stops the offer", plan.action == BindingAction::StopOffering);
        plan = wizard::planControllers(Headset::NoBindingNeeded, Headset::NoBindingNeeded, offer, offer, true);
        expect("nothing to stop", plan.action == BindingAction::None);
    }

    // ROCK offers its SteamVR binding only over SteamVR's default and never replaces the player's choice.
    void steamVrBindingOffer()
    {
        const auto index = sv::controllerFor("knuckles");
        const auto vive = sv::controllerFor("vive_controller");
        expect("Index and Vive wands have a binding", index && vive);
        expect("Touch and Frame need none", !sv::controllerFor("oculus_touch") && !sv::controllerFor("frame_controller"));
        expect("default binding is offered", sv::classify(true, index, "") == sv::Status::Offer);
        expect("not offered outside SteamVR", sv::classify(false, index, "") == sv::Status::NotApplicable);
        expect("not offered for other controllers", sv::classify(true, std::nullopt, "") == sv::Status::NotApplicable);
        expect("a workshop binding is the player's",
            sv::classify(true, index, "vr-input-workshop://3174776330") == sv::Status::PlayersOwn);
        expect("a personal binding is the player's",
            sv::classify(true, index, "file:///C:/Users/a/Documents/steamvr/input/steam.app.611660_knuckles.json") ==
                sv::Status::PlayersOwn);
        expect("ROCK's file in any folder is ROCK's",
            sv::classify(true, index, "file:///D:/Games/My Games/ROCK/SteamVR/Legacy_Bindings_Knuckles_Rock.json") ==
                sv::Status::RockActive);
        expect("the Vive file is not the Index binding",
            sv::classify(true, index, "file:///C:/x/legacy_bindings_vive_controller_rock.json") == sv::Status::PlayersOwn);
        expect("a name that only ends like ROCK's is not ROCK's",
            sv::classify(true, index, "file:///C:/x/mylegacy_bindings_knuckles_rock.json") == sv::Status::PlayersOwn);
        expect("SteamVR's key for the Index binding", sv::settingKey("knuckles") == "knuckles_250820_CurrentURL_legacy");
        expect("file URL with forward slashes", sv::fileUrl("C:\\Users\\a b\\x.json") == "file:///C:/Users/a b/x.json");
    }
}

int main()
{
    optionsComeFromTheCatalog();
    detection();
    finishingPlansTheControllers();
    steamVrBindingOffer();
    if (g_failures == 0) std::printf("ROCK setup wizard tests passed\n");
    return g_failures == 0 ? 0 : 1;
}
