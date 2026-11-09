#include "config/SettingMetadata.h"
#include "setup-wizard/SetupWizardPolicy.h"
#include "setup-wizard/SteamVrBindingPolicy.h"

#include <cstdio>

namespace
{
    namespace wizard = rock::setup_wizard_policy;
    namespace fb = rock::function_binding_policy;
    using fb::Function;
    using fb::Source;

    int g_failures = 0;

    void expect(const char* label, bool value)
    {
        if (!value) {
            std::printf("FAILED: %s\n", label);
            ++g_failures;
        }
    }

    // The defaults are what every player starts from: the wizard must not warn about them.
    void defaultsHaveNoConflict()
    {
        const auto found = wizard::conflicts(fb::homeBindings());
        for (std::size_t i = 0; i < found.count; ++i)
            std::printf("default conflict: %s / %s\n", fb::functionKey(found.items[i].a).data(),
                fb::functionKey(found.items[i].b).data());
        expect("default bindings conflict", found.count == 0);
    }

    void movedBindingsConflict()
    {
        auto bindings = fb::homeBindings();
        bindings[static_cast<std::size_t>(Function::Vats)] = *fb::parseBinding("primary tap a");
        const auto clashes = wizard::clashesWith(bindings, Function::Vats, bindings[static_cast<std::size_t>(Function::Vats)]);
        expect("a tap of A clashes with Reload's tap", (clashes & fb::functionBit(Function::Reload)) != 0);
        expect("Scope's hold does not clash with a tap", (clashes & fb::functionBit(Function::Scope)) == 0);
        const auto chord = *fb::parseBinding("primary tap a +primary grip");
        expect("a chord does not clash with the plain button",
            wizard::clashesWith(bindings, Function::Vats, chord) == 0);
        const auto left = *fb::parseBinding("left tap a");
        expect("firing and left can meet", wizard::clashesWith(bindings, Function::Vats, left) != 0);
        const auto offhand = *fb::parseBinding("offhand tap trigger");
        expect("support and offhand can meet",
            (wizard::clashesWith(bindings, Function::Vats, offhand) & fb::functionBit(Function::SupportTrigger)) != 0);
    }

    [[nodiscard]] wizard::CaptureSample sample(std::uint32_t left, std::uint32_t right)
    {
        return wizard::CaptureSample{ { left, right } };
    }

    [[nodiscard]] constexpr std::uint32_t bit(Source source) { return 1u << static_cast<unsigned>(source); }

    void captureNeedsAFreshPress()
    {
        wizard::Capture capture;
        capture.update(sample(0, bit(Source::Trigger)));
        capture.update(sample(0, 0));
        expect("a held button from the click that started the capture is ignored", !capture.result());
        capture.update(sample(bit(Source::Grip), 0));
        capture.update(sample(0, 0));
        expect("captured the left grip", capture.result() && capture.result()->control.isLeft &&
            capture.result()->control.source == Source::Grip && !capture.result()->modifier);
    }

    // A Touch grip press also reports the grip click; a stick click also lights its pad region.
    void capturePhysicalCompanions()
    {
        wizard::Capture capture;
        capture.update(sample(0, 0));
        capture.update(sample(bit(Source::Grip) | bit(Source::GripClick), 0));
        capture.update(sample(0, 0));
        expect("grip with its click is the grip", capture.result() && capture.result()->control.source == Source::Grip &&
            !capture.result()->modifier);
        capture.start();
        capture.update(sample(0, 0));
        capture.update(sample(0, bit(Source::Stick) | bit(Source::PadCenter)));
        capture.update(sample(0, 0));
        expect("stick click with its region is the stick click",
            capture.result() && capture.result()->control.source == Source::Stick && !capture.result()->modifier);
    }

    void captureChord()
    {
        wizard::Capture capture;
        capture.update(sample(0, 0));
        capture.update(sample(0, bit(Source::Grip)));
        capture.update(sample(0, bit(Source::Grip) | bit(Source::UpperFace)));
        capture.update(sample(0, bit(Source::UpperFace)));
        capture.update(sample(0, 0));
        const auto& result = capture.result();
        expect("chord captured", result && result->control.source == Source::UpperFace && result->modifier &&
            result->modifier->source == Source::Grip);
    }

    // Every option page names a real setting whose help yields its choices.
    void optionPagesComeFromTheCatalog()
    {
        for (const auto& page : wizard::kOptionPages) {
            const rock::config::SettingMetadata* found = nullptr;
            for (const auto& setting : rock::config::kSettingMetadata)
                if (setting.table == page.section && setting.key == page.key) found = &setting;
            expect("option page names a setting", found != nullptr);
            if (!found) continue;
            // The store knows each type; here a numbered mode shows in its help.
            const bool integer = found->description.find("\n1 = ") != std::string_view::npos;
            const auto kind = integer ? wizard::ValueKind::Integer
                : page.key == "DefaultHand" ? wizard::ValueKind::Text
                                            : wizard::ValueKind::Boolean;
            const auto choices = wizard::choicesFor(page, kind, found->description);
            if (choices.count < 2) std::printf("page %s.%s: %zu choices\n", page.section.data(), page.key.data(), choices.count);
            expect("option page has choices", choices.count >= 2);
        }
        const auto equip = wizard::numberedChoices(
            "1 = Trigger Equip with no drop (game default). 2 = Trigger Equip with Toggle Drop.\n"
            "3 = Trigger Equip with Auto Drop. 4 = Auto Equip (always Auto Drop).\nTrigger Equip: pull");
        expect("modes over two lines", equip.count == 4 && equip.items[3].value == "4" &&
            equip.items[3].label == "Auto Equip (always Auto Drop)");
    }

    // ROCK offers its SteamVR binding only over SteamVR's default and never replaces the player's choice.
    void steamVrBindingOffer()
    {
        namespace sv = rock::steamvr_binding_policy;
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
    defaultsHaveNoConflict();
    movedBindingsConflict();
    captureNeedsAFreshPress();
    capturePhysicalCompanions();
    captureChord();
    optionPagesComeFromTheCatalog();
    steamVrBindingOffer();
    if (g_failures == 0) std::printf("ROCK setup wizard tests passed\n");
    return g_failures == 0 ? 0 : 1;
}
