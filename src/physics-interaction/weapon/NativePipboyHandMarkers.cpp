#include "physics-interaction/weapon/NativePipboyHandMarkers.h"

#include "physics-interaction/weapon/NativeHandEquipment.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/PhysicsLog.h"
#include "RE/Scaleform/GFx/GFx_Player.h"
#include <REL/Relocation.h>
#include <Windows.h>
#include <algorithm>
#include <array>
#include <atomic>
#include <string>

namespace rock::native_pipboy_hand_markers
{
    namespace
    {
        // FO4VR 1.2.72, raw disassembly (ghidra_fo4vr):
        // 140C21450 fills one Pip-Boy inventory entry. Members it sets reach
        //   the AS3 entry, but an extra member added there does not (runtime
        //   12 Oct 22:24), so the hand is read from native members only.
        // 140C1E9A0 computes "equipState". For a weapon (form type 0x2B) it
        //   counts the stack's equip-slot bits (flags +0x24 & 7): one bit
        //   without bit 0 gives 2 (140C1EA7C), bit 0 gives 3 or 4
        //   (140C1EAB7/140C1EAC3), several bits give 4 (140C1EA70).
        //   Equipment index 1 sets a non-zero slot bit, so 2 is index 1 and
        //   3/4 is index 0.
        // "formID" is the entry's base form (TESForm+0x14 at 140C2147B).
        // InvListEntry.SetEntryText (Pipboy_InvPage.swf, ABC bytecode) draws
        //   the equipped square only as EquipIcon_mc.visible = equipState > 0
        //   and sets EquipIcon_mc.transform.colorTransform to the row's
        //   SelectedColorTransform (multipliers 1, offsets -255: black) or
        //   UnselectedColorTransform (identity), both built in its <init>.
        //   A white letter inside the icon therefore turns black on the
        //   highlighted row and keeps the square's screen green otherwise.
        constexpr const char* kLetterName = "rockHandLetter";
        constexpr double kLetterColor = 0xFFFFFF;
        constexpr double kLetterScale = 2.0; // Letter font size per square height.

        using Value = RE::Scaleform::GFx::Value;

        bool s_installed = false;

        template <std::size_t N>
        bool matches(std::uintptr_t rva, const std::array<std::uint8_t, N>& expected) noexcept
        {
            std::array<std::uint8_t, N> actual{};
            return native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(REL::Offset(rva).address()),
                actual.data(), actual.size()) && actual == expected;
        }

        // ---- Inventory list presentation (Pip-Boy movie thread) ----------
        //
        // Each visible InvListEntry's EquipIcon_mc gets one TextField child
        // holding the hand letter; while the entry has a hand, the icon's
        // own square children are hidden. The icon itself stays under the
        // native script, so the letter shows exactly when the square would
        // and takes the same selection tint. Nothing else in the row moves.

        std::atomic<std::uint32_t> s_reports{0};

        // Deepest point the per-frame walk reached in the current movie,
        // logged once per new depth so a stop in the chain names itself.
        enum class Stage : std::uint8_t { None, Menu, InventoryPage, List, VisibleRow, EquippedRow, HandResolved, LetterShown };
        constexpr const char* kStageNames[]{ "none", "menu", "inventory-page", "list", "visible-row", "equipped-row",
            "hand-resolved", "letter-shown" };
        Stage s_deepest = Stage::None; // Movie thread only.
        void reach(Stage stage, std::uint32_t a = 0, std::uint32_t b = 0)
        {
            if (stage <= s_deepest) return;
            s_deepest = stage;
            ROCK_LOG_INFO(Weapon, "Pip-Boy hand letters reached stage={} a={} b={}", kStageNames[static_cast<int>(stage)], a, b);
        }

        // AS3 int and uint members arrive as Int/UInt values, not Number.
        bool isNumeric(const Value& value)
        {
            return value.IsNumber() || value.IsInt() || value.IsUInt();
        }
        double numeric(const Value& value)
        {
            return value.IsNumber() ? value.GetNumber() : value.IsInt() ? static_cast<double>(value.GetInt()) :
                value.IsUInt() ? static_cast<double>(value.GetUInt()) : 0.0;
        }

        // AS3 Array or Vector length; GFx's array size does not cover Vectors.
        std::uint32_t lengthOf(const Value& value)
        {
            Value length;
            return value.IsObject() && value.GetMember("length", &length) && isNumeric(length) && numeric(length) > 0 ?
                static_cast<std::uint32_t>(numeric(length)) : 0u;
        }

        bool member(const Value& object, const char* name, Value& out) noexcept
        {
            return object.IsObject() && object.GetMember(name, &out);
        }

        std::string text(const Value& value)
        {
            return value.IsString() && value.GetString() ? value.GetString() : std::string{};
        }

        // The icon's square children: every display child except the letter.
        void showSquare(Value& icon, bool visible)
        {
            Value count;
            if (!member(icon, "numChildren", count) || !isNumeric(count)) return;
            const auto n = static_cast<int>(numeric(count));
            for (int i = 0; i < n && i < 16; ++i) {
                Value child, name;
                const Value index(static_cast<double>(i));
                if (!icon.Invoke("getChildAt", &child, &index, 1) || !child.IsObject()) continue;
                if (member(child, "name", name) && text(name) == kLetterName) continue;
                child.SetMember("visible", Value(visible));
            }
        }

        bool letterField(RE::Scaleform::GFx::Movie* movie, Value& entry, Value& icon, Value& out)
        {
            const Value name(kLetterName);
            if (icon.Invoke("getChildByName", &out, &name, 1) && out.IsObject()) return true;
            // First use on this row: size the letter to the square and copy
            // the row's own font so it renders through the menu's font set.
            Value bounds, field, format, embed, x, y, w, h;
            if (!icon.Invoke("getBounds", &bounds, &icon, 1) || !member(bounds, "x", x) || !member(bounds, "y", y) ||
                !member(bounds, "width", w) || !member(bounds, "height", h) || !isNumeric(x) || !isNumeric(h) ||
                !member(entry, "textField", field) || !field.IsObject() || !field.Invoke("getTextFormat", &format) ||
                !format.IsObject()) return false;
            movie->CreateObject(&out, "flash.text.TextField");
            if (!out.IsObject()) return false;
            const double squareHeight = numeric(h);
            const double size = std::max(8.0, squareHeight * kLetterScale);
            // The row's format carries the row's text colour, which is not
            // the square's; the icon transform alone must decide the colour.
            format.SetMember("size", Value(size));
            format.SetMember("bold", Value(true));
            format.SetMember("color", Value(kLetterColor));
            out.SetMember("name", Value(kLetterName));
            if (member(field, "embedFonts", embed)) out.SetMember("embedFonts", embed);
            out.SetMember("defaultTextFormat", format);
            out.SetMember("selectable", Value(false));
            out.SetMember("mouseEnabled", Value(false));
            out.SetMember("autoSize", Value("left"));
            out.SetMember("text", Value("R"));
            Value fieldWidth, fieldHeight;
            if (member(out, "width", fieldWidth) && member(out, "height", fieldHeight) && isNumeric(fieldWidth) && isNumeric(fieldHeight)) {
                out.SetMember("x", Value(numeric(x) + numeric(w) * 0.5 - numeric(fieldWidth) * 0.5));
                out.SetMember("y", Value(numeric(y) + squareHeight * 0.5 - numeric(fieldHeight) * 0.5));
            }
            if (!icon.Invoke("addChild", nullptr, &out, 1)) return false;
            if (s_reports.fetch_add(1, std::memory_order_relaxed) < 2)
                ROCK_LOG_INFO(Weapon, "Pip-Boy hand letter created square=({:.1f},{:.1f} {:.1f}x{:.1f}) letterSize={:.1f}",
                    numeric(x), numeric(y), isNumeric(w) ? numeric(w) : 0.0, squareHeight, size);
            return true;
        }

        // An equipped weapon row's hand. The native slot from equipState
        // picks the equipment index; formID must match the weapon worn
        // there, so other equipped weapon-type items (grenades) get none.
        // Two identical weapons share a formID and still resolve by slot.
        std::array<std::uint64_t, 8> s_reportedRows{}; // Movie thread only.
        std::uint32_t s_rowReports = 0;
        bool firstReport(std::uint32_t form, int slot)
        {
            const auto key = (static_cast<std::uint64_t>(form) << 8) | static_cast<std::uint64_t>(slot + 1);
            for (std::uint32_t i = 0; i < s_rowReports; ++i) if (s_reportedRows[i] == key) return false;
            if (s_rowReports >= s_reportedRows.size()) return false;
            s_reportedRows[s_rowReports++] = key;
            return true;
        }
        native_hand_equipment::WornHand rowHand(const Value& data)
        {
            Value equip, form;
            if (!member(data, "equipState", equip) || !isNumeric(equip)) return native_hand_equipment::WornHand::None;
            const auto state = static_cast<int>(numeric(equip));
            const int slot = state == 2 ? 1 : state == 3 || state == 4 ? 0 : -1;
            if (slot < 0 || !member(data, "formID", form) || !isNumeric(form)) return native_hand_equipment::WornHand::None;
            reach(Stage::EquippedRow);
            const auto formID = static_cast<std::uint32_t>(numeric(form));
            const auto hand = native_hand_equipment::wornRowHand(formID, slot);
            if (firstReport(formID, slot)) {
                ROCK_LOG_INFO(Weapon, "Pip-Boy hand letters row form={:08X} equipState={} slot={} hand={}",
                    formID, state, slot, hand == native_hand_equipment::WornHand::Left ? "L" :
                        hand == native_hand_equipment::WornHand::Right ? "R" : "none");
            }
            return hand;
        }

        // GetMember reaches only public AS3 traits: PipboyMenu.PageA/GetPage
        // and BSScrollingList.uiNumListItems are private/protected (ABC trait
        // namespaces), so the walk uses CurrentPage and numListItems.
        const char* s_reportedStop = nullptr; // Movie thread only.
        void stop(const char* step)
        {
            if (s_reportedStop == step) return;
            s_reportedStop = step;
            ROCK_LOG_INFO(Weapon, "Pip-Boy hand letters stopped at {} after stage={}", step,
                kStageNames[static_cast<int>(s_deepest)]);
        }

        void updateList(RE::Scaleform::GFx::Movie* movie, Value& list)
        {
            Value entries, count;
            if (!member(list, "entryList", entries) || !entries.IsObject()) return stop("entryList");
            if (!member(list, "numListItems", count) || !isNumeric(count)) return stop("numListItems");
            const auto clips = static_cast<int>(numeric(count));
            const auto size = lengthOf(entries);
            reach(Stage::List, size, static_cast<std::uint32_t>(clips));
            for (int c = 0; c < clips && c < 64; ++c) {
                Value clip, visible, index, icon;
                const Value clipIndex(static_cast<double>(c));
                if (!list.Invoke("GetClipByIndex", &clip, &clipIndex, 1) || !clip.IsObject() ||
                    !member(clip, "EquipIcon_mc", icon) || !icon.IsObject()) continue;
                std::string hand;
                if (member(clip, "visible", visible) && visible.IsBoolean() && visible.GetBoolean() &&
                    member(clip, "itemIndex", index) && isNumeric(index) && numeric(index) >= 0 && numeric(index) < size) {
                    reach(Stage::VisibleRow, static_cast<std::uint32_t>(c));
                    Value data;
                    const auto key = std::to_string(static_cast<std::uint32_t>(numeric(index)));
                    if (entries.GetMember(key.c_str(), &data) && data.IsObject()) {
                        const auto worn = rowHand(data);
                        hand = worn == native_hand_equipment::WornHand::Left ? "L" : worn == native_hand_equipment::WornHand::Right ? "R" : "";
                        if (!hand.empty()) reach(Stage::HandResolved);
                    }
                }
                Value letter;
                const Value name(kLetterName);
                const bool exists = icon.Invoke("getChildByName", &letter, &name, 1) && letter.IsObject();
                if (hand.empty()) {
                    if (!exists) continue;
                    letter.SetMember("visible", Value(false));
                    showSquare(icon, true);
                    continue;
                }
                if (!exists && !letterField(movie, clip, icon, letter)) continue;
                Value current;
                if (!member(letter, "text", current) || text(current) != hand) letter.SetMember("text", Value(hand.c_str()));
                letter.SetMember("visible", Value(true));
                showSquare(icon, false);
                reach(Stage::LetterShown);
            }
        }

        void updateMovie(RE::Scaleform::GFx::Movie* movie)
        {
            Value menu, page, list, doll;
            if (!movie->GetVariable(&menu, "root.Menu_mc") || !menu.IsObject()) return stop("root.Menu_mc");
            reach(Stage::Menu);
            if (!member(menu, "CurrentPage", page) || !page.IsObject()) return stop("CurrentPage");
            // Only the inventory page has the paper doll; other tabs are skipped.
            if (!member(page, "PaperDoll_mc", doll) || !doll.IsObject()) return;
            if (!member(page, "List_mc", list) || !list.IsObject()) return stop("List_mc");
            reach(Stage::InventoryPage);
            updateList(movie, list);
        }

        // PipboyMenu::AdvanceMovie (vtable 142D58418 slot 4 = 140BA09D0; it
        // ends in IMenu::AdvanceMovie 142198D40, which advances the movie
        // held at menu+0x40). The letters are refreshed on the movie's own
        // thread right after its frame scripts ran.
        constexpr std::uintptr_t kAdvanceSlot = 0x2D58438;
        constexpr std::uintptr_t kAdvanceOriginal = 0xBA09D0;
        constexpr std::ptrdiff_t kMenuMovieOffset = 0x40;
        using Advance = void (*)(void*, float, std::uint64_t);
        Advance s_advance = nullptr;
        void* s_lastMovie = nullptr; // Movie thread only.

        void refresh(RE::Scaleform::GFx::Movie* movie, const char* source)
        {
            if (movie != s_lastMovie) {
                s_lastMovie = movie;
                s_deepest = Stage::None;
                s_reportedStop = nullptr;
                s_rowReports = 0;
                ROCK_LOG_INFO(Weapon, "Pip-Boy hand letters first refresh movie={:#x} source={}",
                    reinterpret_cast<std::uintptr_t>(movie), source);
            }
            updateMovie(movie);
        }

        void advanceMovie(void* menu, float delta, std::uint64_t time)
        {
            s_advance(menu, delta, time);
            try {
                RE::Scaleform::GFx::Movie* movie = nullptr;
                if (!native_memory::tryReadField(menu, kMenuMovieOffset, movie) || !movie) return;
                refresh(movie, "advance");
            } catch (...) {
                OutputDebugStringA("ROCK: Pip-Boy hand letters failed\n");
            }
        }
    }

    void onPipboyCodeCall(void* menu) noexcept
    {
        if (!s_installed) return;
        try {
            RE::Scaleform::GFx::Movie* movie = nullptr;
            if (native_memory::tryReadField(menu, kMenuMovieOffset, movie) && movie) refresh(movie, "code-call");
        } catch (...) {
            OutputDebugStringA("ROCK: Pip-Boy hand letters failed\n");
        }
    }

    bool install() noexcept
    {
        std::uintptr_t current = 0;
        const auto slot = REL::Offset(kAdvanceSlot).address();
        if (!native_memory::tryReadValue(reinterpret_cast<const std::uintptr_t*>(slot), current) ||
            current != REL::Offset(kAdvanceOriginal).address() ||
            !matches(0xBA0A95, std::to_array<std::uint8_t>({0xE8,0xA6,0x82,0x5F,0x01}))) {
            ROCK_LOG_WARN(Weapon, "Pip-Boy hand letters unavailable: PipboyMenu advance slot differs");
            return false;
        }
        s_advance = reinterpret_cast<Advance>(current);
        REL::safe_write(slot, reinterpret_cast<std::uintptr_t>(&advanceMovie));
        s_installed = true;
        ROCK_LOG_INFO(Weapon, "Pip-Boy hand markers installed");
        return true;
    }
}
