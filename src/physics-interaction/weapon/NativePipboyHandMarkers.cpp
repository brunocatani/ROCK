#include "physics-interaction/weapon/NativePipboyHandMarkers.h"

#include "physics-interaction/weapon/NativeHandEquipment.h"
#include "physics-interaction/native/EntryTrampolineHook.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/PhysicsLog.h"
#include "RE/Bethesda/BGSInventoryItem.h"
#include "RE/Bethesda/BSExtraData.h"
#include "RE/Bethesda/TESForms.h"
#include "RE/Scaleform/GFx/GFx_Player.h"
#include <REL/Relocation.h>
#include <Windows.h>
#include <algorithm>
#include <array>
#include <atomic>
#include <cstring>
#include <string>

namespace rock::native_pipboy_hand_markers
{
    namespace
    {
        // FO4VR 1.2.72, raw disassembly (ghidra_fo4vr):
        // 140C21450 fills one Pip-Boy inventory entry object (sole caller
        //   140C20E00). Its members reach Pipboy_InvPage.swf as the list
        //   entry data, e.g. "text" through 140C28B80(object, key 145ACB0A8,
        //   BSFixedStringCS) at 140C2152B.
        // 140C45F00, used by every member setter, first removes an existing
        //   member of the same name (140C45870), so re-setting replaces it.
        // InvListEntry.SetEntryText (Pipboy_InvPage.swf, ABC bytecode) draws
        //   the equipped square only as EquipIcon_mc.visible = equipState > 0
        //   and tints it with the row's selection colour transform.
        constexpr std::uintptr_t kPopulate = 0xC21450;
        constexpr std::uintptr_t kSetString = 0xC28B80;
        constexpr std::uintptr_t kMakeString = 0x1BC2050;
        constexpr std::uintptr_t kFreeString = 0x1BC20F0;
        constexpr const char* kHandMember = "rockHand";
        constexpr const char* kLetterName = "rockHandLetter";

        using Stack = RE::BGSInventoryItem::Stack;
        using Populate = void (*)(void*, RE::BGSInventoryItem*, Stack*, void*, void*, std::uint32_t);
        using SetString = void* (*)(void*, const void*, void*);
        using MakeString = void* (*)(void*, const char*);
        using FreeString = void (*)(void*);
        using Value = RE::Scaleform::GFx::Value;

        Populate s_populate = nullptr;
        bool s_installed = false;

        template <std::size_t N>
        bool matches(std::uintptr_t rva, const std::array<std::uint8_t, N>& expected) noexcept
        {
            std::array<std::uint8_t, N> actual{};
            return native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(REL::Offset(rva).address()),
                actual.data(), actual.size()) && actual == expected;
        }

        template <class T>
        T native(std::uintptr_t rva) noexcept { return reinterpret_cast<T>(REL::Offset(rva).address()); }

        int wornIndexOf(RE::BGSInventoryItem* item, const Stack* stack) noexcept
        {
            if (!stack || !stack->IsEquipped()) return -1;
            std::uintptr_t instance = 0;
            if (const auto* extra = stack->extra.get()) {
                if (const auto* data = extra->GetByType<RE::ExtraInstanceData>())
                    instance = reinterpret_cast<std::uintptr_t>(data->data.get());
            }
            return native_hand_equipment::wornWeaponIndex(item->object->formID, instance);
        }

        // Interned member name, created once and kept for the process.
        const void* handKey() noexcept
        {
            alignas(8) static std::array<std::byte, 8> key{};
            static const bool made = (native<MakeString>(kMakeString)(key.data(), kHandMember), true);
            (void)made;
            return key.data();
        }

        // Every weapon entry carries "rockHand": the equipment index wearing
        // it ("0", "1") or empty, so a reused entry object never keeps an
        // earlier one. The letter is resolved from the index every frame, so
        // a hand move without re-equip shows at once. Name, sorting and
        // equipState stay native.
        void populate(void* self, RE::BGSInventoryItem* item, Stack* stack, void* object, void* flags, std::uint32_t mode)
        {
            s_populate(self, item, stack, object, flags, mode);
            if (!object || !item || !item->object || !item->object->IsWeapon()) return;
            try {
                const auto index = wornIndexOf(item, stack);
                const char* letter = index == 0 ? "0" : index == 1 ? "1" : "";
                alignas(8) std::array<std::byte, 8> value{};
                native<MakeString>(kMakeString)(value.data(), letter);
                native<SetString>(kSetString)(object, handKey(), value.data());
                native<FreeString>(kFreeString)(value.data());
            } catch (...) {
                OutputDebugStringA("ROCK: Pip-Boy hand marker data failed\n");
            }
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
        enum class Stage : std::uint8_t { None, Menu, InventoryPage, List, VisibleRow, RowData, HandMember, HandResolved, LetterShown };
        constexpr const char* kStageNames[]{ "none", "menu", "inventory-page", "list", "visible-row", "row-data",
            "hand-member", "hand-resolved", "letter-shown" };
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
            format.SetMember("size", Value(std::max(8.0, squareHeight * 1.4)));
            format.SetMember("bold", Value(true));
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
                    numeric(x), numeric(y), isNumeric(w) ? numeric(w) : 0.0, squareHeight, std::max(8.0, squareHeight * 1.4));
            return true;
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
                    Value data, marker;
                    const auto key = std::to_string(static_cast<std::uint32_t>(numeric(index)));
                    if (entries.GetMember(key.c_str(), &data) && data.IsObject()) reach(Stage::RowData);
                    if (data.IsObject() && member(data, kHandMember, marker)) {
                        const auto slot = text(marker);
                        if (!slot.empty()) reach(Stage::HandMember);
                        const auto worn = slot == "0" ? native_hand_equipment::wornIndexHand(0) :
                            slot == "1" ? native_hand_equipment::wornIndexHand(1) : native_hand_equipment::WornHand::None;
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
        // Helpers called directly, and the populate body that proves they are
        // the member path: the "text" key LEA at 140C2152B (rel32 resolves to
        // 145ACB0A8) feeding the string setter call.
        const bool helpers =
            matches(kSetString, std::to_array<std::uint8_t>({0x48,0x89,0x5C,0x24,0x08,0x48,0x89,0x6C,0x24,0x10})) &&
            matches(kMakeString, std::to_array<std::uint8_t>({0x40,0x53,0x48,0x83,0xEC,0x20,0x48,0x8B,0xD9})) &&
            matches(kFreeString, std::to_array<std::uint8_t>({0xE9,0xEB,0x07,0x00,0x00})) &&
            matches(0xC2152B, std::to_array<std::uint8_t>({0x48,0x8D,0x15,0x76,0x9B,0xEA,0x04}));
        if (!helpers) {
            ROCK_LOG_WARN(Weapon, "Pip-Boy hand markers unavailable: native inventory entry layout differs");
            return false;
        }
        constexpr auto populatePrefix = std::to_array<std::uint8_t>({
            0x48,0x89,0x5C,0x24,0x18,0x48,0x89,0x4C,0x24,0x08,0x55,0x56,0x57,0x41,0x54});
        void* original = nullptr;
        if (!entry_trampoline_hook::install("Pip-Boy hand marker entry", kPopulate, populatePrefix.data(),
                populatePrefix.size(), reinterpret_cast<void*>(&populate), original)) return false;
        s_populate = reinterpret_cast<Populate>(original);
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
