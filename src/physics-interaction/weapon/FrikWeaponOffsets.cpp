#include "physics-interaction/weapon/FrikWeaponOffsets.h"

#include "RockConfig.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/weapon/HoldingGripAdapter.h"
#include "rock_support/Fo4VrRuntime.h"
#include "rock_support/ResourceUtils.h"

#include "RE/Bethesda/TESForms.h"
#include "RE/NetImmerse/NiNode.h"

#include <thomasmonkman-filewatch/FileWatch.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <vector>

namespace rock::frik_weapon_offsets
{
    namespace
    {
        namespace policy = frik_weapon_offset_policy;

        struct CacheSlot
        {
            // Identity witnesses only; never dereferenced.
            const RE::TESObjectWEAP* weapon{ nullptr };
            const RE::NiAVObject* root{ nullptr };
            std::uint64_t revision{ 0 };
            bool powerArmor{ false };
            bool used{ false };
            std::optional<Override> result{};
        };

        // Frame-thread state.
        std::filesystem::path s_directory;
        policy::EntryMap s_entries;
        std::uint64_t s_revision{ 1 };
        std::array<CacheSlot, 4> s_cache{};
        std::size_t s_nextSlot{ 0 };
        std::unique_ptr<filewatch::FileWatch<std::string>> s_watch;

        // Published by the watcher thread, consumed on the frame thread.
        std::atomic<bool> s_reloadPending{ false };
        std::atomic<std::int64_t> s_lastEventTicks{ 0 };

        [[nodiscard]] std::int64_t ticks() noexcept
        {
            return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
        }

        void readDirectory()
        {
            policy::EntryMap entries;
            std::size_t files = 0;
            std::error_code error;
            if (std::filesystem::is_directory(s_directory, error)) {
                std::vector<std::filesystem::path> paths;
                for (const auto& entry : std::filesystem::directory_iterator(s_directory, error)) {
                    if (entry.is_regular_file(error)) {
                        paths.push_back(entry.path());
                    }
                }
                std::sort(paths.begin(), paths.end());
                for (const auto& path : paths) {
                    try {
                        std::ifstream in(path);
                        const auto json = nlohmann::json::parse(in);
                        for (auto& [key, pose] : policy::readFile(json)) {
                            entries.insert_or_assign(key, pose);
                        }
                        ++files;
                    } catch (const std::exception& e) {
                        ROCK_LOG_WARN(Weapon, "FRIK weapon offset file {} skipped: {}", path.filename().string(), e.what());
                    }
                }
            }
            s_entries = std::move(entries);
            ++s_revision;
            ROCK_LOG_INFO(Weapon, "FRIK user weapon offsets: {} entries from {} files in {}", s_entries.size(), files, s_directory.string());
        }

        void startWatch()
        {
            std::error_code error;
            if (s_watch || !std::filesystem::is_directory(s_directory, error)) {
                return;
            }
            try {
                s_watch = std::make_unique<filewatch::FileWatch<std::string>>(s_directory.string(), [](const std::string&, filewatch::Event) {
                    s_lastEventTicks.store(ticks(), std::memory_order_release);
                    s_reloadPending.store(true, std::memory_order_release);
                });
            } catch (const std::exception& e) {
                ROCK_LOG_WARN(Weapon, "FRIK weapon offset folder cannot be watched ({}); changes apply after a restart", e.what());
            }
        }

        [[nodiscard]] std::string_view gripStockName(const RE::NiAVObject* root)
        {
            auto* mount = root ? f4vr::findNode(const_cast<RE::NiAVObject*>(root), "P-Grip") : nullptr;
            auto* child = mount ? f4vr::getFirstChild(mount) : nullptr;
            return child && child->name.c_str() ? std::string_view{ child->name.c_str() } : std::string_view{};
        }
    }

    void load()
    {
        s_directory = resources::getPathInDocuments(R"(\My Games\Fallout4VR\FRIK_Config\Weapons_Offsets)");
        readDirectory();
        startWatch();
    }

    void processPendingReload()
    {
        // The watcher publishes only a flag; editor and FRIK save bursts are
        // coalesced here without sleeping on either thread.
        if (!s_reloadPending.load(std::memory_order_acquire) || ticks() - s_lastEventTicks.load(std::memory_order_acquire) < 200) {
            return;
        }
        if (s_reloadPending.exchange(false, std::memory_order_acq_rel)) {
            readDirectory();
        }
    }

    std::optional<Override> find(const RE::TESObjectWEAP* weapon, const RE::NiAVObject* weaponRoot, const bool inPowerArmor)
    {
        // Opt-in: without the setting no file affects any weapon.
        if (!g_rockConfig.rockFrikUserWeaponOffsets || !weapon || s_entries.empty()) {
            return std::nullopt;
        }
        for (const auto& slot : s_cache) {
            if (slot.used && slot.weapon == weapon && slot.root == weaponRoot && slot.powerArmor == inPowerArmor && slot.revision == s_revision) {
                return slot.result;
            }
        }
        const auto name = RE::TESFullName::GetFullName(*weapon, false);
        const auto key = policy::weaponKey(name, gripStockName(weaponRoot));
        auto& slot = s_cache[s_nextSlot];
        s_nextSlot = (s_nextSlot + 1) % s_cache.size();
        slot = { .weapon = weapon, .root = weaponRoot, .revision = s_revision, .powerArmor = inPowerArmor, .used = true,
            .result = policy::resolve(s_entries, key, inPowerArmor) };
        if (slot.result) {
            ROCK_LOG_INFO(Weapon, "FRIK user weapon offset applies to {:08X} key='{}' powerArmor={} weapon={} primaryHand={} scale={:.4f}",
                weapon->formID, key, inPowerArmor, slot.result->hasWeaponOffset, slot.result->hasPrimaryHandRotation, slot.result->weaponScale());
        }
        return slot.result;
    }

    RE::NiTransform seat(const Override& override, const RE::NiTransform& nativeHandInWeapon)
    {
        return holding_grip_math::toNative(policy::seat(override, holding_grip_math::fromNative(nativeHandInWeapon)));
    }
}
