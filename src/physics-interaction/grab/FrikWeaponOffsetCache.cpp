#include "physics-interaction/grab/FrikWeaponOffsetCache.h"

#include "physics-interaction/PhysicsLog.h"

#include "common/CommonUtils.h"
#include "f4vr/F4VRUtils.h"

#include "RE/Bethesda/FormComponents.h"
#include "RE/Bethesda/TESBoundObjects.h"

#include <nlohmann/json.hpp>

#include <filesystem>
#include <fstream>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>

namespace rock::frik_weapon_offset_cache
{
    namespace
    {
        using json = nlohmann::json;

        constexpr WORD kFrikWeaponOffsetResourceFirst = 200;
        constexpr WORD kFrikWeaponOffsetResourceLast = 600;
        constexpr auto kFrikModuleName = "FRIK.dll";
        constexpr auto kFrikWeaponOffsetsRelativePath = R"(\My Games\Fallout4VR\FRIK_Config\Weapons_Offsets)";

        struct CacheState
        {
            std::unordered_map<std::string, RE::NiTransform> offsets;
            bool loaded = false;
            std::size_t embeddedCount = 0;
            std::size_t customCount = 0;
        };

        std::mutex g_cacheMutex;
        CacheState g_cache;

        [[nodiscard]] bool hasText(std::string_view text)
        {
            return !text.empty();
        }

        [[nodiscard]] std::optional<std::string> readFrikResourceString(WORD resourceId)
        {
            const HMODULE module = GetModuleHandleA(kFrikModuleName);
            if (!module) {
                return std::nullopt;
            }

            const HRSRC resource = FindResourceA(module, MAKEINTRESOURCEA(resourceId), MAKEINTRESOURCEA(10));
            if (!resource) {
                return std::nullopt;
            }

            const HGLOBAL data = LoadResource(module, resource);
            if (!data) {
                return std::nullopt;
            }

            const DWORD size = SizeofResource(module, resource);
            const void* bytes = LockResource(data);
            if (!bytes || size == 0) {
                return std::nullopt;
            }

            return std::string(static_cast<const char*>(bytes), size);
        }

        [[nodiscard]] bool loadOffsetJsonToMap(
            const json& root,
            std::unordered_map<std::string, RE::NiTransform>& offsets)
        {
            bool loadedAny = false;
            for (const auto& [key, value] : root.items()) {
                const auto rotation = value.find("rotation");
                if (rotation == value.end() || !rotation->is_array() || rotation->size() < 12) {
                    continue;
                }

                RE::NiTransform transform{};
                for (int row = 0; row < 3; ++row) {
                    for (int column = 0; column < 4; ++column) {
                        transform.rotate[row][column] = (*rotation)[row * 4 + column].get<float>();
                    }
                }
                transform.translate.x = value.at("x").get<float>();
                transform.translate.y = value.at("y").get<float>();
                transform.translate.z = value.at("z").get<float>();
                transform.scale = value.at("scale").get<float>();

                offsets[key] = transform;
                loadedAny = true;
            }
            return loadedAny;
        }

        [[nodiscard]] bool loadOffsetJsonString(
            const std::string& text,
            std::unordered_map<std::string, RE::NiTransform>& offsets)
        {
            try {
                return loadOffsetJsonToMap(json::parse(text), offsets);
            } catch (const std::exception& e) {
                ROCK_LOG_WARN(Hand, "FRIK weapon offset JSON parse failed: {}", e.what());
                return false;
            }
        }

        [[nodiscard]] bool loadOffsetJsonFile(
            const std::filesystem::path& path,
            std::unordered_map<std::string, RE::NiTransform>& offsets)
        {
            try {
                std::ifstream input(path, std::ios::in);
                if (!input) {
                    return false;
                }
                json parsed;
                input >> parsed;
                return loadOffsetJsonToMap(parsed, offsets);
            } catch (const std::exception& e) {
                ROCK_LOG_WARN(Hand, "FRIK weapon offset file '{}' load failed: {}", path.string(), e.what());
                return false;
            }
        }

        [[nodiscard]] std::size_t loadEmbeddedOffsets(std::unordered_map<std::string, RE::NiTransform>& offsets)
        {
            std::size_t loadedCount = 0;
            for (WORD resourceId = kFrikWeaponOffsetResourceFirst; resourceId <= kFrikWeaponOffsetResourceLast; ++resourceId) {
                const auto resource = readFrikResourceString(resourceId);
                if (!resource) {
                    continue;
                }
                const auto before = offsets.size();
                (void)loadOffsetJsonString(*resource, offsets);
                loadedCount += offsets.size() - before;
            }
            return loadedCount;
        }

        [[nodiscard]] std::size_t loadCustomOffsets(std::unordered_map<std::string, RE::NiTransform>& offsets)
        {
            const std::filesystem::path offsetDir = common::getRelativePathInDocuments(kFrikWeaponOffsetsRelativePath);
            std::error_code ec;
            if (!std::filesystem::exists(offsetDir, ec) || !std::filesystem::is_directory(offsetDir, ec)) {
                return 0;
            }

            std::size_t loadedCount = 0;
            for (const auto& entry : std::filesystem::directory_iterator(offsetDir, ec)) {
                if (ec) {
                    break;
                }
                if (!entry.is_regular_file(ec)) {
                    continue;
                }
                const auto before = offsets.size();
                if (loadOffsetJsonFile(entry.path(), offsets)) {
                    loadedCount += offsets.size() > before ? offsets.size() - before : 1;
                }
            }
            return loadedCount;
        }

        [[nodiscard]] std::string firstChildNameOfPGrip(const RE::NiAVObject* weaponRoot)
        {
            auto* mutableRoot = const_cast<RE::NiAVObject*>(weaponRoot);
            auto* grip = mutableRoot ? f4vr::findNode(mutableRoot, "P-Grip") : nullptr;
            auto* gripChild = grip ? f4vr::getFirstChild(grip) : nullptr;
            return gripChild ? std::string(gripChild->name.c_str()) : std::string{};
        }

        [[nodiscard]] std::string extendWeaponNameLikeFrik(std::string weaponName, const RE::NiAVObject* weaponRoot)
        {
            const std::string stockName = firstChildNameOfPGrip(weaponRoot);
            if (weaponName == "Plasma") {
                if (stockName.starts_with("RiotGrip") ||
                    stockName.starts_with("Sniper") ||
                    stockName.find("Rifle") != std::string::npos) {
                    return weaponName + " Rifle";
                }
            } else if (weaponName == "Pipe" || weaponName == "Pipe Bolt-Action") {
                if (stockName.starts_with("HandmadePaddedStock") ||
                    stockName.starts_with("SpringStock") ||
                    stockName.starts_with("PipeStock")) {
                    return weaponName + " Rifle";
                }
            } else if (weaponName == "Laser" || weaponName == "Institute") {
                if (stockName.find("Rifle") != std::string::npos) {
                    return weaponName + " Rifle";
                }
            }
            return weaponName;
        }

        [[nodiscard]] std::string weaponNodeOffsetKey(std::string_view weaponName, bool inPowerArmor, bool leftHanded)
        {
            // hFRIK Weapon mode has no mode suffix; PrimaryHand is a hand-node rotation offset, not a Weapon-node local transform.
            std::string key(weaponName);
            if (inPowerArmor) {
                key += "-PowerArmor";
            }
            if (leftHanded) {
                key += "-leftHanded";
            }
            return key;
        }

        [[nodiscard]] LookupResult findPrimaryWeaponOffsetLocked(
            const CacheState& cache,
            const std::string& weaponName)
        {
            if (!cache.loaded) {
                return LookupResult{ .found = false, .reason = "cacheNotLoaded" };
            }

            const bool inPowerArmor = f4vr::isInPowerArmor();
            const bool leftHanded = f4vr::isLeftHandedMode();
            if (inPowerArmor) {
                const auto paIt = cache.offsets.find(weaponNodeOffsetKey(weaponName, true, leftHanded));
                if (paIt != cache.offsets.end()) {
                    return LookupResult{ .found = true, .offset = paIt->second, .reason = "powerArmorOffset" };
                }
            }

            const auto it = cache.offsets.find(weaponNodeOffsetKey(weaponName, false, leftHanded));
            if (it != cache.offsets.end()) {
                return LookupResult{ .found = true, .offset = it->second, .reason = "offset" };
            }

            return LookupResult{ .found = false, .reason = "offsetMissing" };
        }
    }

    void preload()
    {
        CacheState next{};
        next.embeddedCount = loadEmbeddedOffsets(next.offsets);
        next.customCount = loadCustomOffsets(next.offsets);
        next.loaded = true;
        const std::size_t totalCount = next.offsets.size();
        const std::size_t embeddedCount = next.embeddedCount;
        const std::size_t customCount = next.customCount;

        {
            std::scoped_lock lock(g_cacheMutex);
            g_cache = std::move(next);
        }

        ROCK_LOG_INFO(Hand,
            "Loaded FRIK weapon offsets for loose weapon attach: entries={} embeddedLoaded={} customLoaded={}",
            totalCount,
            embeddedCount,
            customCount);
    }

    LookupResult findPrimaryWeaponOffset(
        const RE::TESObjectWEAP* weapon,
        const RE::NiAVObject* weaponRoot)
    {
        if (!weapon) {
            return LookupResult{ .found = false, .reason = "missingWeaponForm" };
        }

        const auto fullName = RE::TESFullName::GetFullName(*weapon, false);
        if (!hasText(fullName)) {
            return LookupResult{ .found = false, .reason = "missingWeaponName" };
        }

        const std::string weaponName = extendWeaponNameLikeFrik(std::string(fullName), weaponRoot);
        std::scoped_lock lock(g_cacheMutex);
        return findPrimaryWeaponOffsetLocked(g_cache, weaponName);
    }
}
