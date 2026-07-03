#include "physics-interaction/grab/FrikWeaponOffsetCache.h"

#include "physics-interaction/PhysicsLog.h"

#include "common/CommonUtils.h"
#include "f4vr/F4VRUtils.h"
#include "f4vr/PlayerNodes.h"

#include "RE/Bethesda/FormComponents.h"
#include "RE/Bethesda/TESBoundObjects.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace rock::frik_weapon_offset_cache
{
    namespace
    {
        using json = nlohmann::json;

        constexpr WORD kFrikWeaponOffsetResourceFirst = 200;
        constexpr WORD kFrikWeaponOffsetResourceLast = 600;
        constexpr auto kFrikModuleName = "FRIK.dll";
        constexpr auto kFrikWeaponOffsetsRelativePath = R"(\My Games\Fallout4VR\FRIK_Config\Weapons_Offsets)";
        constexpr auto kPowerArmorSuffix = "-PowerArmor";
        constexpr auto kLeftHandedSuffix = "-leftHanded";

        struct CustomOffsetsSignature
        {
            bool directoryExists = false;
            std::uint32_t fileCount = 0;
            std::int64_t latestWriteTick = 0;

            [[nodiscard]] bool operator==(const CustomOffsetsSignature&) const = default;
        };

        struct CacheState
        {
            std::unordered_map<std::string, RE::NiTransform> offsets;
            CustomOffsetsSignature customSignature{};
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

        [[nodiscard]] bool equalsIgnoreCase(std::string_view lhs, std::string_view rhs)
        {
            if (lhs.size() != rhs.size()) {
                return false;
            }
            for (std::size_t i = 0; i < lhs.size(); ++i) {
                const auto left = static_cast<unsigned char>(lhs[i]);
                const auto right = static_cast<unsigned char>(rhs[i]);
                if (std::tolower(left) != std::tolower(right)) {
                    return false;
                }
            }
            return true;
        }

        [[nodiscard]] std::filesystem::path customOffsetDirectory()
        {
            return common::getRelativePathInDocuments(kFrikWeaponOffsetsRelativePath);
        }

        [[nodiscard]] CustomOffsetsSignature scanCustomOffsetsSignature()
        {
            CustomOffsetsSignature signature{};
            const std::filesystem::path offsetDir = customOffsetDirectory();
            std::error_code ec;
            if (!std::filesystem::exists(offsetDir, ec) || !std::filesystem::is_directory(offsetDir, ec)) {
                return signature;
            }

            signature.directoryExists = true;
            for (const auto& entry : std::filesystem::directory_iterator(offsetDir, ec)) {
                if (ec) {
                    break;
                }

                std::error_code entryEc;
                if (!entry.is_regular_file(entryEc)) {
                    continue;
                }

                ++signature.fileCount;
                const auto writeTime = entry.last_write_time(entryEc);
                if (!entryEc) {
                    const auto ticks = static_cast<std::int64_t>(writeTime.time_since_epoch().count());
                    signature.latestWriteTick = (std::max)(signature.latestWriteTick, ticks);
                }
            }
            return signature;
        }

        [[nodiscard]] bool isFiniteNiTransform(const RE::NiTransform& value)
        {
            bool rotationFinite = true;
            for (std::uint32_t row = 0; row < 3; ++row) {
                for (std::uint32_t column = 0; column < 3; ++column) {
                    rotationFinite = rotationFinite && std::isfinite(value.rotate.entry[row][column]);
                }
            }
            return rotationFinite &&
                   std::isfinite(value.translate.x) &&
                   std::isfinite(value.translate.y) &&
                   std::isfinite(value.translate.z) &&
                   std::isfinite(value.scale) &&
                   value.scale > 0.0001f;
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
            const std::filesystem::path offsetDir = customOffsetDirectory();
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

        [[nodiscard]] std::optional<RE::NiTransform> liveWeaponNodeDefaultOffset()
        {
            auto* weaponNode = f4vr::getWeaponNode();
            if (!weaponNode || !isFiniteNiTransform(weaponNode->local)) {
                return std::nullopt;
            }
            return weaponNode->local;
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

        [[nodiscard]] std::string weaponOffsetKey(std::string_view weaponName, bool inPowerArmor, bool leftHanded)
        {
            std::string key(weaponName);
            if (inPowerArmor) {
                key += kPowerArmorSuffix;
            }
            if (leftHanded) {
                key += kLeftHandedSuffix;
            }
            return key;
        }

        [[nodiscard]] std::optional<RE::NiTransform> findOffsetByKeyLocked(
            const CacheState& cache,
            const std::string& key,
            bool allowCaseInsensitiveFallback)
        {
            const auto it = cache.offsets.find(key);
            if (it != cache.offsets.end()) {
                return it->second;
            }

            if (!allowCaseInsensitiveFallback) {
                return std::nullopt;
            }

            for (const auto& [storedKey, offset] : cache.offsets) {
                if (equalsIgnoreCase(storedKey, key)) {
                    return offset;
                }
            }
            return std::nullopt;
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
                const auto powerArmorOffset = findOffsetByKeyLocked(cache, weaponOffsetKey(weaponName, true, leftHanded), false);
                if (powerArmorOffset) {
                    return LookupResult{ .found = true, .offset = *powerArmorOffset, .reason = "powerArmorOffset" };
                }
            }

            const auto offset = findOffsetByKeyLocked(cache, weaponOffsetKey(weaponName, false, leftHanded), false);
            if (offset) {
                return LookupResult{ .found = true, .offset = *offset, .reason = "offset" };
            }

            return LookupResult{ .found = false, .reason = "offsetMissing" };
        }

        void refreshIfStale()
        {
            const auto currentSignature = scanCustomOffsetsSignature();
            bool shouldReload = false;
            {
                std::scoped_lock lock(g_cacheMutex);
                shouldReload = !g_cache.loaded || !(g_cache.customSignature == currentSignature);
            }

            if (shouldReload) {
                preload();
            }
        }
    }

    void preload()
    {
        CacheState next{};
        next.embeddedCount = loadEmbeddedOffsets(next.offsets);
        next.customCount = loadCustomOffsets(next.offsets);
        next.customSignature = scanCustomOffsetsSignature();
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
        refreshIfStale();

        LookupResult lookup{};
        {
            std::scoped_lock lock(g_cacheMutex);
            lookup = findPrimaryWeaponOffsetLocked(g_cache, weaponName);
        }
        if (lookup.found) {
            return lookup;
        }

        const auto defaultOffset = liveWeaponNodeDefaultOffset();
        if (defaultOffset) {
            return LookupResult{ .found = true, .offset = *defaultOffset, .reason = "defaultWeaponNodeLocal" };
        }
        return lookup;
    }

}
