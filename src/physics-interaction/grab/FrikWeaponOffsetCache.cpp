#include "physics-interaction/grab/FrikWeaponOffsetCache.h"

#include "physics-interaction/PhysicsLog.h"

#include "rock_support/Fo4VrRuntime.h"
#include "rock_support/ResourceUtils.h"

#include "RE/Bethesda/FormComponents.h"
#include "RE/Bethesda/TESBoundObjects.h"

#include <nlohmann/json.hpp>
#include <thomasmonkman-filewatch/FileWatch.hpp>

#include <array>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
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
        constexpr auto kPrimaryHandSuffix = "-primHand";
        constexpr auto kOffHandSuffix = "-offHand";
        constexpr auto kPowerArmorSuffix = "-PowerArmor";
        constexpr auto kLeftHandedSuffix = "-leftHanded";

        enum class GripOffsetMode : std::uint8_t
        {
            Weapon,
            PrimaryHand,
            OffHand,
        };

        constexpr std::array kGripOffsetModes{
            GripOffsetMode::Weapon,
            GripOffsetMode::PrimaryHand,
            GripOffsetMode::OffHand,
        };

        struct CacheState
        {
            struct CachedOffset
            {
                RE::NiTransform transform{};
                OffsetSource source{ OffsetSource::None };
            };

            std::unordered_map<std::string, CachedOffset> offsets;
            bool loaded = false;
            std::size_t embeddedCount = 0;
            std::size_t customCount = 0;
        };

        std::mutex g_cacheMutex;
        std::mutex g_reloadMutex;
        CacheState g_cache;
        std::atomic<std::uint64_t> g_cacheRevision{ 0 };
        std::unique_ptr<filewatch::FileWatch<std::string>> g_customOffsetWatch;

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
            return rock::resources::getPathInDocuments(kFrikWeaponOffsetsRelativePath);
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
            std::unordered_map<std::string, CacheState::CachedOffset>& offsets,
            const OffsetSource source)
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

                if (!isFiniteNiTransform(transform)) {
                    ROCK_LOG_WARN(Hand, "FRIK weapon offset '{}' ignored because its transform is invalid", key);
                    continue;
                }

                offsets[key] = CacheState::CachedOffset{
                    .transform = transform,
                    .source = source,
                };
                loadedAny = true;
            }
            return loadedAny;
        }

        [[nodiscard]] bool loadOffsetJsonString(
            const std::string& text,
            std::unordered_map<std::string, CacheState::CachedOffset>& offsets,
            const OffsetSource source)
        {
            try {
                return loadOffsetJsonToMap(json::parse(text), offsets, source);
            } catch (const std::exception& e) {
                ROCK_LOG_WARN(Hand, "FRIK weapon offset JSON parse failed: {}", e.what());
                return false;
            }
        }

        [[nodiscard]] bool loadOffsetJsonFile(
            const std::filesystem::path& path,
            std::unordered_map<std::string, CacheState::CachedOffset>& offsets,
            const OffsetSource source)
        {
            try {
                std::ifstream input(path, std::ios::in);
                if (!input) {
                    return false;
                }
                json parsed;
                input >> parsed;
                return loadOffsetJsonToMap(parsed, offsets, source);
            } catch (const std::exception& e) {
                ROCK_LOG_WARN(Hand, "FRIK weapon offset file '{}' load failed: {}", path.string(), e.what());
                return false;
            }
        }

        [[nodiscard]] std::size_t loadEmbeddedOffsets(
            std::unordered_map<std::string, CacheState::CachedOffset>& offsets)
        {
            std::size_t loadedCount = 0;
            for (WORD resourceId = kFrikWeaponOffsetResourceFirst; resourceId <= kFrikWeaponOffsetResourceLast; ++resourceId) {
                const auto resource = readFrikResourceString(resourceId);
                if (!resource) {
                    continue;
                }
                const auto before = offsets.size();
                (void)loadOffsetJsonString(*resource, offsets, OffsetSource::EmbeddedResource);
                loadedCount += offsets.size() - before;
            }
            return loadedCount;
        }

        [[nodiscard]] std::size_t loadCustomOffsets(
            std::unordered_map<std::string, CacheState::CachedOffset>& offsets)
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
                if (loadOffsetJsonFile(entry.path(), offsets, OffsetSource::CustomFile)) {
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

        [[nodiscard]] std::string_view gripOffsetModeSuffix(const GripOffsetMode mode)
        {
            switch (mode) {
            case GripOffsetMode::PrimaryHand:
                return kPrimaryHandSuffix;
            case GripOffsetMode::OffHand:
                return kOffHandSuffix;
            case GripOffsetMode::Weapon:
            default:
                return {};
            }
        }

        [[nodiscard]] std::string weaponOffsetKey(
            std::string_view weaponName,
            const GripOffsetMode mode,
            bool inPowerArmor,
            bool leftHanded)
        {
            std::string key(weaponName);
            key += gripOffsetModeSuffix(mode);
            if (inPowerArmor) {
                key += kPowerArmorSuffix;
            }
            if (leftHanded) {
                key += kLeftHandedSuffix;
            }
            return key;
        }

        [[nodiscard]] std::optional<CacheState::CachedOffset> findOffsetByKeyLocked(
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

        [[nodiscard]] std::optional<CacheState::CachedOffset> findEffectiveGripOffsetLocked(
            const CacheState& cache,
            const std::string& weaponName,
            const GripOffsetMode mode,
            const bool inPowerArmor,
            const bool leftHanded)
        {
            if (inPowerArmor) {
                const auto powerArmorOffset = findOffsetByKeyLocked(
                    cache,
                    weaponOffsetKey(weaponName, mode, true, leftHanded),
                    false);
                if (powerArmorOffset) {
                    return powerArmorOffset;
                }
            }

            return findOffsetByKeyLocked(
                cache,
                weaponOffsetKey(weaponName, mode, false, leftHanded),
                false);
        }

        [[nodiscard]] LookupResult findPrimaryWeaponOffsetLocked(
            const CacheState& cache,
            const std::string& weaponName)
        {
            if (!cache.loaded) {
                return LookupResult{ .found = false, .reason = "cacheNotLoaded" };
            }

            const bool inPowerArmor = f4vr::isInPowerArmor();
            constexpr bool leftHanded = false;
            const auto offset = findEffectiveGripOffsetLocked(
                cache,
                weaponName,
                GripOffsetMode::Weapon,
                inPowerArmor,
                leftHanded);
            if (offset) {
                return LookupResult{
                    .found = true,
                    .offset = offset->transform,
                    .source = offset->source,
                    .reason = offset->source == OffsetSource::CustomFile ? "customWeaponOffset" : "embeddedWeaponOffset",
                };
            }

            return LookupResult{ .found = false, .reason = "offsetMissing" };
        }

        [[nodiscard]] CustomGripOverrideResult findCustomGripOverrideLocked(
            const CacheState& cache,
            const std::string& weaponName)
        {
            if (!cache.loaded) {
                return CustomGripOverrideResult{ .found = false, .reason = "cacheNotLoaded" };
            }

            const bool inPowerArmor = f4vr::isInPowerArmor();
            constexpr bool leftHanded = false;
            for (const auto mode : kGripOffsetModes) {
                const auto offset = findEffectiveGripOffsetLocked(
                    cache,
                    weaponName,
                    mode,
                    inPowerArmor,
                    leftHanded);
                if (!offset || offset->source != OffsetSource::CustomFile) {
                    continue;
                }

                switch (mode) {
                case GripOffsetMode::PrimaryHand:
                    return CustomGripOverrideResult{ .found = true, .reason = "customPrimaryHandOffset" };
                case GripOffsetMode::OffHand:
                    return CustomGripOverrideResult{ .found = true, .reason = "customOffHandOffset" };
                case GripOffsetMode::Weapon:
                default:
                    return CustomGripOverrideResult{ .found = true, .reason = "customWeaponOffset" };
                }
            }

            return CustomGripOverrideResult{ .found = false, .reason = "customGripOffsetMissing" };
        }

        void reloadCache()
        {
            // Startup and filewatch callbacks may overlap during teardown or
            // rapid config churn. Serialize source reads, then publish one
            // complete immutable-by-convention snapshot under the query lock.
            std::scoped_lock reloadLock(g_reloadMutex);

            CacheState next{};
            next.embeddedCount = loadEmbeddedOffsets(next.offsets);
            next.customCount = loadCustomOffsets(next.offsets);
            next.loaded = true;
            const std::size_t totalCount = next.offsets.size();
            const std::size_t embeddedCount = next.embeddedCount;
            const std::size_t customCount = next.customCount;

            {
                std::scoped_lock cacheLock(g_cacheMutex);
                g_cache = std::move(next);
            }
            const auto revision = g_cacheRevision.fetch_add(1, std::memory_order_acq_rel) + 1;

            ROCK_LOG_INFO(Hand,
                "Loaded FRIK weapon offsets: entries={} embeddedLoaded={} customLoaded={} revision={}",
                totalCount,
                embeddedCount,
                customCount,
                revision);
        }

        void startCustomOffsetWatch()
        {
            if (g_customOffsetWatch) {
                return;
            }

            const auto offsetDir = customOffsetDirectory();
            std::error_code ec;
            if (!std::filesystem::exists(offsetDir, ec) || !std::filesystem::is_directory(offsetDir, ec)) {
                ROCK_LOG_WARN(Hand,
                    "FRIK custom weapon-offset watch unavailable: directory '{}' does not exist",
                    offsetDir.string());
                return;
            }

            try {
                g_customOffsetWatch = std::make_unique<filewatch::FileWatch<std::string>>(
                    offsetDir.string(),
                    [](const std::string&, const filewatch::Event changeType) {
                        if (changeType != filewatch::Event::modified &&
                            changeType != filewatch::Event::added &&
                            changeType != filewatch::Event::removed &&
                            changeType != filewatch::Event::renamed_new &&
                            changeType != filewatch::Event::renamed_old) {
                            return;
                        }

                        // FileWatch invokes callbacks on its own worker.
                        // JSON/resource I/O and parsing stay off every game,
                        // animation, input, and physics callback thread.
                        std::this_thread::sleep_for(std::chrono::milliseconds(150));
                        reloadCache();
                    });
                ROCK_LOG_INFO(Hand, "Watching hFRIK custom weapon offsets at '{}'", offsetDir.string());
            } catch (const std::exception& e) {
                ROCK_LOG_WARN(Hand,
                    "FRIK custom weapon-offset watch failed for '{}': {}",
                    offsetDir.string(),
                    e.what());
            }
        }
    }

    void preload()
    {
        reloadCache();
        startCustomOffsetWatch();
    }

    std::uint64_t currentRevision() noexcept
    {
        return g_cacheRevision.load(std::memory_order_acquire);
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
            return LookupResult{
                .found = true,
                .offset = *defaultOffset,
                .source = OffsetSource::LiveWeaponNodeFallback,
                .reason = "defaultWeaponNodeLocal",
            };
        }
        return lookup;
    }

    CustomGripOverrideResult findCustomGripOverride(
        const RE::TESObjectWEAP* weapon,
        const RE::NiAVObject* weaponRoot)
    {
        if (!weapon) {
            return CustomGripOverrideResult{ .found = false, .reason = "missingWeaponForm" };
        }

        const auto fullName = RE::TESFullName::GetFullName(*weapon, false);
        if (!hasText(fullName)) {
            return CustomGripOverrideResult{ .found = false, .reason = "missingWeaponName" };
        }

        const std::string weaponName = extendWeaponNameLikeFrik(std::string(fullName), weaponRoot);
        std::scoped_lock lock(g_cacheMutex);
        return findCustomGripOverrideLocked(g_cache, weaponName);
    }

}
