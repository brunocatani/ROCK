#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>

namespace rock::authored_weapon_grip_cache
{
    inline constexpr std::uint32_t kFormatVersion = 2;
    inline constexpr std::uint32_t kPoseAlgorithmVersion = 1;
    inline constexpr std::size_t kFiringFingerCount = 15;
    inline constexpr std::uint32_t kRequiredPersistenceSamples = 5;
    inline constexpr std::uint16_t kCompleteFiringFingerMask = 0x7FFFu;
    inline constexpr std::size_t kMaximumCachedEntries = 2048;
    inline constexpr std::size_t kMaximumRecordBytes = 128 * 1024;

    struct StableFormIdentity
    {
        std::string plugin;
        std::uint32_t localFormId{ 0 };

        [[nodiscard]] bool valid() const noexcept;
        [[nodiscard]] bool operator==(const StableFormIdentity&) const = default;
    };

    struct CacheKey
    {
        StableFormIdentity weapon;
        std::uint64_t pGripVariantKey{ 0 };
        std::uint64_t instanceContentKey{ 0 };
        std::uint64_t graphProfileKey{ 0 };
        bool inPowerArmor{ false };

        [[nodiscard]] bool valid() const noexcept;
        [[nodiscard]] bool operator==(const CacheKey&) const = default;
    };

    struct PersistedTransform
    {
        std::array<float, 9> rotate{ 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f };
        std::array<float, 3> translate{};
        float scale{ 1.0f };
    };

    struct SampleQuality
    {
        std::uint32_t sampleCount{ 0 };
        float selectedTimeSeconds{ 0.0f };
        float durationSeconds{ 0.0f };
        float maxHandTranslationDelta{ 0.0f };
        float maxHandRotationDeltaDegrees{ 0.0f };
        float maxFingerTranslationDelta{ 0.0f };
        float maxFingerRotationDeltaDegrees{ 0.0f };
        float maxScaleDelta{ 0.0f };
        bool stable{ false };
    };

    struct CacheRecord
    {
        std::uint32_t formatVersion{ kFormatVersion };
        std::uint32_t poseAlgorithmVersion{ kPoseAlgorithmVersion };
        CacheKey key{};
        PersistedTransform rightHandWeaponLocal{};
        std::array<PersistedTransform, kFiringFingerCount> rightFiringFingerLocals{};
        std::uint16_t rightFiringFingerMask{ 0 };
        // Paired support-arm relation mirrored from a validated live capture
        // of Bethesda's native right-primary topology. Optional: records
        // persisted before that capture carry supportValid=false and gain
        // the relation on a later save.
        PersistedTransform supportHandWeaponLocal{};
        std::array<PersistedTransform, kFiringFingerCount> supportFingerLocals{};
        std::uint16_t supportFingerMask{ 0 };
        bool supportValid{ false };
        std::string idleClipPath;
        std::uint64_t requestedSubgraphIdentifier{ 0 };
        std::uint64_t bindingSubgraphIdentifier{ 0 };
        SampleQuality quality{};
        std::uint64_t checksum{ 0 };
    };

    struct CacheKeyHash
    {
        [[nodiscard]] std::size_t operator()(const CacheKey& key) const noexcept;
    };

    [[nodiscard]] bool validTransform(const PersistedTransform& transform) noexcept;
    [[nodiscard]] bool validRecord(const CacheRecord& record) noexcept;
    [[nodiscard]] std::uint64_t calculateChecksum(const CacheRecord& record) noexcept;
    [[nodiscard]] std::string serialize(const CacheRecord& record);
    [[nodiscard]] bool parse(std::string_view jsonText, CacheRecord& out, std::string* outError);
}
