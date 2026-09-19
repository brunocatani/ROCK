#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>

namespace rock::authored_weapon_grip_cache
{
    // v4 records distinguish an absent animated support arm from failed capture.
    // Older records may contain reference-only support poses and are reharvested.
    inline constexpr std::uint32_t kFormatVersion = 4;
    // v2 rebuilds records whose weapon ID could be corrupted by the old
    // unconditional light-plugin index removal, including Fallout4.esm forms.
    inline constexpr std::uint32_t kPoseAlgorithmVersion = 2;
    inline constexpr std::size_t kFiringFingerCount = 15;
    inline constexpr std::uint32_t kRequiredPersistenceSamples = 5;
    inline constexpr std::uint16_t kCompleteFiringFingerMask = 0x7FFFu;
    inline constexpr std::size_t kMaximumCachedEntries = 2048;
    inline constexpr std::size_t kMaximumRecordBytes = 128 * 1024;

    [[nodiscard]] constexpr std::uint32_t localWeaponFormId(const std::uint32_t runtimeFormId) noexcept
    {
        const auto index = runtimeFormId >> 24;
        if (index == 0xFFu) return 0; // Runtime-created forms have no stable plugin identity.
        return runtimeFormId & (index == 0xFEu ? 0x0000'0FFFu : 0x00FF'FFFFu);
    }

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
        // Paired support-arm relation (LArm_Hand in Weapon plus left finger
        // locals) sampled from the same idle clip as the primary pose and
        // validated across the same persistence samples. Optional: a clip
        // whose support arm cannot be composed or travels across the idle
        // persists with supportValid=false.
        PersistedTransform supportHandWeaponLocal{};
        std::array<PersistedTransform, kFiringFingerCount> supportFingerLocals{};
        std::uint16_t supportFingerMask{ 0 };
        bool supportValid{ false };
        bool supportAbsent{ false };
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
