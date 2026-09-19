#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <optional>
#include <string_view>

namespace rock::native_idle_grip_preharvest_policy
{
    inline constexpr std::size_t kFirstPersonGraphIndex = 1;
    inline constexpr std::uint32_t kAnimationResourceStateMask = 0x70000000u;
    inline constexpr unsigned kAnimationResourceStateShift = 28;
    inline constexpr std::array<float, 5> kPersistenceSampleFractions{ 0.0f, 0.2f, 0.4f, 0.6f, 0.8f };
    inline constexpr float kMinimumPersistenceDurationSeconds = 0.05f;
    inline constexpr float kMaximumPersistenceDurationSeconds = 600.0f;
    inline constexpr float kMaximumStableHandTranslationDelta = 0.05f;
    inline constexpr float kMaximumStableHandRotationDeltaDegrees = 0.5f;
    inline constexpr float kMaximumStableFingerTranslationDelta = 0.02f;
    inline constexpr float kMaximumStableFingerRotationDeltaDegrees = 1.0f;
    inline constexpr float kMaximumStableScaleDelta = 0.001f;
    /*
     * The support hand is not rigidly attached to the Weapon bone, so an
     * idle's breathing sway moves it relative to the weapon across the
     * clip. Tolerate the same sway the runtime value witness accepts
     * (1gu, ~3deg); a support arm that actually travels across the idle
     * fails closed and leaves the record without a support relation.
     */
    inline constexpr float kMaximumStableSupportHandTranslationDelta = 1.0f;
    inline constexpr float kMaximumStableSupportHandRotationDeltaDegrees = 3.0f;
    inline constexpr std::size_t kMaxBoneChainLength = 64;

    enum class IdleClipPriority : std::uint8_t
    {
        None,
        GenericIdle,
        Idle,
        IdleReady,
    };

    inline constexpr std::array kIdleClipSearchOrder{
        IdleClipPriority::IdleReady,
        IdleClipPriority::Idle,
        IdleClipPriority::GenericIdle,
    };

    struct FirstPersonSelection
    {
        bool valid{ false };
        std::size_t graphIndex{ 0 };
    };

    /*
     * A sole-form fallback is sufficient while the candidate graph is still
     * generic (variant key zero), but a resolved nonzero stock variant must be
     * harvested under its own exact key. A live-equipped fallback never
     * suppresses native-idle work because it cannot carry the exact finger
     * locals.
     */
    [[nodiscard]] constexpr bool shouldStartNativeIdleHarvest(
        const bool lookupFound,
        const bool lookupIsNativeIdle,
        const bool lookupUsedVariantFallback,
        const std::uint64_t candidateVariantKey) noexcept
    {
        if (!lookupFound || !lookupIsNativeIdle) {
            return true;
        }
        return lookupUsedVariantFallback && candidateVariantKey != 0;
    }

    /*
     * Bethesda builds the background actor manager in paired graph order:
     * third person first, first person second. RequestAnimationSubGraph visits
     * those same graphs in order and appends matching handles and identifiers
     * in lockstep.
     * Never fall back to graph zero: that would silently harvest a flat/third-
     * person relation instead of the FO4VR first-person grip.
     */
    [[nodiscard]] constexpr FirstPersonSelection selectFirstPersonGraph(
        const std::size_t graphCount,
        const std::size_t handleCount,
        const std::size_t identifierCount) noexcept
    {
        if (graphCount <= kFirstPersonGraphIndex || handleCount <= kFirstPersonGraphIndex || identifierCount <= kFirstPersonGraphIndex) {
            return {};
        }
        return FirstPersonSelection{
            .valid = true,
            .graphIndex = kFirstPersonGraphIndex,
        };
    }

    /*
     * hkaAnimationBinding uses an empty transformTrackToBoneIndices array to
     * mean identity mapping. A non-empty mapping must cover every sampled
     * transform track; truncated or malformed bindings fail closed.
     */
    [[nodiscard]] constexpr int findTransformTrackForBone(const int boneIndex, const int transformTrackCount,
        const std::span<const std::int16_t> transformTrackToBoneIndices) noexcept
    {
        if (boneIndex < 0 || transformTrackCount <= 0) {
            return -1;
        }
        if (transformTrackToBoneIndices.empty()) {
            return boneIndex < transformTrackCount ? boneIndex : -1;
        }
        if (transformTrackToBoneIndices.size() < static_cast<std::size_t>(transformTrackCount)) {
            return -1;
        }
        for (int trackIndex = 0; trackIndex < transformTrackCount; ++trackIndex) {
            if (transformTrackToBoneIndices[static_cast<std::size_t>(trackIndex)] == boneIndex) {
                return trackIndex;
            }
        }
        return -1;
    }

    [[nodiscard]] constexpr bool weaponIsDirectChildOfHand(const int weaponBoneIndex, const int handBoneIndex, const std::span<const std::int16_t> parentIndices) noexcept
    {
        return weaponBoneIndex >= 0 && handBoneIndex >= 0 && static_cast<std::size_t>(weaponBoneIndex) < parentIndices.size() &&
            parentIndices[static_cast<std::size_t>(weaponBoneIndex)] == handBoneIndex;
    }

    /*
     * AnimationFileManagerSingleton only dereferences a BShkbHkxDB entry's
     * BSAnimationDBData pointer in resource states 3 and 4. Mirror that exact
     * native gate before ROCK inspects the retained off-screen idle handle.
     */
    [[nodiscard]] constexpr std::uint32_t animationResourceState(const std::uint32_t flags) noexcept
    {
        return (flags & kAnimationResourceStateMask) >> kAnimationResourceStateShift;
    }

    [[nodiscard]] constexpr bool animationResourceCanExposeData(const std::uint32_t flags) noexcept
    {
        const auto state = animationResourceState(flags);
        return state == 3u || state == 4u;
    }

    [[nodiscard]] constexpr bool clipPathHasStem(const std::string_view path, const std::string_view expectedStem) noexcept
    {
        const auto separator = path.find_last_of("/\\");
        const auto stemBegin = separator == std::string_view::npos ? 0 : separator + 1;
        const auto extension = path.find_last_of('.');
        const auto stemEnd = extension == std::string_view::npos || extension < stemBegin ? path.size() : extension;
        if (stemEnd - stemBegin != expectedStem.size()) {
            return false;
        }

        const auto asciiLower = [](const char value) { return value >= 'A' && value <= 'Z' ? static_cast<char>(value + ('a' - 'A')) : value; };
        for (std::size_t index = 0; index < expectedStem.size(); ++index) {
            if (asciiLower(path[stemBegin + index]) != asciiLower(expectedStem[index])) {
                return false;
            }
        }
        return true;
    }

    [[nodiscard]] constexpr IdleClipPriority idleClipPriority(const std::string_view path) noexcept
    {
        if (clipPathHasStem(path, "WPNIdleReady")) {
            return IdleClipPriority::IdleReady;
        }
        if (clipPathHasStem(path, "WPNIdle")) {
            return IdleClipPriority::Idle;
        }
        // First-person melee subgraphs use Idle.hkx (1HM, 2HM, Board).
        // Callers search only the requested weapon's first-person subgraph;
        // a weapon-specific WPN idle still takes precedence when present.
        if (clipPathHasStem(path, "Idle")) {
            return IdleClipPriority::GenericIdle;
        }
        return IdleClipPriority::None;
    }

    [[nodiscard]] constexpr bool sameClipPath(const std::string_view left, const std::string_view right) noexcept
    {
        if (left.size() != right.size()) {
            return false;
        }

        const auto normalize = [](const char value) {
            if (value == '/') {
                return '\\';
            }
            return value >= 'A' && value <= 'Z' ? static_cast<char>(value + ('a' - 'A')) : value;
        };
        for (std::size_t index = 0; index < left.size(); ++index) {
            if (normalize(left[index]) != normalize(right[index])) {
                return false;
            }
        }
        return true;
    }

    [[nodiscard]] constexpr float persistenceSampleTimeSeconds(const float durationSeconds, const std::size_t sampleIndex) noexcept
    {
        return sampleIndex < kPersistenceSampleFractions.size() ? durationSeconds * kPersistenceSampleFractions[sampleIndex] : 0.0f;
    }

    [[nodiscard]] constexpr bool stableForPersistence(
        const std::size_t sampleCount,
        const float durationSeconds,
        const float maxHandTranslationDelta,
        const float maxHandRotationDeltaDegrees,
        const float maxFingerTranslationDelta,
        const float maxFingerRotationDeltaDegrees,
        const float maxScaleDelta) noexcept
    {
        return sampleCount == kPersistenceSampleFractions.size() &&
               durationSeconds >= kMinimumPersistenceDurationSeconds &&
               durationSeconds <= kMaximumPersistenceDurationSeconds &&
               maxHandTranslationDelta <= kMaximumStableHandTranslationDelta &&
               maxHandRotationDeltaDegrees <= kMaximumStableHandRotationDeltaDegrees &&
               maxFingerTranslationDelta <= kMaximumStableFingerTranslationDelta &&
               maxFingerRotationDeltaDegrees <= kMaximumStableFingerRotationDeltaDegrees &&
               maxScaleDelta <= kMaximumStableScaleDelta;
    }

    [[nodiscard]] constexpr bool supportStableForPersistence(
        const std::size_t sampleCount,
        const float durationSeconds,
        const float maxSupportHandTranslationDelta,
        const float maxSupportHandRotationDeltaDegrees,
        const float maxSupportFingerTranslationDelta,
        const float maxSupportFingerRotationDeltaDegrees,
        const float maxSupportScaleDelta) noexcept
    {
        return sampleCount == kPersistenceSampleFractions.size() &&
               durationSeconds >= kMinimumPersistenceDurationSeconds &&
               durationSeconds <= kMaximumPersistenceDurationSeconds &&
               maxSupportHandTranslationDelta <= kMaximumStableSupportHandTranslationDelta &&
               maxSupportHandRotationDeltaDegrees <= kMaximumStableSupportHandRotationDeltaDegrees &&
               maxSupportFingerTranslationDelta <= kMaximumStableFingerTranslationDelta &&
               maxSupportFingerRotationDeltaDegrees <= kMaximumStableFingerRotationDeltaDegrees &&
               maxSupportScaleDelta <= kMaximumStableScaleDelta;
    }

    /*
     * Walk hkaSkeleton parent indices from a bone to its root, leaf first.
     * The support relation is composed from graph-local bone transforms
     * along both arm chains, exactly like the live capture composes
     * logical model transforms; the shared ancestors cancel in the
     * inverse(primary) * support product. Fails closed (length 0) on an
     * out-of-range parent, a chain longer than the caller's buffer, or a
     * cycle, so a malformed skeleton can never drive an unbounded walk.
     */
    [[nodiscard]] constexpr std::size_t collectBoneChainToRoot(
        const int leafBoneIndex,
        const std::span<const std::int16_t> parentIndices,
        const std::span<int> outChain) noexcept
    {
        std::size_t length = 0;
        int current = leafBoneIndex;
        while (current >= 0) {
            if (static_cast<std::size_t>(current) >= parentIndices.size() ||
                length >= outChain.size() ||
                length >= parentIndices.size()) {
                return 0;
            }
            outChain[length++] = current;
            current = parentIndices[static_cast<std::size_t>(current)];
        }
        return length;
    }

    // Common animated ancestors (spine/root) do not author an offhand grip.
    [[nodiscard]] constexpr std::optional<bool> supportBranchHasAnimation(int primaryHand,
        int supportHand, std::span<const std::int16_t> parents, int trackCount,
        std::span<const std::int16_t> mapping) noexcept
    {
        std::array<int, kMaxBoneChainLength> primary{}, support{};
        const auto primaryCount = collectBoneChainToRoot(primaryHand, parents, primary);
        const auto supportCount = collectBoneChainToRoot(supportHand, parents, support);
        if (!primaryCount || !supportCount || primary[primaryCount - 1] != support[supportCount - 1] ||
            trackCount <= 0 || (!mapping.empty() && mapping.size() < static_cast<std::size_t>(trackCount))) return {};
        for (std::size_t i = 0; i < supportCount; ++i) {
            for (std::size_t j = 0; j < primaryCount; ++j) {
                if (support[i] == primary[j]) return false;
            }
            if (findTransformTrackForBone(support[i], trackCount, mapping) >= 0) return true;
        }
        return {};
    }
}
