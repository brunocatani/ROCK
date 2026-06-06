#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cmath>

namespace rock::generated_body_contact_registry
{
    inline constexpr std::uint32_t kInvalidBodyId = 0x7FFF'FFFFu;
    inline constexpr std::uint32_t kInvalidBodyIdAllBits = 0xFFFF'FFFFu;

    enum class GeneratedBodyKind : std::uint32_t
    {
        Unknown = 0,
        RightHand,
        LeftHand,
        Weapon,
        Body,
    };

    inline constexpr std::uint32_t kFlagPrimaryAnchor = 1u << 0;
    inline constexpr std::uint32_t kFlagSampledVelocity = 1u << 1;
    inline constexpr std::uint32_t kFlagPowerArmor = 1u << 2;

    struct Entry
    {
        std::uint32_t bodyId = kInvalidBodyId;
        GeneratedBodyKind kind = GeneratedBodyKind::Unknown;
        std::uint32_t role = 0;
        std::uint32_t partKind = 0;
        std::uint32_t subRole = 0;
        std::uint32_t socketRole = 0;
        std::uint32_t actionRole = 0;
        std::uint32_t gripPose = 0;
        std::uint32_t descriptorIndex = 0;
        std::uint32_t zone = 0;
        std::uint32_t side = 0;
        std::uint32_t flags = 0;
        std::uint64_t generationKey = 0;
        float sampledVelocityHavokX = 0.0f;
        float sampledVelocityHavokY = 0.0f;
        float sampledVelocityHavokZ = 0.0f;
        float lengthGameUnits = 0.0f;
        float radiusGameUnits = 0.0f;
    };

    struct Classification : Entry
    {
        bool valid = false;
    };

    [[nodiscard]] inline constexpr bool isValidBodyId(std::uint32_t bodyId) noexcept
    {
        return bodyId != kInvalidBodyId && bodyId != kInvalidBodyIdAllBits;
    }

    [[nodiscard]] inline constexpr bool hasFlag(std::uint32_t flags, std::uint32_t flag) noexcept
    {
        return (flags & flag) != 0;
    }

    [[nodiscard]] inline bool hasFiniteSampledVelocity(const Entry& entry) noexcept
    {
        return hasFlag(entry.flags, kFlagSampledVelocity) &&
               std::isfinite(entry.sampledVelocityHavokX) &&
               std::isfinite(entry.sampledVelocityHavokY) &&
               std::isfinite(entry.sampledVelocityHavokZ);
    }

    template <std::size_t Capacity>
    class Registry
    {
    public:
        static_assert(Capacity > 0);

        void clear() noexcept
        {
            beginPublication();
            _count.store(0, std::memory_order_release);
            for (auto& slot : _slots) {
                storeSlot(slot, Entry{});
            }
            endPublication();
        }

        void publish(const Entry* entries, std::size_t count)
        {
            std::array<Entry, Capacity> sorted{};
            std::size_t sortedCount = 0;
            const std::size_t inputCount = (std::min)(count, Capacity);
            for (std::size_t i = 0; i < inputCount; ++i) {
                if (!entries || !isValidBodyId(entries[i].bodyId) || entries[i].kind == GeneratedBodyKind::Unknown) {
                    continue;
                }
                sorted[sortedCount++] = entries[i];
            }

            std::sort(sorted.begin(), sorted.begin() + static_cast<std::ptrdiff_t>(sortedCount), [](const Entry& lhs, const Entry& rhs) {
                return lhs.bodyId < rhs.bodyId;
            });

            beginPublication();
            std::size_t publishedCount = 0;
            for (std::size_t i = 0; i < sortedCount;) {
                std::size_t next = i + 1;
                while (next < sortedCount && sorted[next].bodyId == sorted[i].bodyId) {
                    ++next;
                }

                if (next == i + 1 && publishedCount < Capacity) {
                    storeSlot(_slots[publishedCount], sorted[i]);
                    ++publishedCount;
                }
                i = next;
            }

            for (std::size_t i = publishedCount; i < Capacity; ++i) {
                storeSlot(_slots[i], Entry{});
            }
            _count.store(static_cast<std::uint32_t>(publishedCount), std::memory_order_release);
            endPublication();
        }

        [[nodiscard]] bool tryClassify(std::uint32_t bodyId, Classification& outClassification) const noexcept
        {
            outClassification = {};
            if (!isValidBodyId(bodyId)) {
                return false;
            }

            for (int attempt = 0; attempt < 4; ++attempt) {
                const std::uint64_t startVersion = _publicationVersion.load(std::memory_order_acquire);
                if ((startVersion & 1ull) != 0) {
                    continue;
                }

                const std::uint32_t count = (std::min)(_count.load(std::memory_order_acquire), static_cast<std::uint32_t>(Capacity));
                std::uint32_t low = 0;
                std::uint32_t high = count;
                bool found = false;
                Classification candidate{};

                while (low < high) {
                    const std::uint32_t mid = low + ((high - low) / 2u);
                    const std::uint32_t midBodyId = _slots[mid].bodyId.load(std::memory_order_acquire);
                    if (midBodyId < bodyId) {
                        low = mid + 1u;
                    } else if (midBodyId > bodyId) {
                        high = mid;
                    } else {
                        candidate = loadSlot(_slots[mid]);
                        found = true;
                        break;
                    }
                }

                const std::uint64_t endVersion = _publicationVersion.load(std::memory_order_acquire);
                if (startVersion != endVersion || (endVersion & 1ull) != 0) {
                    continue;
                }

                if (!found || candidate.bodyId != bodyId || candidate.kind == GeneratedBodyKind::Unknown) {
                    return false;
                }

                candidate.valid = true;
                outClassification = candidate;
                return true;
            }

            return false;
        }

        [[nodiscard]] std::uint32_t count() const noexcept
        {
            return (std::min)(_count.load(std::memory_order_acquire), static_cast<std::uint32_t>(Capacity));
        }

    private:
        struct AtomicSlot
        {
            std::atomic<std::uint32_t> bodyId{ kInvalidBodyId };
            std::atomic<std::uint32_t> kind{ static_cast<std::uint32_t>(GeneratedBodyKind::Unknown) };
            std::atomic<std::uint32_t> role{ 0 };
            std::atomic<std::uint32_t> partKind{ 0 };
            std::atomic<std::uint32_t> subRole{ 0 };
            std::atomic<std::uint32_t> socketRole{ 0 };
            std::atomic<std::uint32_t> actionRole{ 0 };
            std::atomic<std::uint32_t> gripPose{ 0 };
            std::atomic<std::uint32_t> descriptorIndex{ 0 };
            std::atomic<std::uint32_t> zone{ 0 };
            std::atomic<std::uint32_t> side{ 0 };
            std::atomic<std::uint32_t> flags{ 0 };
            std::atomic<std::uint64_t> generationKey{ 0 };
            std::atomic<float> sampledVelocityHavokX{ 0.0f };
            std::atomic<float> sampledVelocityHavokY{ 0.0f };
            std::atomic<float> sampledVelocityHavokZ{ 0.0f };
            std::atomic<float> lengthGameUnits{ 0.0f };
            std::atomic<float> radiusGameUnits{ 0.0f };
        };

        void beginPublication() noexcept
        {
            const std::uint64_t version = _publicationVersion.load(std::memory_order_relaxed);
            _publicationVersion.store((version & ~1ull) + 1ull, std::memory_order_release);
        }

        void endPublication() noexcept
        {
            const std::uint64_t version = _publicationVersion.load(std::memory_order_relaxed);
            _publicationVersion.store((version | 1ull) + 1ull, std::memory_order_release);
        }

        static void storeSlot(AtomicSlot& slot, const Entry& entry) noexcept
        {
            slot.kind.store(static_cast<std::uint32_t>(entry.kind), std::memory_order_release);
            slot.role.store(entry.role, std::memory_order_release);
            slot.partKind.store(entry.partKind, std::memory_order_release);
            slot.subRole.store(entry.subRole, std::memory_order_release);
            slot.socketRole.store(entry.socketRole, std::memory_order_release);
            slot.actionRole.store(entry.actionRole, std::memory_order_release);
            slot.gripPose.store(entry.gripPose, std::memory_order_release);
            slot.descriptorIndex.store(entry.descriptorIndex, std::memory_order_release);
            slot.zone.store(entry.zone, std::memory_order_release);
            slot.side.store(entry.side, std::memory_order_release);
            slot.flags.store(entry.flags, std::memory_order_release);
            slot.generationKey.store(entry.generationKey, std::memory_order_release);
            slot.sampledVelocityHavokX.store(entry.sampledVelocityHavokX, std::memory_order_release);
            slot.sampledVelocityHavokY.store(entry.sampledVelocityHavokY, std::memory_order_release);
            slot.sampledVelocityHavokZ.store(entry.sampledVelocityHavokZ, std::memory_order_release);
            slot.lengthGameUnits.store(entry.lengthGameUnits, std::memory_order_release);
            slot.radiusGameUnits.store(entry.radiusGameUnits, std::memory_order_release);
            slot.bodyId.store(entry.bodyId, std::memory_order_release);
        }

        static Classification loadSlot(const AtomicSlot& slot) noexcept
        {
            Classification classification{};
            classification.bodyId = slot.bodyId.load(std::memory_order_acquire);
            classification.kind = static_cast<GeneratedBodyKind>(slot.kind.load(std::memory_order_acquire));
            classification.role = slot.role.load(std::memory_order_acquire);
            classification.partKind = slot.partKind.load(std::memory_order_acquire);
            classification.subRole = slot.subRole.load(std::memory_order_acquire);
            classification.socketRole = slot.socketRole.load(std::memory_order_acquire);
            classification.actionRole = slot.actionRole.load(std::memory_order_acquire);
            classification.gripPose = slot.gripPose.load(std::memory_order_acquire);
            classification.descriptorIndex = slot.descriptorIndex.load(std::memory_order_acquire);
            classification.zone = slot.zone.load(std::memory_order_acquire);
            classification.side = slot.side.load(std::memory_order_acquire);
            classification.flags = slot.flags.load(std::memory_order_acquire);
            classification.generationKey = slot.generationKey.load(std::memory_order_acquire);
            classification.sampledVelocityHavokX = slot.sampledVelocityHavokX.load(std::memory_order_acquire);
            classification.sampledVelocityHavokY = slot.sampledVelocityHavokY.load(std::memory_order_acquire);
            classification.sampledVelocityHavokZ = slot.sampledVelocityHavokZ.load(std::memory_order_acquire);
            classification.lengthGameUnits = slot.lengthGameUnits.load(std::memory_order_acquire);
            classification.radiusGameUnits = slot.radiusGameUnits.load(std::memory_order_acquire);
            return classification;
        }

        std::array<AtomicSlot, Capacity> _slots{};
        std::atomic<std::uint32_t> _count{ 0 };
        std::atomic<std::uint64_t> _publicationVersion{ 0 };
    };
}
